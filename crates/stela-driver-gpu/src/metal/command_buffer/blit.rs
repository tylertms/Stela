//! `MTLBlitCommandEncoder`.
//!
//! Copies map to Vulkan transfer commands with Metal's byte strides
//! converted to Vulkan's texel strides, or, for a texture whose image stores
//! another encoding than its format's bytes, to the texel kernels, which
//! convert each texel through a raw storage view; fills split unaligned edges into
//! copies from upload memory; mipmap generation is a chain of linear blits.
//! Hints with no Vulkan counterpart in the general image layout
//! (`optimizeContentsFor*Access`) record nothing, and fences need no
//! commands because the hazard tracker orders the queue.
use super::{BufferRef, CommandBuffer, Encoder, MappedRange, Resolver};
use crate::metal::{
    abi::BlitOption,
    binding::{ReorderArguments, ResourceId, TexelArguments},
    error::{Error, Result, ensure, require},
    hazard::Use,
    pixel_format::{Description, Encoding, Kind},
    resource::TextureDescriptor,
    vulkan::{
        Access, Filter, ImageAspects, PipelineStages,
        command::{BufferCopy, BufferImageCopy, Command, ImageBlit, ImageCopy, Subresource},
        object::{Buffer, Image},
    },
};
use alloc::vec::Vec;

pub(super) const TRANSFER: Use = Use::new(
    PipelineStages::ALL_TRANSFER,
    Access::TRANSFER_READ.union(Access::TRANSFER_WRITE),
);

/// A texture as a blit sees it.
#[derive(Clone, Copy, Debug)]
pub struct BlitTexture<'a> {
    pub image: Image,
    pub descriptor: &'a TextureDescriptor,
    /// For a buffer copy of a texture whose image stores another encoding,
    /// a raw storage view of the copied level's slice, which the texel
    /// kernels convert through; [`ResourceId::NULL`] otherwise.
    pub storage: ResourceId,
    /// A decoded format's blocks in the CPU copy's layout
    /// ([`CpuAccess::Blocks`](crate::metal::resource::CpuAccess::Blocks)),
    /// which copies read and write in place of its image.
    pub blocks: Option<BufferRef>,
}

/// A texel region of one slice and level: `MTLOrigin` and `MTLSize`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Region {
    pub slice: u64,
    pub level: u64,
    pub origin: [u64; 3],
    pub size: [u64; 3],
}

impl Region {
    /// Validates the region of `descriptor`'s texture and returns its
    /// subresource, offset and extent.
    pub fn resolve(
        &self,
        descriptor: &TextureDescriptor,
        aspects: ImageAspects,
    ) -> Result<(Subresource, [i32; 3], [u32; 3])> {
        ensure(
            self.level < descriptor.mipmap_level_count,
            "level beyond the texture's mipmaps",
        )?;
        ensure(
            self.slice < descriptor.slice_count(),
            "slice beyond the texture",
        )?;
        let size = descriptor.level_extent(self.level);
        let block = descriptor.pixel_format.describe().block;
        let block_size = [u64::from(block.width), u64::from(block.height), 1];
        for axis in 0..3 {
            let end = self.origin[axis].checked_add(self.size[axis]);
            ensure(
                end.is_some_and(|end| end <= size[axis]),
                "region extends past the texture",
            )?;
            ensure(
                self.origin[axis].is_multiple_of(block_size[axis]),
                "regions start on block boundaries",
            )?;
            ensure(
                self.size[axis].is_multiple_of(block_size[axis])
                    || self.origin[axis] + self.size[axis] == size[axis],
                "regions cover whole blocks",
            )?;
        }
        let subresource = Subresource {
            aspects,
            mip_level: self.level as u32,
            base_array_layer: self.slice as u32,
            layer_count: 1,
        };
        Ok((
            subresource,
            self.origin.map(|origin| origin as i32),
            self.size.map(|size| size as u32),
        ))
    }
}

impl Region {
    /// The region Metal copies of a format whose levels copy as runs of
    /// their blocks ([`Codec::whole_levels`](crate::metal::pixel_format::Codec::whole_levels)):
    /// as many blocks as `self` covers, from the level's first, whatever its
    /// origin. Packed, they are the level's first block rows or the start of
    /// its first; a run that ends within a later row is refused. Other
    /// formats' regions are themselves. The caller's bytes are packed too.
    pub fn block_run(&self, descriptor: &TextureDescriptor) -> Result<Self> {
        let description = descriptor.pixel_format.describe();
        if !matches!(description.encoding(), Encoding::Decoded(codec) if codec.whole_levels()) {
            return Ok(*self);
        }
        self.resolve(descriptor, ImageAspects::COLOR)?;
        let (width, height) = (
            u64::from(description.block.width),
            u64::from(description.block.height),
        );
        let [level_width, level_height, _] = descriptor.level_extent(self.level);
        let columns = level_width.div_ceil(width);
        let blocks = self.size[0].div_ceil(width) * self.size[1].div_ceil(height);
        let size = if blocks < columns {
            [blocks * width, height.min(level_height), self.size[2]]
        } else {
            require(
                blocks.is_multiple_of(columns),
                "PVRTC regions whose blocks end within a later row of their level",
            )?;
            let rows = blocks / columns;
            [level_width, (rows * height).min(level_height), self.size[2]]
        };
        Ok(Self {
            origin: [0; 3],
            size,
            ..*self
        })
    }
}

/// A buffer's side of a buffer-texture copy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BufferLayout {
    pub buffer: BufferRef,
    pub offset: u64,
    pub bytes_per_row: u64,
    /// Zero for single-image copies.
    pub bytes_per_image: u64,
}

impl BlitTexture<'_> {
    fn description(&self) -> &'static Description {
        self.descriptor.pixel_format.describe()
    }

    /// The aspect a copy reads or writes; combined depth-stencil formats
    /// need a blit option naming one.
    fn copy_aspect(&self, options: BlitOption) -> Result<ImageAspects> {
        let description = self.description();
        match description.kind {
            Kind::Color(_) => {
                ensure(
                    options.is_empty(),
                    "blit options apply to depth-stencil textures",
                )?;
                Ok(ImageAspects::COLOR)
            }
            Kind::Depth => Ok(ImageAspects::DEPTH),
            Kind::Stencil => Ok(ImageAspects::STENCIL),
            Kind::DepthStencil => {
                let depth = options.contains(BlitOption::DEPTH_FROM_DEPTH_STENCIL);
                let stencil = options.contains(BlitOption::STENCIL_FROM_DEPTH_STENCIL);
                match (depth, stencil) {
                    (true, false) => Ok(ImageAspects::DEPTH),
                    (false, true) => Ok(ImageAspects::STENCIL),
                    _ => Err(Error::Invalid("depth-stencil copies name one aspect")),
                }
            }
        }
    }

    /// Bytes of one block of `aspect` in a buffer.
    fn block_bytes(&self, aspect: ImageAspects) -> u64 {
        match (self.descriptor.pixel_format.describe().kind, aspect) {
            (Kind::DepthStencil | Kind::Stencil, ImageAspects::STENCIL) => 1,
            (Kind::DepthStencil, _) => 4,
            _ => u64::from(self.description().block.bytes),
        }
    }
}

impl CommandBuffer {
    pub(super) fn blit(&mut self) -> Result<()> {
        ensure(
            matches!(self.encoder, Encoder::Blit),
            "no blit command encoder is encoding",
        )
    }

    pub fn begin_blit(&mut self) -> Result<()> {
        self.idle()?;
        self.encoder = Encoder::Blit;
        Ok(())
    }

    /// `copyFromBuffer:sourceOffset:toBuffer:destinationOffset:size:`.
    pub fn copy_buffer(
        &mut self,
        src: BufferRef,
        src_offset: u64,
        dst: BufferRef,
        dst_offset: u64,
        size: u64,
    ) -> Result<()> {
        self.blit()?;
        let src_offset = src.at(src_offset, size)?;
        let dst_offset = dst.at(dst_offset, size)?;
        if size == 0 {
            return Ok(());
        }
        self.recorder.access(TRANSFER);
        self.recorder.push(Command::CopyBuffer {
            src: src.buffer,
            dst: dst.buffer,
            region: BufferCopy {
                src_offset,
                dst_offset,
                size,
            },
        });
        Ok(())
    }

    /// `copyFromBuffer:…toTexture:…options:`.
    pub fn copy_buffer_to_texture(
        &mut self,
        resolver: &mut dyn Resolver,
        src: BufferLayout,
        dst: BlitTexture,
        region: Region,
        options: BlitOption,
    ) -> Result<()> {
        self.blit()?;
        if let Some(blocks) = dst.blocks {
            if whole_levels(&dst) {
                self.reorder_blocks(resolver, &src, &dst, blocks, &region, false)?;
                let held = BufferLayout {
                    buffer: blocks,
                    offset: 0,
                    bytes_per_row: 0,
                    bytes_per_image: 0,
                };
                return self.decode_blocks(resolver, &dst, held, &region);
            }
            let (copies, held) = buffer_blocks(&src, &dst, blocks, &region)?;
            self.copy_regions(src.buffer.buffer, blocks.buffer, copies);
            return self.decode_blocks(resolver, &dst, held, &region);
        }
        if let Some(arguments) = texel_arguments(&src, &dst, &region, options)? {
            return self.convert_texels(resolver, arguments, true);
        }
        let copy = buffer_image_copy(&src, &dst, &region, options)?;
        self.recorder.access(TRANSFER);
        self.recorder.push(Command::CopyBufferToImage {
            src: src.buffer.buffer,
            dst: dst.image,
            region: copy,
        });
        Ok(())
    }

    /// `copyFromTexture:…toBuffer:…options:`.
    pub fn copy_texture_to_buffer(
        &mut self,
        resolver: &mut dyn Resolver,
        src: BlitTexture,
        region: Region,
        dst: BufferLayout,
        options: BlitOption,
    ) -> Result<()> {
        self.blit()?;
        if let Some(blocks) = src.blocks {
            if whole_levels(&src) {
                return self.reorder_blocks(resolver, &dst, &src, blocks, &region, true);
            }
            let (copies, _) = buffer_blocks(&dst, &src, blocks, &region)?;
            let copies = copies.into_iter().map(BufferCopy::reversed).collect();
            self.copy_regions(blocks.buffer, dst.buffer.buffer, copies);
            return Ok(());
        }
        if let Some(arguments) = texel_arguments(&dst, &src, &region, options)? {
            return self.convert_texels(resolver, arguments, false);
        }
        let copy = buffer_image_copy(&dst, &src, &region, options)?;
        self.recorder.access(TRANSFER);
        self.recorder.push(Command::CopyImageToBuffer {
            src: src.image,
            dst: dst.buffer.buffer,
            region: copy,
        });
        Ok(())
    }

    /// `copyFromTexture:sourceSlice:sourceLevel:sourceOrigin:sourceSize:toTexture:…`.
    pub fn copy_texture(
        &mut self,
        src: BlitTexture,
        src_region: Region,
        dst: BlitTexture,
        dst_slice: u64,
        dst_level: u64,
        dst_origin: [u64; 3],
    ) -> Result<()> {
        self.blit()?;
        check_copy_compatible(&src, &dst)?;
        let aspects = src.description().image_aspects();
        let (src_subresource, src_offset, extent) = src_region.resolve(src.descriptor, aspects)?;
        // Compressed-to-uncompressed copies scale the extent by the blocks.
        let (src_block, dst_block) = (src.description().block, dst.description().block);
        let size = [
            src_region.size[0] / u64::from(src_block.width) * u64::from(dst_block.width),
            src_region.size[1] / u64::from(src_block.height) * u64::from(dst_block.height),
            src_region.size[2],
        ];
        let dst_region = Region {
            slice: dst_slice,
            level: dst_level,
            origin: dst_origin,
            size,
        };
        let (dst_subresource, dst_offset, _) = dst_region.resolve(dst.descriptor, aspects)?;
        if extent.contains(&0) {
            return Ok(());
        }
        if whole_levels(&src) {
            let whole = |texture: &BlitTexture, region: &Region| {
                region.origin == [0; 3]
                    && region.size == texture.descriptor.level_extent(region.level)
            };
            require(
                whole(&src, &src_region) && whole(&dst, &dst_region),
                "copies between PVRTC textures of less than a whole level",
            )?;
        }
        self.copy_held_blocks(&src, &src_region, &dst, &dst_region)?;
        self.recorder.access(TRANSFER);
        self.recorder.push(Command::CopyImage {
            src: src.image,
            dst: dst.image,
            regions: alloc::vec![ImageCopy {
                src: src_subresource,
                src_offset,
                dst: dst_subresource,
                dst_offset,
                extent
            }],
        });
        Ok(())
    }

    /// `copyFromTexture:sourceSlice:sourceLevel:toTexture:destinationSlice:destinationLevel:sliceCount:levelCount:`.
    #[allow(clippy::too_many_arguments)]
    pub fn copy_texture_slices(
        &mut self,
        src: BlitTexture,
        src_slice: u64,
        src_level: u64,
        dst: BlitTexture,
        dst_slice: u64,
        dst_level: u64,
        slice_count: u64,
        level_count: u64,
    ) -> Result<()> {
        self.blit()?;
        check_copy_compatible(&src, &dst)?;
        let aspects = src.description().image_aspects();
        let mut regions = Vec::new();
        for level in 0..level_count {
            let size = src.descriptor.level_extent(src_level + level);
            ensure(
                dst.descriptor.level_extent(dst_level + level) == size,
                "copied levels have equal sizes",
            )?;
            for slice in 0..slice_count {
                let region = |slice, level| Region {
                    slice,
                    level,
                    origin: [0; 3],
                    size,
                };
                let (src_region, dst_region) = (
                    region(src_slice + slice, src_level + level),
                    region(dst_slice + slice, dst_level + level),
                );
                self.copy_held_blocks(&src, &src_region, &dst, &dst_region)?;
                let (src_subresource, src_offset, extent) =
                    src_region.resolve(src.descriptor, aspects)?;
                let (dst_subresource, dst_offset, _) =
                    dst_region.resolve(dst.descriptor, aspects)?;
                regions.push(ImageCopy {
                    src: src_subresource,
                    src_offset,
                    dst: dst_subresource,
                    dst_offset,
                    extent,
                });
            }
        }
        if regions.is_empty() {
            return Ok(());
        }
        self.recorder.access(TRANSFER);
        self.recorder.push(Command::CopyImage {
            src: src.image,
            dst: dst.image,
            regions,
        });
        Ok(())
    }

    /// `fillBuffer:range:value:`. Vulkan fills whole aligned words, so
    /// unaligned edges are copied from upload memory.
    pub fn fill_buffer(
        &mut self,
        resolver: &mut dyn Resolver,
        buffer: BufferRef,
        start: u64,
        length: u64,
        value: u8,
    ) -> Result<()> {
        self.blit()?;
        let begin = buffer.at(start, length)?;
        if length == 0 {
            return Ok(());
        }
        let end = begin + length;
        let (aligned_begin, aligned_end) = (begin.next_multiple_of(4).min(end), end / 4 * 4);
        self.recorder.access(TRANSFER);
        if aligned_end > aligned_begin {
            let data = u32::from_ne_bytes([value; 4]);
            let size = aligned_end - aligned_begin;
            self.recorder.push(Command::FillBuffer {
                buffer: buffer.buffer,
                offset: aligned_begin,
                size,
                data,
            });
        }
        let edges = if aligned_end > aligned_begin {
            [(begin, aligned_begin), (aligned_end, end)]
        } else {
            [(begin, end), (end, end)]
        };
        for (from, to) in edges.into_iter().filter(|(from, to)| to > from) {
            let upload =
                self.recorder
                    .upload(resolver, &alloc::vec![value; (to - from) as usize], 4)?;
            let region = BufferCopy {
                src_offset: upload.offset,
                dst_offset: from,
                size: to - from,
            };
            self.recorder.push(Command::CopyBuffer {
                src: upload.buffer,
                dst: buffer.buffer,
                region,
            });
        }
        Ok(())
    }

    /// `generateMipmapsForTexture:`: each level is a linear blit of the one
    /// above, ordered by a barrier.
    pub fn generate_mipmaps(&mut self, texture: BlitTexture, filterable: bool) -> Result<()> {
        self.blit()?;
        let description = texture.description();
        ensure(
            matches!(description.kind, Kind::Color(numeric) if !numeric.is_integer())
                && !description.is_compressed(),
            "mipmaps are generated for uncompressed float color textures",
        )?;
        ensure(
            filterable,
            "mipmap generation needs a filterable, renderable format",
        )?;
        let layers = texture.descriptor.slice_count() as u32;
        for level in 1..texture.descriptor.mipmap_level_count {
            let bounds = |size: [u64; 3]| [[0; 3], size.map(|size| size as i32)];
            let subresource = |level: u64| Subresource {
                aspects: ImageAspects::COLOR,
                mip_level: level as u32,
                base_array_layer: 0,
                layer_count: layers,
            };
            self.recorder.access(TRANSFER);
            self.recorder.push(Command::BlitImage {
                src: texture.image,
                dst: texture.image,
                regions: alloc::vec![ImageBlit {
                    src: subresource(level - 1),
                    src_bounds: bounds(texture.descriptor.level_extent(level - 1)),
                    dst: subresource(level),
                    dst_bounds: bounds(texture.descriptor.level_extent(level)),
                }],
                filter: Filter::Linear,
            });
        }
        Ok(())
    }

    /// `synchronizeResource:` for a managed buffer: GPU writes reach its
    /// mapping, which is invalidated once the command buffer completes.
    pub fn synchronize_buffer(&mut self, range: MappedRange) -> Result<()> {
        self.synchronize()?;
        self.recorder.invalidations.push(range);
        Ok(())
    }

    /// `synchronizeResource:` for a managed texture the CPU reads with host
    /// image copies: GPU writes become visible to the host once the command
    /// buffer completes.
    pub fn synchronize(&mut self) -> Result<()> {
        self.blit()?;
        if let Some(barrier) = self.recorder.hazards.host() {
            self.recorder.push(Command::Barrier(barrier));
        }
        Ok(())
    }

    /// Copies `region` of `image`, in the encoding the image stores, to
    /// `buffer` as `region` lays it out, and makes the copy visible to the
    /// host once the command buffer completes: a CPU read of a texture
    /// through host memory rather than through the image's own.
    pub(crate) fn read_back(
        &mut self,
        image: Image,
        buffer: Buffer,
        region: BufferImageCopy,
    ) -> Result<()> {
        self.blit()?;
        self.recorder.access(TRANSFER);
        self.recorder.push(Command::CopyImageToBuffer {
            src: image,
            dst: buffer,
            region,
        });
        self.synchronize()
    }

    /// `synchronizeTexture:slice:level:` for a managed texture whose CPU copy
    /// is a staging buffer: the region is copied there, then published.
    pub fn synchronize_texture(
        &mut self,
        resolver: &mut dyn Resolver,
        texture: BlitTexture,
        staging: BufferLayout,
        region: Region,
    ) -> Result<()> {
        self.copy_texture_to_buffer(resolver, texture, region, staging, BlitOption::EMPTY)?;
        self.synchronize()
    }

    /// Copies between two buffers' regions, after the pending work.
    fn copy_regions(&mut self, src: Buffer, dst: Buffer, regions: Vec<BufferCopy>) {
        if regions.is_empty() {
            return;
        }
        self.recorder.access(TRANSFER);
        self.recorder
            .push(Command::CopyBufferRegions { src, dst, regions });
    }

    /// For a copy between textures of a decoded format, the copy of the
    /// region's blocks, which their images hold decoded.
    fn copy_held_blocks(
        &mut self,
        src: &BlitTexture,
        src_region: &Region,
        dst: &BlitTexture,
        dst_region: &Region,
    ) -> Result<()> {
        let decoded = |texture: &BlitTexture| texture.description().decoder().is_some();
        let (Some(from), Some(to)) = (src.blocks, dst.blocks) else {
            return require(
                !decoded(src) && !decoded(dst),
                "copies through views of compressed textures the device samples decoded",
            );
        };
        let (from_start, from_pitch) = held_start(src, from, src_region);
        let (to_start, to_pitch) = held_start(dst, to, dst_region);
        let (rows, row_bytes) = block_rows(src, src_region);
        let copies = row_copies(from_start, from_pitch, to_start, to_pitch, rows, row_bytes);
        self.copy_regions(from.buffer, to.buffer, copies);
        Ok(())
    }

    /// Decodes a region of a decoded format's blocks, `held` from the
    /// region's first, into its image, after their copy.
    fn decode_blocks(
        &mut self,
        resolver: &mut dyn Resolver,
        texture: &BlitTexture,
        held: BufferLayout,
        region: &Region,
    ) -> Result<()> {
        let (kernel, encoding) = texture
            .description()
            .decoder()
            .ok_or(Error::Invalid("the texture holds no decoded blocks"))?;
        require(
            texture.storage != ResourceId::NULL,
            "buffer copies of 1D and 3D textures of a stored encoding",
        )?;
        let narrow = |value: u64| {
            u32::try_from(value).map_err(|_| Error::Unsupported("buffer copies of 4 GiB of texels"))
        };
        // A texel of a format that decodes whole levels depends on blocks
        // around its own.
        let (held, region) = if whole_levels(texture) {
            let level = Region {
                origin: [0; 3],
                size: texture.descriptor.level_extent(region.level),
                ..*region
            };
            let (start, pitch) = held_start(texture, held.buffer, &level);
            let held = BufferLayout {
                offset: start - held.buffer.offset,
                bytes_per_row: pitch,
                ..held
            };
            (held, level)
        } else {
            (held, *region)
        };
        let region = &region;
        let [width, height, _] = region.size;
        let arguments = TexelArguments {
            buffer: held.buffer.address + held.offset,
            count: narrow(width * height)?,
            encoding,
            image: texture.storage.0,
            width: narrow(width)?,
            height: narrow(height)?,
            bytes_per_row: narrow(held.bytes_per_row)?,
            bytes_per_image: narrow(held.bytes_per_image)?,
            // The view holds the copied slice alone.
            origin: [narrow(region.origin[0])?, narrow(region.origin[1])?, 0],
            ..TexelArguments::default()
        };
        if arguments.count == 0 {
            return Ok(());
        }
        let upload =
            self.recorder
                .upload(resolver, &arguments.encode(), TexelArguments::ALIGNMENT)?;
        self.recorder.access(Use::new(
            PipelineStages::COMPUTE_SHADER,
            Access::SHADER_READ.union(Access::SHADER_WRITE),
        ));
        self.recorder.push(Command::DecodeBlocks {
            kernel,
            arguments: upload.address,
            count: arguments.count,
        });
        Ok(())
    }

    /// Copies a buffer copy's region of blocks between the buffer's rows
    /// and a PVRTC level's Morton order in the texture's `blocks`, into the
    /// buffer when `to_rows`.
    fn reorder_blocks(
        &mut self,
        resolver: &mut dyn Resolver,
        layout: &BufferLayout,
        texture: &BlitTexture,
        blocks: BufferRef,
        region: &Region,
        to_rows: bool,
    ) -> Result<()> {
        // Validated as Metal validates a copy of the format's blocks.
        buffer_image_copy(layout, texture, region, BlitOption::EMPTY)?;
        ensure(region.size[2] <= 1, "copies of 2D textures hold one image")?;
        let block = texture.description().block;
        let (width, height) = (u64::from(block.width), u64::from(block.height));
        let [level_width, level_height, _] = texture.descriptor.level_extent(region.level);
        let level = texture
            .descriptor
            .subresource_layout(region.level, region.slice)
            .offset;
        let narrow = |value: u64| {
            u32::try_from(value).map_err(|_| Error::Unsupported("buffer copies of 4 GiB of texels"))
        };
        let (columns, rows) = (
            region.size[0].div_ceil(width),
            region.size[1].div_ceil(height),
        );
        let arguments = ReorderArguments {
            rows: layout.buffer.address + layout.offset,
            level: blocks.address + level,
            count: narrow(columns * rows)?,
            to_rows,
            columns: narrow(level_width.div_ceil(width))?,
            level_rows: narrow(level_height.div_ceil(height))?,
            origin: [
                narrow(region.origin[0] / width)?,
                narrow(region.origin[1] / height)?,
            ],
            width: narrow(columns)?,
            row_bytes: narrow(layout.bytes_per_row)?,
        };
        if arguments.count == 0 {
            return Ok(());
        }
        let upload =
            self.recorder
                .upload(resolver, &arguments.encode(), ReorderArguments::ALIGNMENT)?;
        self.recorder.access(Use::new(
            PipelineStages::COMPUTE_SHADER,
            Access::SHADER_READ.union(Access::SHADER_WRITE),
        ));
        self.recorder.push(Command::ReorderBlocks {
            arguments: upload.address,
            count: arguments.count,
        });
        Ok(())
    }

    /// Runs a texel kernel over a copy's texels: `store` from the buffer
    /// into the texture, else back.
    fn convert_texels(
        &mut self,
        resolver: &mut dyn Resolver,
        arguments: TexelArguments,
        store: bool,
    ) -> Result<()> {
        if arguments.count == 0 {
            return Ok(());
        }
        let upload =
            self.recorder
                .upload(resolver, &arguments.encode(), TexelArguments::ALIGNMENT)?;
        self.recorder.access(Use::new(
            PipelineStages::COMPUTE_SHADER,
            Access::SHADER_READ.union(Access::SHADER_WRITE),
        ));
        let (arguments, count) = (upload.address, arguments.count);
        self.recorder.push(if store {
            Command::StoreTexels { arguments, count }
        } else {
            Command::LoadTexels { arguments, count }
        });
        Ok(())
    }
}

/// Whether a decoded format's texture decodes whole levels and orders its
/// levels' blocks itself (`Codec::whole_levels`): PVRTC.
fn whole_levels(texture: &BlitTexture) -> bool {
    texture
        .description()
        .encoding()
        .decoded()
        .is_some_and(|codec| codec.whole_levels())
}

/// A decoded format's region's block rows and the bytes of each.
fn block_rows(texture: &BlitTexture, region: &Region) -> (u64, u64) {
    let block = texture.description().block;
    (
        region.size[1].div_ceil(u64::from(block.height)),
        region.size[0].div_ceil(u64::from(block.width)) * u64::from(block.bytes),
    )
}

/// Where a region's first block lies in a decoded format's `blocks`,
/// within its buffer, and the bytes per block row there.
fn held_start(texture: &BlitTexture, blocks: BufferRef, region: &Region) -> (u64, u64) {
    let block = texture.description().block;
    let layout = texture
        .descriptor
        .subresource_layout(region.level, region.slice);
    let start = blocks.offset
        + layout.offset
        + region.origin[1] / u64::from(block.height) * layout.bytes_per_row
        + region.origin[0] / u64::from(block.width) * u64::from(block.bytes);
    (start, layout.bytes_per_row)
}

/// Copies of `rows` rows of `row_bytes`, one copy when both sides pack them.
fn row_copies(
    src: u64,
    src_pitch: u64,
    dst: u64,
    dst_pitch: u64,
    rows: u64,
    row_bytes: u64,
) -> Vec<BufferCopy> {
    if rows == 0 || row_bytes == 0 {
        return Vec::new();
    }
    if src_pitch == row_bytes && dst_pitch == row_bytes {
        return alloc::vec![BufferCopy {
            src_offset: src,
            dst_offset: dst,
            size: rows * row_bytes,
        }];
    }
    (0..rows)
        .map(|row| BufferCopy {
            src_offset: src + row * src_pitch,
            dst_offset: dst + row * dst_pitch,
            size: row_bytes,
        })
        .collect()
}

/// A buffer copy of a decoded format's region as copies of its block rows
/// from the buffer into the texture's `blocks`, and where the region's
/// blocks lie there.
fn buffer_blocks(
    layout: &BufferLayout,
    texture: &BlitTexture,
    blocks: BufferRef,
    region: &Region,
) -> Result<(Vec<BufferCopy>, BufferLayout)> {
    // Validated as Metal validates a copy of the format's blocks.
    let copy = buffer_image_copy(layout, texture, region, BlitOption::EMPTY)?;
    ensure(region.size[2] <= 1, "copies of 2D textures hold one image")?;
    let (start, pitch) = held_start(texture, blocks, region);
    let (rows, row_bytes) = block_rows(texture, region);
    let copies = row_copies(
        copy.buffer_offset,
        layout.bytes_per_row,
        start,
        pitch,
        rows,
        row_bytes,
    );
    let held = BufferLayout {
        buffer: blocks,
        offset: start - blocks.offset,
        bytes_per_row: pitch,
        bytes_per_image: 0,
    };
    Ok((copies, held))
}

/// The texel kernels' arguments for a buffer copy of a texture whose image
/// stores another encoding than its format's bytes; `None` for one whose
/// image holds them, which a transfer copies.
fn texel_arguments(
    layout: &BufferLayout,
    texture: &BlitTexture,
    region: &Region,
    options: BlitOption,
) -> Result<Option<TexelArguments>> {
    let encoding = texture.description().encoding();
    if encoding.is_exact() {
        return Ok(None);
    }
    // A decoded format's view holds no blocks of its own.
    require(
        !matches!(encoding, Encoding::Decoded(_)),
        "copies through views of compressed textures the device samples decoded",
    )?;
    require(
        texture.storage != ResourceId::NULL,
        "buffer copies of 1D and 3D textures of a stored encoding",
    )?;
    let copy = buffer_image_copy(layout, texture, region, options)?;
    let narrow = |value: u64| {
        u32::try_from(value).map_err(|_| Error::Unsupported("buffer copies of 4 GiB of texels"))
    };
    let [width, height, depth] = copy.extent;
    let image_bytes = u64::from(copy.buffer_image_height) * layout.bytes_per_row;
    Ok(Some(TexelArguments {
        buffer: layout.buffer.address + layout.offset,
        count: narrow(u64::from(width) * u64::from(height) * u64::from(depth))?,
        encoding: encoding.texel_kernel(),
        image: texture.storage.0,
        width,
        height,
        bytes_per_row: narrow(layout.bytes_per_row)?,
        bytes_per_image: narrow(image_bytes)?,
        // The view holds the copied slice alone.
        origin: [copy.offset[0] as u32, copy.offset[1] as u32, 0],
        ..TexelArguments::default()
    }))
}

/// Metal copies between textures of one format, sRGB pairs or equal block
/// sizes and sample counts.
fn check_copy_compatible(src: &BlitTexture, dst: &BlitTexture) -> Result<()> {
    let (from, to) = (src.descriptor, dst.descriptor);
    ensure(
        from.sample_count == to.sample_count,
        "copied textures share a sample count",
    )?;
    let (a, b) = (src.description(), dst.description());
    // A stored encoding's texels mean its own format's values alone.
    if !a.encoding().is_exact() || !b.encoding().is_exact() {
        return ensure(
            from.pixel_format == to.pixel_format,
            "copied textures have compatible pixel formats",
        );
    }
    let same = from.pixel_format == to.pixel_format
        || from.pixel_format.srgb_pair() == Some(to.pixel_format);
    ensure(
        same || (a.is_color() && b.is_color() && a.block.bytes == b.block.bytes),
        "copied textures have compatible pixel formats",
    )
}

/// Converts Metal's byte strides to Vulkan's texel strides.
fn buffer_image_copy(
    layout: &BufferLayout,
    texture: &BlitTexture,
    region: &Region,
    options: BlitOption,
) -> Result<BufferImageCopy> {
    let aspect = texture.copy_aspect(options)?;
    let (subresource, offset, extent) = region.resolve(texture.descriptor, aspect)?;
    let block = texture.description().block;
    let block_bytes = texture.block_bytes(aspect);
    let rows = region.size[1].div_ceil(u64::from(block.height));
    let row_bytes = region.size[0].div_ceil(u64::from(block.width)) * block_bytes;
    ensure(
        layout.bytes_per_row >= row_bytes,
        "bytesPerRow is smaller than a row",
    )?;
    ensure(
        layout.bytes_per_row.is_multiple_of(block_bytes),
        "bytesPerRow is a multiple of the block size",
    )?;
    let image_bytes = if region.size[2] > 1 || layout.bytes_per_image != 0 {
        ensure(
            layout.bytes_per_image >= layout.bytes_per_row * rows,
            "bytesPerImage is smaller than an image",
        )?;
        ensure(
            layout.bytes_per_image.is_multiple_of(layout.bytes_per_row),
            "bytesPerImage is a multiple of bytesPerRow",
        )?;
        layout.bytes_per_image
    } else {
        layout.bytes_per_row * rows
    };
    let alignment = if aspect == ImageAspects::COLOR {
        block_bytes
    } else {
        4
    };
    ensure(
        layout.offset.is_multiple_of(block_bytes),
        "buffer offsets are multiples of the block size",
    )?;
    let last_image = (region.size[2].max(1) - 1) * image_bytes;
    let bytes = last_image + (rows.max(1) - 1) * layout.bytes_per_row + row_bytes;
    let buffer_offset = layout.buffer.at(layout.offset, bytes)?;
    if !buffer_offset.is_multiple_of(alignment) {
        return Err(Error::Unsupported(
            "depth and stencil copies at offsets that are not multiples of four",
        ));
    }
    Ok(BufferImageCopy {
        buffer_offset,
        buffer_row_length: (layout.bytes_per_row / block_bytes * u64::from(block.width)) as u32,
        buffer_image_height: (image_bytes / layout.bytes_per_row * u64::from(block.height)) as u32,
        subresource,
        offset,
        extent,
    })
}
