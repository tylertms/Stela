//! Buffers, textures, samplers and heaps.
use super::{
    Call, Object, Service, StartedReadBack, boolean, region,
    wire::{
        AUTOMATIC, BufferRangeRequest, BufferResponse, Empty, Fault, FormatAlignmentsResponse,
        Handle, HeapBufferSizeRequest, HeapDescriptorRecord, HeapUsageRequest, HeapUsageResponse,
        NewBufferRequest, NewBufferTextureRequest, NewHeapBufferRequest, NewHeapTextureRequest,
        NewImportedBufferRequest, NewTextureViewRequest, PixelFormatRequest,
        SamplerDescriptorRecord, SamplerResponse, SizeAndAlignRecord, Span, SparseTileRequest,
        SparseTileResponse, Status, TextureBytesRequest, TextureDescriptorRecord, TextureResponse,
    },
};
use crate::metal::{
    abi::{
        CompareFunction, HeapType, ResourceOptions, SamplerAddressMode, SamplerBorderColor,
        SamplerMinMagFilter, SamplerMipFilter, SamplerReductionMode, SparsePageSize, StorageMode,
        SwizzleChannels, TextureSwizzle, TextureType, TextureUsage,
    },
    command_buffer::blit::Region,
    device::{BufferId, Device, HeapId, MemoryId, TextureId, resources::Strides},
    error::Error,
    heap::{HeapDescriptor, SizeAndAlign},
    pixel_format::PixelFormat,
    resource::{PlaneLayout, TextureDescriptor, TextureViewDescriptor},
    sampler::SamplerDescriptor,
    shader::Compiler,
    timing::{Detail, Phase, Timed},
    vulkan::backend::Backend,
};
use alloc::vec::Vec;
use core::num::NonZeroU64;

/// How many heap nils the platform's log names; the rest only count.
const NOTED_HEAP_NILS: u64 = 32;

impl<B: Backend, C: Compiler> Service<B, C> {
    pub(super) fn new_buffer(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewBufferRequest = call.request()?;
        let options = options(request.options)?;
        // A guest task reaches only its own memory.
        if options.storage_mode != StorageMode::Private {
            return Err(Fault::new(
                Status::Invalid,
                "the CPU's buffers are imported",
            ));
        }
        let buffer = self.device.new_buffer(request.length, options)?;
        let response = self.adopt_buffer(call.connection, buffer)?;
        call.reply(&response)
    }

    pub(super) fn new_imported_buffer(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewImportedBufferRequest = call.request()?;
        let options = options(request.options)?;
        let memory = self.object::<MemoryId>(call.connection, request.memory)?;
        let buffer = self
            .device
            .new_imported_buffer(memory, request.length, options)?;
        self.take_memory(call.connection, request.memory)?;
        let response = self.adopt_buffer(call.connection, buffer)?;
        call.reply(&response)
    }

    pub(super) fn format_alignments(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: PixelFormatRequest = call.request()?;
        let format = pixel_format(request.pixel_format)?;
        let capabilities = self.device.capabilities();
        let block = format.describe().block;
        call.reply(&FormatAlignmentsResponse {
            linear_texture: capabilities.minimum_linear_texture_alignment(format),
            texture_buffer: capabilities.minimum_texture_buffer_alignment(format),
            block_bytes: block.bytes.into(),
            block_width: block.width.into(),
            block_height: block.height.into(),
        })
    }

    pub(super) fn sparse_tile_size(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: SparseTileRequest = call.request()?;
        let size = self.device.capabilities().sparse_tile_size(
            texture_type(request.texture_type)?,
            pixel_format(request.pixel_format)?,
            request.sample_count,
        );
        call.reply(&SparseTileResponse {
            size: size.map_or([0; 3], |size| [size.width, size.height, size.depth]),
        })
    }

    pub(super) fn buffer_did_modify_range(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: BufferRangeRequest = call.request()?;
        let buffer = self.object::<BufferId>(call.connection, request.buffer)?;
        self.device
            .did_modify_range(buffer, request.offset, request.length)?;
        call.reply(&Empty {})
    }

    pub(super) fn new_texture(&mut self, call: &mut Call) -> Result<(), Fault> {
        self.device.timing().begin(Timed::NewTexture);
        let descriptor = texture_descriptor(&call.request()?)?;
        let texture = self.device.new_texture(&descriptor)?;
        let response = self.adopt_texture(call.connection, texture)?;
        call.reply(&response)
    }

    pub(super) fn new_texture_view(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewTextureViewRequest = call.request()?;
        let parent = self.object::<TextureId>(call.connection, request.texture)?;
        let view = TextureViewDescriptor {
            pixel_format: pixel_format(request.pixel_format)?,
            texture_type: texture_type(request.texture_type)?,
            levels: range(request.levels)?,
            slices: range(request.slices)?,
            swizzle: swizzle(request.swizzle)?,
            min_lod: request.min_lod,
        };
        let texture = self.device.new_texture_view(parent, &view)?;
        let response = self.adopt_texture(call.connection, texture)?;
        call.reply(&response)
    }

    pub(super) fn new_buffer_texture(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewBufferTextureRequest = call.request()?;
        let buffer = self.object::<BufferId>(call.connection, request.buffer)?;
        let descriptor = texture_descriptor(&request.descriptor)?;
        let plane = |offset, bytes_per_row| PlaneLayout {
            offset,
            bytes_per_row,
        };
        // An IOSurface's extended-range texture keeps its texels in the
        // surface's two planes ('&b38'), which the display reads.
        let texture = if descriptor.pixel_format.extended_planes().is_some() {
            if request.second_bytes_per_row == 0 {
                return Err(crate::metal::error::Error::Unsupported(
                    "buffer textures of two-plane extended-range formats",
                )
                .into());
            }
            self.device.new_extended_plane_texture(
                buffer,
                &descriptor,
                [
                    plane(request.offset, request.bytes_per_row),
                    plane(request.second_offset, request.second_bytes_per_row),
                ],
            )?
        } else if descriptor.pixel_format.planes().is_some() {
            if request.second_bytes_per_row == 0 {
                return Err(Fault::malformed(
                    "a two-plane texture without its second plane",
                ));
            }
            self.device.new_two_plane_texture(
                buffer,
                &descriptor,
                [
                    plane(request.offset, request.bytes_per_row),
                    plane(request.second_offset, request.second_bytes_per_row),
                ],
                request.surface_format as u32,
            )?
        } else {
            self.device.new_texture_from_buffer(
                buffer,
                &descriptor,
                request.offset,
                request.bytes_per_row,
            )?
        };
        let response = self.adopt_texture(call.connection, texture)?;
        call.reply(&response)
    }

    /// A band at a time ([`Device::cpu_bands`]), so a texture of any size
    /// moves through a bounded amount of memory.
    pub(super) fn replace_region(&mut self, call: &mut Call) -> Result<(), Fault> {
        let transfer = self.transfer(call)?;
        for (region, span) in transfer.bands {
            let bytes = call.memory.read(span)?;
            self.device
                .replace_region(transfer.texture, &region, &bytes, transfer.strides)?;
        }
        call.reply(&Empty {})
    }

    /// A band at a time, as [`Self::replace_region`]. The bytes between rows
    /// keep what the caller had there.
    pub(super) fn get_bytes(&mut self, call: &mut Call) -> Result<(), Fault> {
        self.device.timing().begin(Timed::GetBytes);
        let transfer = self.transfer(call)?;
        let format = self
            .device
            .texture_descriptor(transfer.texture)?
            .pixel_format;
        self.device.timing().detail(
            Timed::GetBytes,
            Detail::Bytes {
                format,
                width: transfer
                    .bands
                    .first()
                    .map_or(0, |(region, _)| region.size[0]),
                rows: transfer
                    .bands
                    .iter()
                    .map(|(region, _)| region.size[1])
                    .sum(),
                bands: transfer.bands.len() as u64,
                bytes: transfer.bands.iter().map(|(_, span)| span.length).sum(),
            },
        );
        // The bands before it were copied to the caller already.
        let (from, count) = (
            self.read_back_from.take().unwrap_or(0),
            transfer.bands.len(),
        );
        for (band, (region, span)) in transfer.bands.into_iter().enumerate().skip(from) {
            if self.reads_back {
                let length = usize::try_from(span.length)
                    .map_err(|_| Fault::malformed("span length").value(span.length))?;
                let started = self.device.begin_read_back(
                    call.connection.get(),
                    transfer.texture,
                    &region,
                    transfer.strides,
                    length,
                )?;
                if let Some(read_back) = started {
                    self.read_back_started = Some(StartedReadBack {
                        read_back,
                        band,
                        span,
                        last: band + 1 == count,
                    });
                    return Err(Fault::new(Status::Pending, ""));
                }
            }
            let start = self.device.timing().now();
            let mut bytes = call.memory.read(span)?;
            self.device.timing().since(Phase::BytesRead, start);
            self.device
                .get_bytes(transfer.texture, &region, &mut bytes, transfer.strides)?;
            let start = self.device.timing().now();
            call.memory.write(span, &bytes)?;
            self.device.timing().since(Phase::BytesWrite, start);
        }
        call.reply(&Empty {})
    }

    /// A CPU texture transfer, refused whole before any band moves.
    fn transfer(&mut self, call: &mut Call) -> Result<Transfer, Fault> {
        let request: TextureBytesRequest = call.request()?;
        let texture = self.object::<TextureId>(call.connection, request.texture)?;
        let strides = strides(&request);
        let bands = self
            .device
            .cpu_bands(texture, &region(&request.region), strides)?
            .into_iter()
            .map(|band| {
                if band.bytes.end > request.bytes.length {
                    return Err(Fault::new(
                        Status::Invalid,
                        "the bytes end before the region",
                    ));
                }
                let address = request
                    .bytes
                    .address
                    .checked_add(band.bytes.start)
                    .ok_or(Fault::malformed("span address").bits(request.bytes.address))?;
                let span = Span {
                    address,
                    length: band.bytes.end - band.bytes.start,
                };
                Ok((band.region, span))
            })
            .collect::<Result<_, Fault>>()?;
        Ok(Transfer {
            texture,
            strides,
            bands,
        })
    }

    pub(super) fn new_sampler(&mut self, call: &mut Call) -> Result<(), Fault> {
        let descriptor = sampler_descriptor(&call.request()?)?;
        // Shaders apply both, reading the sampler's flags.
        if descriptor.bicubic() || !descriptor.normalized_coordinates {
            self.admit_bicubic(call.connection)?;
        }
        let sampler = self.device.new_sampler(&descriptor)?;
        let (handle, resource_id) =
            self.adopt(call.connection, Object::Sampler(sampler), |device| {
                device.sampler_resource_id(sampler)
            })?;
        call.reply(&SamplerResponse {
            handle: handle.handle,
            resource_id: resource_id.0,
        })
    }

    pub(super) fn new_heap(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: HeapDescriptorRecord = call.request()?;
        let descriptor = HeapDescriptor {
            size: request.size,
            options: options(request.options)?,
            heap_type: super::decode(HeapType::from_raw, request.heap_type, "heap type")?,
            sparse_page_size: super::decode(
                SparsePageSize::from_raw,
                request.sparse_page_size,
                "sparse page size",
            )?,
        };
        let heap = if request.memory == 0 {
            // A guest task reaches only its own memory.
            if descriptor.options.storage_mode != StorageMode::Private {
                return Err(Fault::new(Status::Invalid, "the CPU's heaps are imported"));
            }
            self.device.new_heap(&descriptor)?
        } else {
            let memory = self.object::<MemoryId>(call.connection, request.memory)?;
            let heap = self.device.new_imported_heap(memory, &descriptor)?;
            self.take_memory(call.connection, request.memory)?;
            heap
        };
        let handle = self.hold(call.connection, Object::Heap(heap))?;
        call.reply(&handle)
    }

    pub(super) fn heap_buffer_size_and_align(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: HeapBufferSizeRequest = call.request()?;
        let options = options(request.options)?;
        let size = self
            .device
            .heap_buffer_size_and_align(request.length, options)?;
        call.reply(&size_and_align(size))
    }

    pub(super) fn heap_texture_size_and_align(&mut self, call: &mut Call) -> Result<(), Fault> {
        let descriptor = texture_descriptor(&call.request()?)?;
        let size = self.device.heap_texture_size_and_align(&descriptor)?;
        call.reply(&size_and_align(size))
    }

    pub(super) fn new_heap_buffer(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewHeapBufferRequest = call.request()?;
        let heap = self.object::<HeapId>(call.connection, request.heap)?;
        let options = options(request.options)?;
        let buffer =
            match self
                .device
                .new_heap_buffer(heap, request.length, options, offset(request.offset))
            {
                Err(Error::HeapFull) => {
                    let asked = self
                        .device
                        .heap_buffer_size_and_align(request.length, options);
                    let what = alloc::format!("buffer length={}", request.length);
                    self.note_heap_full(call.connection, heap, asked, what);
                    return Err(Error::HeapFull.into());
                }
                buffer => buffer?,
            };
        let response = self.adopt_buffer(call.connection, buffer)?;
        call.reply(&response)
    }

    pub(super) fn new_heap_texture(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewHeapTextureRequest = call.request()?;
        let heap = self.object::<HeapId>(call.connection, request.heap)?;
        let descriptor = texture_descriptor(&request.descriptor)?;
        // A heap without room answers nil, as Metal does; the device counts it.
        let texture = match self
            .device
            .new_heap_texture(heap, &descriptor, offset(request.offset))
        {
            Err(Error::HeapFull) => {
                let asked = self.device.heap_texture_size_and_align(&descriptor);
                let what = alloc::format!(
                    "texture {:?} {}x{}x{} levels={} samples={} layers={} usage={:#x} options={:?}",
                    descriptor.pixel_format,
                    descriptor.width,
                    descriptor.height,
                    descriptor.depth,
                    descriptor.mipmap_level_count,
                    descriptor.sample_count,
                    descriptor.array_length,
                    descriptor.usage.bits(),
                    descriptor.options,
                );
                self.note_heap_full(call.connection, heap, asked, what);
                return Err(Error::HeapFull.into());
            }
            texture => texture?,
        };
        let response = self.adopt_texture(call.connection, texture)?;
        call.reply(&response)
    }

    /// Names the first [`NOTED_HEAP_NILS`] nils of full heaps in the
    /// platform's log: the connection, the heap's size, use and largest free
    /// piece at the request's alignment, and the request.
    fn note_heap_full(
        &mut self,
        connection: NonZeroU64,
        heap: HeapId,
        asked: Result<SizeAndAlign, Error>,
        what: alloc::string::String,
    ) {
        let refusals = self.device.heap_refusals();
        let count = refusals.full + refusals.fragmented;
        let (Ok(asked), Ok((size, resources))) = (asked, self.device.heap_occupancy(heap)) else {
            return;
        };
        let Ok((used, largest)) = self.device.heap_usage(heap, asked.align) else {
            return;
        };
        if count > NOTED_HEAP_NILS {
            return;
        }
        self.noted = Some(alloc::format!(
            "HEAP NIL {count} connection={connection} heap={size} resources={resources} \
             used={used} largest={largest} asked={}@{:#x} {what}",
            asked.size,
            asked.align,
        ));
    }

    pub(super) fn heap_usage(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: HeapUsageRequest = call.request()?;
        let heap = self.object::<HeapId>(call.connection, request.heap)?;
        let (used_size, max_available_size) = self.device.heap_usage(heap, request.alignment)?;
        let (size, _) = self.device.heap_occupancy(heap)?;
        call.reply(&HeapUsageResponse {
            used_size,
            max_available_size,
            size,
        })
    }

    pub(super) fn make_aliasable(&mut self, call: &mut Call) -> Result<(), Fault> {
        let Handle { handle } = call.request()?;
        match self.entry(call.connection, handle)? {
            Object::Buffer(buffer) => self.device.make_buffer_aliasable(buffer)?,
            Object::Texture(texture) => self.device.make_texture_aliasable(texture)?,
            _ => return Err(Fault::STALE),
        }
        call.reply(&Empty {})
    }

    pub(super) fn adopt_buffer(
        &mut self,
        connection: NonZeroU64,
        buffer: BufferId,
    ) -> Result<BufferResponse, Fault> {
        let (handle, response) = self.adopt(connection, Object::Buffer(buffer), |device| {
            let placement = device.buffer_placement(buffer)?;
            Ok(BufferResponse {
                handle: 0,
                gpu_address: device.buffer(buffer)?.address,
                allocated_size: placement.allocated_size,
                heap_offset: placement.heap_offset,
            })
        })?;
        Ok(BufferResponse {
            handle: handle.handle,
            ..response
        })
    }

    pub(super) fn adopt_texture(
        &mut self,
        connection: NonZeroU64,
        texture: TextureId,
    ) -> Result<TextureResponse, Fault> {
        let (handle, response) = self.adopt(connection, Object::Texture(texture), |device| {
            texture_response(device, texture)
        })?;
        Ok(TextureResponse {
            handle: handle.handle,
            ..response
        })
    }
}

fn texture_response<B: Backend, C: Compiler>(
    device: &Device<B, C>,
    texture: TextureId,
) -> Result<TextureResponse, Error> {
    let placement = device.texture_placement(texture)?;
    let tail = device.sparse_tail(texture)?;
    let (first_mipmap_in_tail, tail_size) = tail.unwrap_or_default();
    Ok(TextureResponse {
        handle: 0,
        resource_id: device.texture_resource_id(texture)?.0,
        allocated_size: placement.allocated_size,
        heap_offset: placement.heap_offset,
        first_mipmap_in_tail,
        tail_size,
        sparse: tail.is_some().into(),
        reserved: 0,
    })
}

fn options(raw: u64) -> Result<ResourceOptions, Fault> {
    super::decode_bits(ResourceOptions::from_raw, raw, "resource options")
}

pub(super) fn pixel_format(raw: u64) -> Result<PixelFormat, Fault> {
    super::decode(PixelFormat::from_raw, raw, "pixel format")
}

fn texture_type(raw: u64) -> Result<TextureType, Fault> {
    super::decode(TextureType::from_raw, raw, "texture type")
}

fn swizzle(raw: [u8; 4]) -> Result<SwizzleChannels, Fault> {
    let mut channels = SwizzleChannels::IDENTITY;
    for (channel, raw) in channels.0.iter_mut().zip(raw) {
        *channel = super::decode(TextureSwizzle::from_raw, raw, "texture swizzle")?;
    }
    Ok(channels)
}

/// An `NSRange` of a location and a length.
fn range([location, length]: [u64; 2]) -> Result<core::ops::Range<u64>, Fault> {
    let end = location
        .checked_add(length)
        .ok_or(Fault::malformed("range"))?;
    Ok(location..end)
}

fn offset(raw: u64) -> Option<u64> {
    (raw != AUTOMATIC).then_some(raw)
}

fn texture_descriptor(record: &TextureDescriptorRecord) -> Result<TextureDescriptor, Fault> {
    Ok(TextureDescriptor {
        texture_type: texture_type(record.texture_type)?,
        pixel_format: pixel_format(record.pixel_format)?,
        width: record.width,
        height: record.height,
        depth: record.depth,
        mipmap_level_count: record.mipmap_level_count,
        sample_count: record.sample_count,
        array_length: record.array_length,
        options: options(record.options)?,
        usage: super::decode_bits(TextureUsage::from_bits, record.usage, "texture usage")?,
        swizzle: swizzle(record.swizzle)?,
    })
}

fn sampler_descriptor(record: &SamplerDescriptorRecord) -> Result<SamplerDescriptor, Fault> {
    let filter = |raw| super::decode(SamplerMinMagFilter::from_raw, raw, "sampler filter");
    let mut address_modes = [SamplerAddressMode::ClampToEdge; 3];
    for (mode, raw) in address_modes.iter_mut().zip(record.address_modes) {
        *mode = super::decode(SamplerAddressMode::from_raw, raw, "sampler address mode")?;
    }
    Ok(SamplerDescriptor {
        min_filter: filter(record.min_filter)?,
        mag_filter: filter(record.mag_filter)?,
        mip_filter: super::decode(
            SamplerMipFilter::from_raw,
            record.mip_filter,
            "sampler mip filter",
        )?,
        max_anisotropy: record.max_anisotropy,
        address_modes,
        border_color: super::decode(
            SamplerBorderColor::from_raw,
            record.border_color,
            "sampler border color",
        )?,
        reduction_mode: super::decode(
            SamplerReductionMode::from_raw,
            record.reduction_mode,
            "sampler reduction mode",
        )?,
        normalized_coordinates: boolean(record.normalized_coordinates)?,
        lod_min_clamp: record.lod_min_clamp,
        lod_max_clamp: record.lod_max_clamp,
        compare_function: super::decode(
            CompareFunction::from_raw,
            record.compare_function,
            "compare function",
        )?,
    })
}

/// A CPU texture transfer: its texture and strides, and each band's region
/// with the caller's bytes of it.
struct Transfer {
    texture: TextureId,
    strides: Strides,
    bands: Vec<(Region, Span)>,
}

fn strides(request: &TextureBytesRequest) -> Strides {
    Strides {
        bytes_per_row: request.bytes_per_row,
        bytes_per_image: request.bytes_per_image,
    }
}

fn size_and_align(size: SizeAndAlign) -> SizeAndAlignRecord {
    SizeAndAlignRecord {
        size: size.size,
        align: size.align,
    }
}
