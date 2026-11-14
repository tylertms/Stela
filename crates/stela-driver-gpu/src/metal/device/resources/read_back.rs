//! CPU reads of textures whose image only host image copies reach.
//!
//! Such an image lies in device-local memory, which the host maps
//! uncached or write-combined: a host image copy out of it reads VRAM
//! through the PCIe BAR at about a megabyte a second, so a `getBytes` of a
//! 420x420 texture held the device for 600 ms. Instead the GPU copies the
//! region into host-cached memory, and the CPU reads it from there once the
//! copy completes, which the device need not be held for.
use super::{CpuRows, Strides, stored_len};
use crate::metal::{
    command_buffer::blit::Region,
    device::{Device, TextureId, pins::Scope},
    error::{Error, Result},
    pixel_format::{Encoding, PixelFormat},
    resource::CpuAccess,
    shader::Compiler,
    vulkan::{
        BufferCreate, BufferUsage, MemoryProperties,
        backend::{Backend, BufferAllocation, Object},
        object::{BufferInfo, MemoryRequest},
    },
};
use core::ptr::NonNull;

/// The Metal queue of a connection its read-backs commit on, which no
/// guest queue's held work delays.
const READ_BACK_QUEUE: u64 = u64::MAX;

/// A texture region the GPU copies into host-cached memory, for the CPU
/// to read once command buffer [`ReadBack::serial`] completes.
pub struct ReadBack {
    staging: BufferAllocation,
    /// The host's view of the staging memory.
    host: NonNull<u8>,
    /// The bytes the GPU writes: the region tightly packed, in the encoding
    /// the image stores.
    stored: usize,
    format: PixelFormat,
    /// Where the region's rows go in the caller's bytes.
    rows: CpuRows,
    serial: u64,
}

// SAFETY: `host` points into memory only [`Device::end_read_back`] frees,
// which takes the read-back; what a processor reads through it the GPU
// wrote before the completion the reader saw.
unsafe impl Send for ReadBack {}

impl ReadBack {
    /// The command buffer whose completion makes the bytes readable.
    pub fn serial(&self) -> u64 {
        self.serial
    }

    /// Gives `write` the region's bytes as the caller lays them out, each
    /// run with its offset in the caller's bytes: one run when the rows
    /// are packed, else one for each row, leaving the bytes between rows
    /// as the caller had them. Only once command buffer [`Self::serial`]
    /// completed, and while the device that made the read-back holds it.
    pub fn write_out<E>(
        &self,
        mut write: impl FnMut(u64, &[u8]) -> core::result::Result<(), E>,
    ) -> core::result::Result<(), E> {
        // SAFETY: the staging memory is mapped, host-coherent and
        // `stored` bytes long, and the GPU finished writing it.
        let stored = unsafe { core::slice::from_raw_parts(self.host.as_ptr(), self.stored) };
        let decoded;
        let packed = match self.format.describe().encoding() {
            Encoding::Exact => &stored[..self.rows.packed_bytes],
            encoding => {
                let mut bytes = alloc::vec![0; self.rows.packed_bytes];
                encoding.load(stored, &mut bytes);
                decoded = bytes;
                &decoded[..]
            }
        };
        if self.rows.tight() {
            return write(0, packed);
        }
        let mut rest = packed;
        for row in self.rows.ranges() {
            let (head, tail) = rest.split_at(row.len());
            write(row.start as u64, head)?;
            rest = tail;
        }
        Ok(())
    }
}

impl<B: Backend, C: Compiler> Device<B, C> {
    /// Starts the read of `region` of texture `id` into `length` bytes laid
    /// out by `strides`, as [`Self::get_bytes`] reads it, by a GPU copy that
    /// connection `key` commits: `None` for a texture the CPU reaches
    /// through host memory already, which `get_bytes` reads at once.
    pub fn begin_read_back(
        &mut self,
        key: u64,
        id: TextureId,
        region: &Region,
        strides: Strides,
        length: usize,
    ) -> Result<Option<ReadBack>> {
        let (region, strides) = self.block_run(id, region, strides)?;
        let (copy, rows) = self.cpu_region(id, &region, strides, length)?;
        let texture = self.resources.textures.get(id.0)?;
        self.cpu_access(texture)?;
        let image = texture.image.ok_or(Error::Invalid(
            "texture buffers are read through their buffer",
        ))?;
        if texture.cpu_access != CpuAccess::HostImageCopy
            || texture.cpu_copy.is_some()
            || rows.packed_bytes == 0
        {
            return Ok(None);
        }
        let format = texture.descriptor.pixel_format;
        let description = format.describe();
        let stored = match description.encoding() {
            Encoding::Exact => rows.packed_bytes,
            _ => stored_len(description, rows.packed_bytes),
        };
        let info = BufferInfo {
            size: stored as u64,
            usage: BufferUsage::TRANSFER_DST,
            flags: BufferCreate::EMPTY,
        };
        let request = MemoryRequest {
            required: MemoryProperties::HOST_VISIBLE | MemoryProperties::HOST_COHERENT,
            preferred: MemoryProperties::HOST_CACHED,
            mapped: true,
        };
        let staging = self.backend.create_buffer(&info, request)?;
        self.allocated += staging.allocated;
        let Some(host) = self.backend.host_bytes(staging.memory, 0, stored as u64) else {
            // No coherent view of it: the host image copy reads at once.
            self.free_staging(staging);
            return Ok(None);
        };
        let mut commands = self.new_command_buffer();
        let recorded = commands
            .begin_blit()
            .and_then(|()| commands.read_back(image, staging.buffer, copy))
            .and_then(|()| commands.end_encoding(&mut self.resolver()));
        let committed = match recorded {
            Ok(()) => self.commit_on(key, READ_BACK_QUEUE, commands),
            Err(error) => {
                self.discard(commands);
                Err(error)
            }
        };
        let serial = match committed {
            Ok(serial) => serial,
            Err(error) => {
                self.free_staging(staging);
                return Err(error);
            }
        };
        Ok(Some(ReadBack {
            staging,
            host,
            stored,
            format,
            rows,
            serial,
        }))
    }

    /// Whether `read_back`'s copy completed, after a [`Self::poll`].
    pub fn read_back_complete(&mut self, read_back: &ReadBack) -> Result<bool> {
        if self.is_lost() {
            return Err(Error::DeviceLost);
        }
        self.poll()?;
        Ok(self.serial_complete(read_back.serial))
    }

    /// Frees `read_back`'s memory once its copy no longer runs.
    pub fn end_read_back(&mut self, read_back: ReadBack) {
        self.free_staging(read_back.staging);
    }

    fn free_staging(&mut self, staging: BufferAllocation) {
        // The copy that writes it is the device's own, whatever the scope
        // of the call that frees it.
        self.release_for(Scope::DEVICE, |device| {
            device.defer(Object::Buffer(staging.buffer));
            device.release_memory(staging.memory, staging.allocated);
        });
    }
}

#[cfg(test)]
mod tests;
