use super::super::Strides;
use crate::metal::{
    abi::{
        CpuCacheMode, HazardTrackingMode, ResourceOptions, StorageMode, SwizzleChannels,
        TextureType, TextureUsage,
    },
    command_buffer::blit::Region,
    device::{
        TextureId,
        fixture::{MockBackend, TestDevice, device_on},
    },
    pixel_format::PixelFormat,
    resource::TextureDescriptor,
    vulkan::{backend::Object, command::Command},
};
use alloc::vec::Vec;

fn texture(
    device: &mut TestDevice,
    storage_mode: StorageMode,
    pixel_format: PixelFormat,
    width: u64,
) -> TextureId {
    let descriptor = TextureDescriptor {
        texture_type: TextureType::D2,
        pixel_format,
        width,
        height: 2,
        depth: 1,
        mipmap_level_count: 1,
        sample_count: 1,
        array_length: 1,
        options: ResourceOptions {
            cpu_cache_mode: CpuCacheMode::DefaultCache,
            storage_mode,
            hazard_tracking_mode: HazardTrackingMode::Default,
        },
        usage: TextureUsage::SHADER_READ,
        swizzle: SwizzleChannels::IDENTITY,
    };
    device.new_texture(&descriptor).unwrap()
}

fn whole(width: u64) -> Region {
    Region {
        slice: 0,
        level: 0,
        origin: [0; 3],
        size: [width, 2, 1],
    }
}

#[test]
fn a_shared_texture_is_read_through_host_memory_a_gpu_copy_filled() {
    let mut device = device_on(MockBackend::new());
    let texture = texture(&mut device, StorageMode::Shared, PixelFormat::Rgba8Unorm, 2);
    let tight = Strides {
        bytes_per_row: 8,
        bytes_per_image: 0,
    };
    let written: Vec<u8> = (1..=16).collect();
    device
        .replace_region(texture, &whole(2), &written, tight)
        .unwrap();
    let allocated = device.memory().allocated;
    // Rows 12 bytes apart: the 4 between them stay the caller's.
    let strided = Strides {
        bytes_per_row: 12,
        ..tight
    };
    let read_back = device
        .begin_read_back(5, texture, &whole(2), strided, 20)
        .unwrap()
        .expect("only host image copies reach the image");
    let image = device.texture_image(texture).unwrap();
    let copied = device
        .backend
        .submissions
        .iter()
        .flatten()
        .flat_map(|segment| &segment.commands)
        .any(|command| matches!(command, Command::CopyImageToBuffer { src, .. } if *src == image));
    assert!(copied, "the GPU copies the image");
    assert!(
        !device.read_back_complete(&read_back).unwrap(),
        "readable only once the copy completed"
    );
    device.backend.finish();
    assert!(device.read_back_complete(&read_back).unwrap());
    let mut read = [0xee; 20];
    read_back
        .write_out::<()>(|offset, run| {
            read[offset as usize..][..run.len()].copy_from_slice(run);
            Ok(())
        })
        .unwrap();
    assert_eq!(read[..8], written[..8]);
    assert_eq!(read[8..12], [0xee; 4]);
    assert_eq!(read[12..], written[8..]);
    device.end_read_back(read_back);
    assert_eq!(device.memory().allocated, allocated, "its memory is freed");
    assert!(
        device
            .backend
            .destroyed
            .iter()
            .any(|object| matches!(object, Object::Buffer(_)))
    );
}

#[test]
fn a_read_back_decodes_what_the_image_stores() {
    let mut device = device_on(MockBackend::new());
    let texture = texture(&mut device, StorageMode::Shared, PixelFormat::Bgr10Xr, 1);
    let tight = Strides {
        bytes_per_row: 4,
        bytes_per_image: 0,
    };
    // Red 1023, green 894, blue 384; then every channel at 0.
    let pack = |red: u32, green: u32, blue: u32| blue | green << 10 | red << 20 | 3 << 30;
    let written: Vec<u8> = [pack(1023, 894, 384), pack(0, 0, 0)]
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect();
    device
        .replace_region(texture, &whole(1), &written, tight)
        .unwrap();
    let read_back = device
        .begin_read_back(5, texture, &whole(1), tight, 8)
        .unwrap()
        .unwrap();
    device.backend.finish();
    assert!(device.read_back_complete(&read_back).unwrap());
    let mut runs = Vec::new();
    read_back
        .write_out::<()>(|offset, run| {
            runs.push((offset, run.to_vec()));
            Ok(())
        })
        .unwrap();
    assert_eq!(runs, [(0, written)], "packed rows are one run");
    device.end_read_back(read_back);
}

#[test]
fn a_texture_with_a_cpu_copy_needs_no_read_back() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.host_image_copy = false;
    let mut device = device_on(backend);
    let texture = texture(
        &mut device,
        StorageMode::Managed,
        PixelFormat::Rgba8Unorm,
        2,
    );
    let tight = Strides {
        bytes_per_row: 8,
        bytes_per_image: 0,
    };
    let submitted = device.backend.submissions.len();
    assert!(
        device
            .begin_read_back(5, texture, &whole(2), tight, 16)
            .unwrap()
            .is_none()
    );
    assert_eq!(device.backend.submissions.len(), submitted);
}
