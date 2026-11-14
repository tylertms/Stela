use super::{
    GuestMemory, Outcome, Service,
    identity::DeviceEpoch,
    wire::{
        self, BufferResponse, ColorAttachmentRecord, ConstantInfo, ConstantRecord, Empty, Fault,
        FunctionReflectionRequest, FunctionReflectionResponse, Handle, Length, LibraryDataRequest,
        LibraryFunctionNamesRequest, LibraryIdentityRequest, LibraryIdentityResponse,
        LinkedFunctionsRecord, NegotiateRequest, NegotiateResponse, NewBufferRequest,
        NewFunctionRequest, NewImportedBufferRequest, NewLibraryRequest, NewRenderPipelineRequest,
        NewSourceLibraryRequest, Operation, PipelineFlags, Record, RenderTargetsRecord,
        ReportRequest, Span, Status, TessellationRecord, TextureDescriptorRecord, Value,
        commands::{CommandBufferCode, CommandHeader, CommitRequest},
    },
};
use crate::metal::{
    abi::{
        BlendFactor, BlendOperation, ColorWriteMask, CpuCacheMode, DataType, FunctionType,
        HazardTrackingMode, PrimitiveTopologyClass, ResourceOptions, StorageMode, SwizzleChannels,
        TessellationControlPointIndexType, TessellationFactorFormat,
        TessellationFactorStepFunction, TessellationPartitionMode, TextureType, TextureUsage,
        Winding,
    },
    capabilities::block::Block,
    device::{
        FunctionId, LibraryId,
        fixture::{MockBackend, MockCompiler, device_on},
    },
    library::tests::fixture,
    pixel_format::PixelFormat,
    vulkan::command::Command,
};
use alloc::{vec, vec::Vec};
use core::num::NonZeroU64;
use stela_contracts::gate::MemoryRange;

mod commands;
mod deferral;
mod gate;
mod indirect;
mod lifecycle;
mod pipelines;
mod queue;
mod reflection;
mod resources;
mod sharing;

type TestService = Service<MockBackend, MockCompiler>;

/// A guest address space of one mapping at `BASE`.
#[derive(Default)]
struct Memory {
    bytes: Vec<u8>,
    /// The longest span a single read or write has moved.
    longest: u64,
}

impl Memory {
    const BASE: u64 = 0x1_0000;

    /// Maps `data` and spans it.
    fn put(&mut self, data: &[u8]) -> Span {
        let span = Span {
            address: Self::BASE + self.bytes.len() as u64,
            length: data.len() as u64,
        };
        self.bytes.extend_from_slice(data);
        span
    }

    fn records<T: Record>(&mut self, records: &[T]) -> Span {
        let mut data = vec![0; core::mem::size_of_val(records)];
        for (record, out) in records
            .iter()
            .zip(data.chunks_exact_mut(core::mem::size_of::<T>()))
        {
            wire::write(record, out).unwrap();
        }
        self.put(&data)
    }

    fn range(&self, span: Span) -> Result<core::ops::Range<usize>, Fault> {
        if span.length == 0 {
            return Ok(0..0);
        }
        let start = span
            .address
            .checked_sub(Self::BASE)
            .ok_or(Fault::malformed("unmapped"))? as usize;
        let end = start + span.length as usize;
        if end > self.bytes.len() {
            return Err(Fault::malformed("unmapped"));
        }
        Ok(start..end)
    }
}

impl GuestMemory for Memory {
    fn read(&mut self, span: Span) -> Result<Vec<u8>, Fault> {
        self.longest = self.longest.max(span.length);
        Ok(self.bytes[self.range(span)?].to_vec())
    }

    fn write(&mut self, span: Span, bytes: &[u8]) -> Result<(), Fault> {
        self.longest = self.longest.max(span.length);
        let range = self.range(span)?;
        self.bytes[range].copy_from_slice(bytes);
        Ok(())
    }
}

/// `length` bytes of wired guest RAM, as the kext imports them.
fn imported(service: &mut TestService, connection: NonZeroU64, length: u64) -> u64 {
    let range = MemoryRange {
        address: 0x8_0000_0000,
        bytes: length,
    };
    let token = NonZeroU64::MIN;
    service.import(connection, &[range], token).unwrap().get()
}

/// A buffer the CPU reaches, as the frontend makes one.
fn cpu_buffer(
    service: &mut TestService,
    connection: NonZeroU64,
    length: u64,
    storage_mode: StorageMode,
) -> BufferResponse {
    let request = NewImportedBufferRequest {
        memory: imported(service, connection, length),
        length,
        options: options(storage_mode),
    };
    call(
        service,
        connection,
        Operation::NewImportedBuffer,
        &request,
        &mut Memory::default(),
    )
    .unwrap()
}

fn options(storage_mode: StorageMode) -> u64 {
    ResourceOptions {
        cpu_cache_mode: CpuCacheMode::DefaultCache,
        storage_mode,
        hazard_tracking_mode: HazardTrackingMode::Default,
    }
    .raw()
}

const IDENTITY: [u8; 4] = [2, 3, 4, 5];

fn texture(storage_mode: StorageMode) -> TextureDescriptorRecord {
    assert_eq!(
        SwizzleChannels::IDENTITY.0.map(|swizzle| swizzle.raw()),
        IDENTITY
    );
    TextureDescriptorRecord {
        texture_type: TextureType::D2.raw(),
        pixel_format: PixelFormat::Rgba8Unorm.raw(),
        width: 8,
        height: 4,
        depth: 1,
        mipmap_level_count: 1,
        sample_count: 1,
        array_length: 1,
        options: options(storage_mode),
        usage: TextureUsage::SHADER_READ.bits(),
        swizzle: IDENTITY,
        reserved: 0,
    }
}

/// A render pipeline of `vertex` and `fragment` drawing to one RGBA8 color
/// attachment.
fn render_pipeline_request(vertex: u64, fragment: u64) -> NewRenderPipelineRequest {
    let unused = ColorAttachmentRecord {
        pixel_format: 0,
        source_rgb_blend_factor: BlendFactor::One.raw(),
        destination_rgb_blend_factor: BlendFactor::Zero.raw(),
        rgb_blend_operation: BlendOperation::Add.raw(),
        source_alpha_blend_factor: BlendFactor::One.raw(),
        destination_alpha_blend_factor: BlendFactor::Zero.raw(),
        alpha_blend_operation: BlendOperation::Add.raw(),
        write_mask: ColorWriteMask::KNOWN.bits(),
        blending_enabled: 0,
        reserved: 0,
    };
    let mut targets = RenderTargetsRecord {
        color_attachments: [unused; 8],
        raster_sample_count: 1,
        rasterization_enabled: 1,
        ..RenderTargetsRecord::default()
    };
    targets.color_attachments[0].pixel_format = PixelFormat::Rgba8Unorm.raw();
    NewRenderPipelineRequest {
        vertex_function: vertex,
        fragment_function: fragment,
        input_primitive_topology: PrimitiveTopologyClass::Unspecified.raw(),
        max_vertex_amplification_count: 1,
        targets,
        vertex_descriptor: Default::default(),
        tessellation: TessellationRecord {
            partition_mode: TessellationPartitionMode::Pow2.raw(),
            max_tessellation_factor: 16,
            factor_format: TessellationFactorFormat::Half.raw(),
            control_point_index_type: TessellationControlPointIndexType::None.raw(),
            factor_step_function: TessellationFactorStepFunction::Constant.raw(),
            output_winding_order: Winding::Clockwise.raw(),
            factor_scale_enabled: 0,
            reserved: 0,
        },
        vertex_linked_functions: LinkedFunctionsRecord::default(),
        fragment_linked_functions: LinkedFunctionsRecord::default(),
        archives: Default::default(),
        flags: PipelineFlags::EMPTY.bits(),
    }
}

/// A command or encoding stream under construction.
#[derive(Default)]
struct Stream(Vec<u8>);

impl Stream {
    /// An entry of `parts`, one after the other.
    fn entry(&mut self, code: u32, parts: &[&[u8]]) -> &mut Self {
        let length = parts.iter().map(|part| part.len()).sum::<usize>() as u32;
        self.0
            .extend_from_slice(wire::bytes(&CommandHeader { code, length }));
        for part in parts {
            self.0.extend_from_slice(part);
        }
        self.0.resize(self.0.len().next_multiple_of(8), 0);
        self
    }

    fn push<T: Record>(&mut self, code: u32, record: &T, data: &[u8]) -> &mut Self {
        self.entry(code, &[wire::bytes(record), data])
    }

    fn end(&mut self) -> &mut Self {
        self.push(CommandBufferCode::EndEncoding.raw(), &Empty {}, &[])
    }
}

fn commit(
    service: &mut TestService,
    connection: NonZeroU64,
    stream: &Stream,
) -> Result<Value, Outcome> {
    commit_on(service, connection, 1, stream)
}

/// Commits `stream` on the connection's Metal queue `queue`.
fn commit_on(
    service: &mut TestService,
    connection: NonZeroU64,
    queue: u64,
    stream: &Stream,
) -> Result<Value, Outcome> {
    let memory = &mut Memory::default();
    let request = CommitRequest {
        commands: memory.put(&stream.0),
        queue,
    };
    call(service, connection, Operation::Commit, &request, memory)
}

fn new_buffer(service: &mut TestService, connection: NonZeroU64, storage_mode: StorageMode) -> u64 {
    if storage_mode != StorageMode::Private {
        return cpu_buffer(service, connection, 256, storage_mode).handle;
    }
    let request = NewBufferRequest {
        length: 256,
        options: options(storage_mode),
    };
    let buffer: BufferResponse = call(
        service,
        connection,
        Operation::NewBuffer,
        &request,
        &mut Memory::default(),
    )
    .unwrap();
    buffer.handle
}

/// The commands of the last submission.
fn submitted(service: &mut TestService) -> Vec<Command> {
    let segments = service.device().backend().submissions.last().unwrap();
    segments
        .iter()
        .flat_map(|segment| segment.commands.iter().cloned())
        .collect()
}

fn service() -> TestService {
    service_on(MockBackend::new())
}

fn service_on(backend: MockBackend) -> TestService {
    Service::new(
        device_on(backend),
        DeviceEpoch::from_registered(NonZeroU64::MIN),
    )
}

/// Runs `operation` with `request`, decoding its response.
fn call<Q: Record, R: Record>(
    service: &mut TestService,
    connection: NonZeroU64,
    operation: Operation,
    request: &Q,
    memory: &mut Memory,
) -> Result<R, Outcome> {
    let mut request_bytes = vec![0; core::mem::size_of::<Q>()];
    wire::write(request, &mut request_bytes).unwrap();
    let mut response = vec![0; core::mem::size_of::<R>()];
    let outcome = service.call(
        connection.get(),
        operation.raw(),
        &request_bytes,
        &mut response,
        memory,
    );
    match outcome.status {
        Status::Ok => Ok(wire::read(&response).unwrap()),
        _ => Err(outcome),
    }
}

fn status<R>(result: Result<R, Outcome>) -> Status {
    result.map_or_else(|outcome| outcome.status, |_| Status::Ok)
}

/// A negotiated connection.
fn connect(service: &mut TestService) -> NonZeroU64 {
    let connection = service.connect().unwrap();
    let request = NegotiateRequest {
        major: wire::MAJOR,
        minor: wire::MINOR,
    };
    let _: NegotiateResponse = call(
        service,
        connection,
        Operation::Negotiate,
        &request,
        &mut Memory::default(),
    )
    .unwrap();
    connection
}

fn new_library(service: &mut TestService, connection: NonZeroU64, memory: &mut Memory) -> u64 {
    let request = NewLibraryRequest {
        data: memory.put(&fixture(false)),
    };
    let library: Handle =
        call(service, connection, Operation::NewLibrary, &request, memory).unwrap();
    library.handle
}

fn new_function(
    service: &mut TestService,
    connection: NonZeroU64,
    library: u64,
    name: &str,
    constants: &[ConstantRecord],
    memory: &mut Memory,
) -> Result<Handle, Outcome> {
    new_specialized_function(service, connection, library, name, constants, "", memory)
}

/// `newFunctionWithDescriptor:` with a `specializedName`, empty for none.
fn new_specialized_function(
    service: &mut TestService,
    connection: NonZeroU64,
    library: u64,
    name: &str,
    constants: &[ConstantRecord],
    specialized_name: &str,
    memory: &mut Memory,
) -> Result<Handle, Outcome> {
    let request = NewFunctionRequest {
        library,
        name: memory.put(name.as_bytes()),
        constants: memory.records(constants),
        specialized: 1,
        reserved: 0,
        specialized_name: match specialized_name {
            "" => Span::default(),
            name => memory.put(name.as_bytes()),
        },
    };
    call(
        service,
        connection,
        Operation::NewFunction,
        &request,
        memory,
    )
}

#[test]
fn connections_negotiate_before_anything_else() {
    let mut service = service();
    let memory = &mut Memory::default();
    let connection = service.connect().unwrap();
    let capabilities = |service: &mut TestService, memory: &mut Memory| {
        call::<_, Block>(
            service,
            connection,
            Operation::Capabilities,
            &Empty {},
            memory,
        )
    };
    assert_eq!(
        capabilities(&mut service, memory).unwrap_err().status,
        Status::Invalid
    );
    let negotiate = |service: &mut TestService, major| {
        let request = NegotiateRequest { major, minor: 7 };
        call::<_, NegotiateResponse>(
            service,
            connection,
            Operation::Negotiate,
            &request,
            &mut Memory::default(),
        )
    };
    assert_eq!(
        status(negotiate(&mut service, wire::MAJOR + 1)),
        Status::Unsupported
    );
    assert_eq!(
        negotiate(&mut service, wire::MAJOR),
        Ok(NegotiateResponse {
            major: wire::MAJOR,
            minor: wire::MINOR,
            capability_bytes: core::mem::size_of::<Block>() as u32,
            reserved: 0,
        })
    );
    // The block the frontend reads decodes to the device's capabilities.
    let block = capabilities(&mut service, memory).unwrap();
    assert_eq!(
        block.capabilities().as_ref(),
        Ok(service.device().capabilities())
    );
}

#[test]
fn calls_carry_exact_records_of_known_operations() {
    let mut service = service();
    let connection = connect(&mut service);
    let memory = &mut Memory::default();
    let mut response = [0; 8];
    let mut outcome = |connection, operation, request: &[u8], response: &mut [u8]| {
        service
            .call(connection, operation, request, response, memory)
            .status
    };
    let release = Operation::Release.raw();
    assert_eq!(
        outcome(connection.get(), release, &[0; 7], &mut []),
        Status::Malformed
    );
    assert_eq!(
        outcome(connection.get(), release, &[0; 8], &mut response),
        Status::Malformed
    );
    assert_eq!(
        outcome(connection.get(), 0xffff, &[], &mut []),
        Status::UnknownOperation
    );
    assert_eq!(outcome(0, release, &[0; 8], &mut []), Status::Closed);
    assert_eq!(
        outcome(connection.get() + 1, release, &[0; 8], &mut []),
        Status::Closed
    );
    // Handle zero names nothing.
    assert_eq!(
        outcome(connection.get(), release, &[0; 8], &mut []),
        Status::StaleHandle
    );
}

#[test]
fn source_libraries_arrive_as_air() {
    let mut service = service();
    let connection = connect(&mut service);
    let memory = &mut Memory::default();
    let bytes = crate::metal::library::tests::dynamic(b"lib/stela.metallib\0");
    let data = memory.put(&bytes);
    let dependency = call::<_, Handle>(
        &mut service,
        connection,
        Operation::NewLibrary,
        &NewLibraryRequest { data },
        memory,
    )
    .unwrap()
    .handle;

    let request = NewSourceLibraryRequest {
        air: memory.put(b"BC\xc0\xde\0\0\0\0"),
        install_name: memory.put(b"lib/source"),
        libraries: memory.put(&dependency.to_le_bytes()),
        conversion_rounding: 1,
    };
    let library = call::<_, Handle>(
        &mut service,
        connection,
        Operation::NewSourceLibrary,
        &request,
        memory,
    )
    .unwrap()
    .handle;
    let out = memory.put(&[0; 10]);
    let identity: LibraryIdentityResponse = call(
        &mut service,
        connection,
        Operation::LibraryIdentity,
        &LibraryIdentityRequest { library, out },
        memory,
    )
    .unwrap();
    assert_eq!((identity.library_type, identity.length), (1, 10));
    assert_eq!(memory.read(out).unwrap(), b"lib/source");
}

#[test]
fn dynamic_libraries_name_themselves_and_read_back() {
    let mut service = service();
    let connection = connect(&mut service);
    let memory = &mut Memory::default();
    let bytes = crate::metal::library::tests::dynamic(b"lib/stela.metallib\0");
    let data = memory.put(&bytes);
    let library = call::<_, Handle>(
        &mut service,
        connection,
        Operation::NewLibrary,
        &NewLibraryRequest { data },
        memory,
    )
    .unwrap()
    .handle;

    let out = memory.put(&[0xee; 20]);
    let identity: LibraryIdentityResponse = call(
        &mut service,
        connection,
        Operation::LibraryIdentity,
        &LibraryIdentityRequest { library, out },
        memory,
    )
    .unwrap();
    assert_eq!((identity.library_type, identity.length), (1, 18));
    assert_eq!(memory.read(out).unwrap(), b"lib/stela.metallib\xee\xee");

    let out = memory.put(&vec![0; bytes.len()]);
    let length: Length = call(
        &mut service,
        connection,
        Operation::LibraryData,
        &LibraryDataRequest { library, out },
        memory,
    )
    .unwrap();
    assert_eq!(length.length, bytes.len() as u64);
    assert_eq!(memory.read(out).unwrap(), bytes);
}

#[test]
fn libraries_list_and_make_their_functions() {
    let mut service = service();
    let connection = connect(&mut service);
    let memory = &mut Memory::default();
    let library = new_library(&mut service, connection, memory);

    // Too short a span receives nothing but the length.
    let mut names = |memory: &mut Memory, capacity| {
        let out = memory.put(&vec![0xee; capacity]);
        let request = LibraryFunctionNamesRequest { library, out };
        let length: Length = call(
            &mut service,
            connection,
            Operation::LibraryFunctionNames,
            &request,
            memory,
        )
        .unwrap();
        (length.length, memory.read(out).unwrap().to_vec())
    };
    assert_eq!(names(memory, 3), (7, vec![0xee; 3]));
    assert_eq!(names(memory, 8), (7, b"kernel\0\xee".to_vec()));

    let scale = |memory: &mut Memory, name: &str, data_type: DataType| ConstantRecord {
        data_type: data_type.raw(),
        index: 0,
        name: memory.put(name.as_bytes()),
        value: memory.put(&2u32.to_le_bytes()),
    };
    let named = scale(memory, "scale", DataType::UInt);
    let function = new_function(
        &mut service,
        connection,
        library,
        "constants",
        &[named],
        memory,
    );
    assert!(function.is_ok());
    let indexed = scale(memory, "", DataType::UInt);
    assert!(
        new_function(
            &mut service,
            connection,
            library,
            "constants",
            &[indexed],
            memory
        )
        .is_ok()
    );
    let mistyped = scale(memory, "", DataType::Int);
    assert_eq!(
        new_function(
            &mut service,
            connection,
            library,
            "constants",
            &[mistyped],
            memory
        ),
        Err(Outcome {
            status: Status::Invalid,
            reason: "a function constant value of another type",
            raw: None,
        })
    );
    let unknown = ConstantRecord {
        data_type: 1000,
        ..indexed
    };
    assert_eq!(
        status(new_function(
            &mut service,
            connection,
            library,
            "constants",
            &[unknown],
            memory
        )),
        Status::Malformed
    );
    let not_utf8 = ConstantRecord {
        name: memory.put(&[0xff]),
        ..indexed
    };
    assert_eq!(
        status(new_function(
            &mut service,
            connection,
            library,
            "constants",
            &[not_utf8],
            memory
        )),
        Status::Malformed
    );
    // A name the library lacks is Metal's nil, which the service notes
    // rather than refuses.
    assert_eq!(
        new_function(&mut service, connection, library, "missing", &[], memory),
        Err(Outcome {
            status: Status::Declined,
            reason: "no function of that name",
            raw: None,
        })
    );
    // A function's handle names no library.
    let function = function.unwrap().handle;
    assert_eq!(
        status(new_function(
            &mut service,
            connection,
            function,
            "kernel",
            &[],
            memory
        )),
        Status::StaleHandle
    );
}

#[test]
fn functions_take_their_descriptors_specialized_names() {
    let mut service = service();
    let connection = connect(&mut service);
    let memory = &mut Memory::default();
    let library = new_library(&mut service, connection, memory);
    let mut named = |entry, name| {
        let handle =
            new_specialized_function(&mut service, connection, library, entry, &[], name, memory)
                .unwrap()
                .handle;
        let id = service.object::<FunctionId>(connection, handle).unwrap();
        alloc::string::String::from(service.device().function_name(id).unwrap())
    };
    assert_eq!(named("visible", "renamed"), "renamed");
    assert_eq!(named("visible", ""), "visible");
    // A stage is linked by no name.
    assert_eq!(named("kernel", "renamed_kernel"), "kernel");
}

#[test]
fn functions_reflect_their_constants_and_patches() {
    let mut service = service();
    let connection = connect(&mut service);
    let memory = &mut Memory::default();
    let library = new_library(&mut service, connection, memory);
    let scale = ConstantRecord {
        data_type: DataType::UInt.raw(),
        index: 0,
        name: Span::default(),
        value: memory.put(&[2, 0, 0, 0]),
    };
    let constants = new_function(
        &mut service,
        connection,
        library,
        "constants",
        &[scale],
        memory,
    )
    .unwrap()
    .handle;
    let patch = new_function(&mut service, connection, library, "patch", &[], memory)
        .unwrap()
        .handle;
    // The hash of the function's AIR module, as the device's compiler gives it.
    let id = service
        .object::<FunctionId>(connection, constants)
        .ok()
        .unwrap();
    let hash = service.device().function_slice_hash(id).unwrap();
    let mut reflect = |function, capacity| {
        let out = memory.put(&vec![0; capacity]);
        let request = FunctionReflectionRequest { function, out };
        let response: FunctionReflectionResponse = call(
            &mut service,
            connection,
            Operation::FunctionReflection,
            &request,
            memory,
        )
        .unwrap();
        (response, memory.read(out).unwrap().to_vec())
    };
    // Two constants' records, then "scale" and "tint".
    let (response, nothing) = reflect(constants, 8);
    assert_eq!(nothing, [0; 8]);
    let length = 2 * size_of::<ConstantInfo>() + 9;
    assert_eq!(
        response,
        FunctionReflectionResponse {
            function_type: FunctionType::Kernel.raw(),
            patch_type: 0,
            patch_control_points: 0,
            constant_count: 2,
            attribute_count: 0,
            length: length as u64,
            bitcode_hash: hash,
        }
    );
    let (_, data) = reflect(constants, length);
    let infos: Vec<ConstantInfo> =
        wire::read_array(&data[..2 * size_of::<ConstantInfo>()]).unwrap();
    let tint = infos[1];
    assert_eq!(
        (tint.index, tint.required, tint.data_type),
        (1, 0, DataType::Half3.raw())
    );
    let name = tint.name as usize..(tint.name + tint.name_length) as usize;
    assert_eq!(&data[name], b"tint");

    let (response, _) = reflect(patch, 0);
    assert_eq!((response.patch_type, response.patch_control_points), (2, 4));
}

#[test]
fn a_report_reaches_the_platform_as_a_refusal_does() {
    let mut service = service();
    let connection = connect(&mut service);
    let memory = &mut Memory::default();
    let text = memory.put(b"newTextureWithDescriptor: is absent");
    call::<_, Empty>(
        &mut service,
        connection,
        Operation::Report,
        &ReportRequest { text },
        memory,
    )
    .unwrap();
    assert_eq!(
        core::mem::take(&mut service.refused),
        ["newTextureWithDescriptor: is absent"]
    );
}

#[test]
fn released_handles_name_nothing() {
    let mut service = service();
    let connection = connect(&mut service);
    let memory = &mut Memory::default();
    let library = new_library(&mut service, connection, memory);
    let function = new_function(&mut service, connection, library, "kernel", &[], memory)
        .unwrap()
        .handle;
    let release = |service: &mut TestService, handle| {
        call::<_, Empty>(
            service,
            connection,
            Operation::Release,
            &Handle { handle },
            &mut Memory::default(),
        )
    };
    // The function keeps the library it was made from.
    release(&mut service, library).unwrap();
    assert!(
        service
            .device()
            .library_function_names(LibraryId(1))
            .is_ok()
    );
    assert_eq!(status(release(&mut service, library)), Status::StaleHandle);
    assert_eq!(
        status(new_function(
            &mut service,
            connection,
            library,
            "kernel",
            &[],
            memory
        )),
        Status::StaleHandle
    );
    release(&mut service, function).unwrap();
    assert!(
        service
            .device()
            .library_function_names(LibraryId(1))
            .is_err()
    );
}

#[test]
fn connections_hold_their_own_objects_until_they_close() {
    let mut service = service();
    let (first, second) = (connect(&mut service), connect(&mut service));
    let memory = &mut Memory::default();
    let library = new_library(&mut service, first, memory);
    new_function(&mut service, first, library, "kernel", &[], memory).unwrap();
    assert_eq!(
        status(new_function(
            &mut service,
            second,
            library,
            "kernel",
            &[],
            memory
        )),
        Status::StaleHandle
    );
    service.disconnect(first).unwrap();
    assert!(
        service
            .device()
            .library_function_names(LibraryId(1))
            .is_err()
    );
    assert_eq!(
        status(new_function(
            &mut service,
            first,
            library,
            "kernel",
            &[],
            memory
        )),
        Status::Closed
    );
    assert_eq!(service.disconnect(first), Err(Fault::CLOSED));
}

/// A heap with no large free block on the thread that asks for one: every
/// allocation of at least a size is refused there, for what must work
/// without such a block. Other threads allocate as usual.
pub(super) mod heap {
    extern crate std;
    use core::alloc::{GlobalAlloc, Layout};
    use core::cell::Cell;

    std::thread_local! {
        static REFUSED_FROM: Cell<usize> = const { Cell::new(usize::MAX) };
        static REFUSED: Cell<usize> = const { Cell::new(0) };
    }

    struct Capped;

    #[global_allocator]
    static ALLOCATOR: Capped = Capped;

    fn refuses(size: usize) -> bool {
        let refused = REFUSED_FROM
            .try_with(|from| size >= from.get())
            .unwrap_or(false);
        if refused {
            let _ = REFUSED.try_with(|count| count.set(count.get() + 1));
        }
        refused
    }

    // SAFETY: every call forwards to `System` unchanged, or refuses with null.
    unsafe impl GlobalAlloc for Capped {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            if refuses(layout.size()) {
                return core::ptr::null_mut();
            }
            // SAFETY: the caller's `GlobalAlloc::alloc` contract, forwarded.
            unsafe { std::alloc::System.alloc(layout) }
        }

        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            // SAFETY: `pointer` came from `System` with `layout`.
            unsafe { std::alloc::System.dealloc(pointer, layout) }
        }

        unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            if refuses(size) {
                return core::ptr::null_mut();
            }
            // SAFETY: the caller's `GlobalAlloc::realloc` contract, forwarded.
            unsafe { std::alloc::System.realloc(pointer, layout, size) }
        }
    }

    /// Runs `body` with every allocation of `bytes` or more refused on this
    /// thread, and says how many were.
    pub(in crate::metal::service) fn without_blocks_of<R>(
        bytes: usize,
        body: impl FnOnce() -> R,
    ) -> (R, usize) {
        REFUSED.set(0);
        REFUSED_FROM.set(bytes);
        let result = body();
        REFUSED_FROM.set(usize::MAX);
        (result, REFUSED.get())
    }
}
