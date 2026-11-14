//! Libraries and their functions.
use super::{
    Call, Object, Service, StagedLibrary, decode,
    wire::{
        self, ArchiveDataRequest, AttributeInfo, ConstantInfo, ConstantRecord, Fault,
        FunctionReflectionRequest, FunctionReflectionResponse, LibraryDataRequest,
        LibraryFunctionNamesRequest, LibraryIdentityRequest, LibraryIdentityResponse,
        NewArchiveRequest, NewFunctionRequest, NewLibraryRequest, NewSourceLibraryRequest,
        NewStitchedLibraryRequest, Span, Status,
    },
};
use crate::metal::{
    abi::{ConversionRounding, DataType},
    device::{
        ArchiveId, FunctionId, LibraryId,
        pipelines::{ARCHIVE_HEADER_BYTES, ConstantKey, ConstantValue, library_digest},
    },
    error::NO_FUNCTION,
    shader::{Compiler, PatchDomain, Reflection},
    stitching,
    timing::{Detail, Name, Timed},
    vulkan::backend::Backend,
};
use alloc::{string::String, vec::Vec};

/// Which names of a library `LibraryFunctionNames` and the operations
/// like it list.
#[derive(Clone, Copy)]
pub(super) enum LibraryNames {
    Functions,
    Externs,
    Linked,
}

impl<B: Backend, C: Compiler> Service<B, C> {
    pub(super) fn new_library(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewLibraryRequest = call.request()?;
        // Read before the turn when the shared service serves the call;
        // otherwise here, whole.
        let staged = match self.staged_library.take() {
            Some(staged) if staged.span == request.data => staged,
            _ => {
                let bytes = call.memory.read(request.data)?;
                let digest = library_digest(&bytes);
                StagedLibrary {
                    span: request.data,
                    bytes: Some(bytes),
                    digest,
                }
            }
        };
        let length = request.data.length;
        let shared = self.device.share_library(length, &staged.digest)?;
        let library = match (shared, staged.bytes) {
            (Some(library), _) => library,
            (None, Some(bytes)) => self.device.new_library_digested(bytes, staged.digest)?,
            // Only digested: the bytes are read again, kept, before the
            // call's next turn.
            (None, None) => {
                self.library_wanted = Some(request.data);
                return Err(Fault::new(Status::Pending, ""));
            }
        };
        let handle = self.hold(call.connection, Object::Library(library))?;
        call.reply(&handle)
    }

    pub(super) fn new_source_library(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewSourceLibraryRequest = call.request()?;
        let memory = &mut *call.memory;
        let air = memory.read(request.air)?;
        let install_name = (request.install_name.length != 0)
            .then(|| text(&memory.read(request.install_name)?))
            .transpose()?;
        let libraries = wire::read_array::<u64>(&memory.read(request.libraries)?)?
            .into_iter()
            .map(|handle| self.object::<LibraryId>(call.connection, handle))
            .collect::<Result<Vec<_>, Fault>>()?;
        let rounding = decode(
            ConversionRounding::from_raw,
            request.conversion_rounding,
            "conversion rounding mode",
        )?;
        let library =
            self.device
                .new_source_library(&air, install_name.as_deref(), &libraries, rounding)?;
        let handle = self.hold(call.connection, Object::Library(library))?;
        call.reply(&handle)
    }

    pub(super) fn new_stitched_library(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewStitchedLibraryRequest = call.request()?;
        let memory = &mut *call.memory;
        let functions = wire::read_array::<u64>(&memory.read(request.functions)?)?
            .into_iter()
            .map(|handle| self.object::<FunctionId>(call.connection, handle))
            .collect::<Result<Vec<_>, Fault>>()?;
        let graphs = stitching::decode(&memory.read(request.graphs)?, functions.len())?;
        let library = self.device.new_stitched_library(&functions, &graphs)?;
        let handle = self.hold(call.connection, Object::Library(library))?;
        call.reply(&handle)
    }

    /// The names of the library's functions, its extern functions or the
    /// dynamic libraries it links, as `names` says.
    pub(super) fn library_names(
        &mut self,
        call: &mut Call,
        names: LibraryNames,
    ) -> Result<(), Fault> {
        let request: LibraryFunctionNamesRequest = call.request()?;
        let library = self.object::<LibraryId>(call.connection, request.library)?;
        let listed = match names {
            LibraryNames::Functions => self.device.library_function_names(library)?,
            LibraryNames::Externs => self.device.library_extern_function_names(library)?,
            LibraryNames::Linked => self.device.library_linked_names(library)?,
        };
        let mut names = Vec::new();
        for name in listed {
            names.extend_from_slice(name.as_bytes());
            names.push(0);
        }
        call.reply_data(request.out, &names)
    }

    pub(super) fn library_identity(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: LibraryIdentityRequest = call.request()?;
        let library = self.object::<LibraryId>(call.connection, request.library)?;
        let (dynamic, install_name) = self.device.library_identity(library)?;
        let install_name = install_name.unwrap_or_default().as_bytes();
        call.write_fitting(request.out, install_name)?;
        call.reply(&LibraryIdentityResponse {
            // `MTLLibraryType`.
            library_type: dynamic.into(),
            length: install_name.len() as u64,
        })
    }

    pub(super) fn library_data(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: LibraryDataRequest = call.request()?;
        let library = self.object::<LibraryId>(call.connection, request.library)?;
        call.reply_data(request.out, self.device.library_data(library)?)
    }

    pub(super) fn new_archive(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: NewArchiveRequest = call.request()?;
        // Another device's archive, as large as its pipelines, holds none
        // of this one's: only its header is read.
        let span = request.data;
        let header = call.memory.read(Span {
            address: span.address,
            length: span.length.min(ARCHIVE_HEADER_BYTES),
        })?;
        let data = match self.device.archive_header_matches(&header) {
            true => call.memory.read(span)?,
            false => Vec::new(),
        };
        let archive = self.device.new_archive(&data)?;
        let handle = self.hold(call.connection, Object::Archive(archive))?;
        call.reply(&handle)
    }

    pub(super) fn archive_data(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: ArchiveDataRequest = call.request()?;
        let archive = self.object::<ArchiveId>(call.connection, request.archive)?;
        let data = self.device.archive_data(archive)?;
        call.reply_data(request.out, &data)
    }

    pub(super) fn new_function(&mut self, call: &mut Call) -> Result<(), Fault> {
        self.device.timing().begin(Timed::NewFunction);
        let request: NewFunctionRequest = call.request()?;
        let library = self.object::<LibraryId>(call.connection, request.library)?;
        let memory = &mut *call.memory;
        let name = text(&memory.read(request.name)?)?;
        let values = wire::read_array::<ConstantRecord>(&memory.read(request.constants)?)?
            .into_iter()
            .map(|record| {
                let key = if record.name.length == 0 {
                    let index = u32::try_from(record.index).map_err(|_| Fault::MALFORMED)?;
                    ConstantKey::Index(index)
                } else {
                    ConstantKey::Name(text(&memory.read(record.name)?)?)
                };
                Ok(ConstantValue {
                    key,
                    data_type: DataType::from_raw(record.data_type).ok_or(
                        Fault::malformed("function constant data type").value(record.data_type),
                    )?,
                    value: memory.read(record.value)?,
                })
            })
            .collect::<Result<Vec<_>, Fault>>()?;
        let values = (request.specialized != 0).then_some(values.as_slice());
        let specialized = match request.specialized_name.length {
            0 => None,
            _ => Some(text(&memory.read(request.specialized_name)?)?),
        };
        self.device.timing().detail(
            Timed::NewFunction,
            Detail::Function {
                entry: Name::new(&name),
            },
        );
        // A name the library lacks is Metal's nil answer, not a refusal:
        // Core Image looks for its kernels library by library.
        let function =
            self.device
                .new_function(library, &name, values)
                .map_err(|error| match error {
                    NO_FUNCTION => Fault::new(Status::Declined, "no function of that name"),
                    error => error.into(),
                })?;
        if let Some(specialized) = &specialized {
            self.device.name_function(function, specialized)?;
        }
        let handle = self.hold(call.connection, Object::Function(function))?;
        call.reply(&handle)
    }

    pub(super) fn function_reflection(&mut self, call: &mut Call) -> Result<(), Fault> {
        let request: FunctionReflectionRequest = call.request()?;
        let function = self.object::<FunctionId>(call.connection, request.function)?;
        let reflection = self.device.function_reflection(function)?;
        let (patch, attributes) = match reflection {
            Reflection::Stage(interface) => {
                (interface.patch, interface.declared_attributes.as_slice())
            }
            Reflection::Visible { .. } => (None, &[][..]),
        };
        let constants = reflection.constants();
        // Records first, then the names they point into.
        let mut names = Names {
            base: constants.len() * size_of::<ConstantInfo>()
                + attributes.len() * size_of::<AttributeInfo>(),
            bytes: Vec::new(),
        };
        let mut data = Vec::with_capacity(names.base);
        for constant in constants {
            let (name, name_length) = names.add(&constant.name)?;
            data.extend_from_slice(wire::bytes(&ConstantInfo {
                name,
                name_length,
                data_type: constant.data_type.raw(),
                index: constant.index,
                required: constant.required.into(),
            }));
        }
        for attribute in attributes {
            let (name, name_length) = names.add(&attribute.name)?;
            data.extend_from_slice(wire::bytes(&AttributeInfo {
                name,
                name_length,
                data_type: attribute.data_type.raw(),
                index: attribute.index,
                active: attribute.active.into(),
                patch_control_point: attribute.patch_control_point.into(),
                reserved: 0,
            }));
        }
        data.extend_from_slice(&names.bytes);
        let response = FunctionReflectionResponse {
            function_type: reflection.function_type().raw(),
            // `MTLPatchType`.
            patch_type: match patch.map(|patch| patch.domain) {
                None => 0,
                Some(PatchDomain::Triangle) => 1,
                Some(PatchDomain::Quad) => 2,
            },
            patch_control_points: patch
                .and_then(|patch| patch.control_points)
                .map_or(0, u64::from),
            constant_count: constants.len() as u32,
            attribute_count: attributes.len() as u32,
            length: data.len() as u64,
            bitcode_hash: self.device.function_slice_hash(function)?,
        };
        call.write_fitting(request.out, &data)?;
        call.reply(&response)
    }
}

/// The names of a reflection, which follow its `base` bytes of records.
pub(super) struct Names {
    pub(super) base: usize,
    pub(super) bytes: Vec<u8>,
}

impl Names {
    /// Adds `name`, returning its offset and length in the reflection.
    pub(super) fn add(&mut self, name: &str) -> Result<(u32, u32), Fault> {
        let offset = u32::try_from(self.base + self.bytes.len()).map_err(|_| {
            Fault::malformed("reflection too large").value((self.base + self.bytes.len()) as u64)
        })?;
        self.bytes.extend_from_slice(name.as_bytes());
        Ok((offset, name.len() as u32))
    }
}

/// UTF-8 text.
fn text(bytes: &[u8]) -> Result<String, Fault> {
    core::str::from_utf8(bytes)
        .map(String::from)
        .map_err(|_| Fault::malformed("text that is not UTF-8"))
}
