//! Request-driven restore host, independent of USB and guest execution.
//!
//! Wire evidence: libimobiledevice property_list_service.c/restore.c
//! (`fa0f791`) and idevicerestore restore.c/asr.c (`4b3e847`). No
//! signing-service access exists here.
//! An adapter supplies mux streams; each call performs bounded work so the
//! caller can schedule control, ASR and independent DataPort jobs fairly.
pub mod answers;
pub mod asr;
pub mod driver;
pub mod fdr;
pub mod native;
pub mod plist;
pub mod proxy;
pub mod progress;
pub mod route;
pub mod session;
pub mod source;
pub mod store;
pub mod stream;
pub mod ticket;
pub mod tss;

use alloc::{
    boxed::Box,
    format,
    string::{String, ToString},
};
use plist::Value;
use stela_contracts::Refusal;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    Invalid(&'static str),
    Limit(&'static str),
    Transport(Refusal),
    Payload(Refusal),
    /// Preserve the entire bounded message, including updater, arguments,
    /// DataPort and asynchronous context. Never a blank successful response.
    Refused {
        reason: Refusal,
        request: Box<Value>,
    },
    /// Not a failure: the answer waits for a signing exchange in flight,
    /// as a non-blocking socket's operation would block. The same request
    /// asked again continues it.
    Pending,
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl Error {
    /// The error in one line, without a refused request's message.
    pub fn summary(&self) -> String {
        match self {
            Self::Invalid(what) => format!("invalid {what}"),
            Self::Limit(what) => format!("over the limit of {what}"),
            Self::Transport(why) => format!("transport: {why}"),
            Self::Payload(why) => format!("payload: {why}"),
            Self::Refused { reason, .. } => reason.to_string(),
            Self::Pending => String::from("waiting for a signing exchange"),
        }
    }
}
pub type Result<T> = core::result::Result<T, Error>;
pub fn refuse(reason: impl Into<Refusal>, request: &Value) -> Error {
    Error::Refused {
        reason: reason.into(),
        request: Box::new(request.clone()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_summary_is_one_line_without_the_refused_request() {
        let refused = refuse("device final status is failure", &Value::Integer(6));
        assert_eq!(refused.summary(), "device final status is failure");
        assert_eq!(
            Error::Invalid("identity number").summary(),
            "invalid identity number"
        );
    }
}
