//! A remote install image (`payload::Layout::Remote`): the IPSW on Apple's
//! CDN, read by URL-aware byte ranges over the mount's own network. HTTPS
//! requires the native owner's admitted TLS time and entropy capabilities.
//!
//! Each request carries `If-Range` with the entity tag setup recorded, so a
//! changed object fails instead of mixing two versions; the restore store
//! then checks the bytes as the manifest says (`payload::Member::sha256`).
//! The ranges, their read-ahead and the bounded reads of a PE that serves
//! the restore host are `stela_runtime::net::ranges::Ranges`; this is only
//! its transport over the mount's network and serial output.
use super::network::Shared;
use crate::Result;
use crate::output;
use alloc::string::String;
use stela_contracts::refuse;
use {
    stela_formats::payload::Layout,
    stela_formats::payload::PayloadManifest,
    stela_formats::payload::ReadAt,
    stela_runtime::net::HttpTransfer,
    stela_runtime::net::NetError,
    stela_runtime::net::http::Ranged,
    stela_runtime::net::ranges::{RangeSource, Ranges},
};

/// Ranges of one URL whose entity tag setup recorded.
pub struct Http {
    network: Shared,
    url: String,
    etag: String,
    len: u64,
}

impl RangeSource for Http {
    type Range = (HttpTransfer, Ranged);

    fn open(&mut self, first: u64, last: u64) -> Result<Self::Range, NetError> {
        let ranged = Ranged::with_identity(first, last, &self.etag, self.len)?;
        let transfer = self
            .network
            .0
            .borrow_mut()
            .download(&self.url, first, last, &self.etag)?;
        Ok((transfer, ranged))
    }

    fn read(
        &mut self,
        (transfer, ranged): &mut Self::Range,
        take: &mut dyn FnMut(&[u8]),
    ) -> Result<bool, NetError> {
        self.network
            .0
            .borrow_mut()
            .read_download(transfer, |bytes, closed| {
                ranged.feed(bytes, closed, &mut *take)
            })
    }

    fn close(&mut self, (transfer, _): Self::Range) {
        self.network.0.borrow_mut().close_download(transfer);
    }

    fn wait(&mut self, micros: u64) -> Result<(), NetError> {
        self.network.0.borrow_mut().wait(micros)
    }

    fn millis(&mut self) -> u64 {
        output::millis_since(0)
    }

    fn note(&mut self, line: core::fmt::Arguments<'_>) {
        output::arguments(line);
    }
}

pub struct Remote(Ranges<Http>);

impl Remote {
    pub fn new(network: Shared, manifest: &PayloadManifest) -> Result<Self> {
        let Layout::Remote { url, etag } = &manifest.layout else {
            return Err(refuse!("the install payload is not a remote image"));
        };
        network
            .0
            .borrow()
            .check_url(url)
            .map_err(|error| refuse!("remote image URL: {error}", %error))?;
        let last = manifest
            .source
            .bytes
            .checked_sub(1)
            .ok_or_else(|| refuse!("the remote image is empty"))?;
        Ranged::with_identity(0, last, etag, manifest.source.bytes)
            .map_err(|error| refuse!("remote image range: {error}", %error))?;
        output::arguments(format_args!(
            "REMOTE IMAGE {url} bytes={} etag={etag}\n",
            manifest.source.bytes
        ));
        let http = Http {
            network,
            url: url.clone(),
            etag: etag.clone(),
            len: manifest.source.bytes,
        };
        Ok(Self(Ranges::new(http, manifest.source.bytes)))
    }

    /// Called between restore polls, outside the guest devices' lock.
    pub fn diagnostics(&mut self) -> String {
        let mut line = alloc::format!(
            "source=remote ranges={} buffered={} kept={}",
            self.0.ranges(),
            self.0.buffered(),
            self.0.kept()
        );
        if let Ok(mut network) = self.0.source_mut().network.0.try_borrow_mut()
            && let Some(nic) = network.adapter_diagnostics()
        {
            line.push_str(&alloc::format!(" {nic}"));
        }
        line
    }

    /// Have every later read wait for the network at most
    /// `ranges::BUDGET_MS`: from when the restore host serves on a PE.
    pub fn bound(&mut self) {
        self.0.bound();
    }

    /// Release the restore-only ranges and buffers after the protocol has
    /// ended.
    pub(super) fn retire(&mut self) {
        let bytes = self.0.retire();
        output::arguments(format_args!(
            "REMOTE IMAGE retired read-ahead capacity={bytes} ranges={}\n",
            self.0.ranges()
        ));
    }

    /// Without waiting, takes what has arrived for the open ranges.
    pub fn pump(&mut self) {
        self.0.pump();
    }
}

impl ReadAt for Remote {
    type Error = NetError;

    fn len(&self) -> u64 {
        self.0.len()
    }

    fn blocked(&mut self) -> bool {
        self.0.blocked()
    }

    fn read_at(&mut self, offset: u64, out: &mut [u8]) -> Result<(), NetError> {
        self.0.read_at(offset, out)
    }
}
