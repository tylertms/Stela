//! What every HTTP request `stela` makes shares: a request that fails in
//! transit, or that a server refuses for now, is asked again with growing
//! waits, and a failure says its cause in words a user can act on (no
//! network, a name that does not resolve, a network that intercepts HTTPS).
use anyhow::{Result, anyhow};
use reqwest::{
    StatusCode,
    blocking::{RequestBuilder, Response},
};
use std::{error::Error, io, time::Duration};

/// Attempts per request: with waits of 1, 2, 4 and 8 s between them, a
/// network that drops for about 15 s costs a pause, not the setup.
pub(crate) const ATTEMPTS: u32 = 5;

/// The first wait between attempts; each later one doubles.
fn first_wait() -> Duration {
    if cfg!(test) {
        Duration::from_millis(10)
    } else {
        Duration::from_secs(1)
    }
}

/// A server's answer that the same request may not get again: a server
/// error, a timeout answer, or a rate limit.
pub(crate) fn transient_status(status: StatusCode) -> bool {
    status.is_server_error()
        || status == StatusCode::REQUEST_TIMEOUT
        || status == StatusCode::TOO_MANY_REQUESTS
}

/// A failure the same request may not meet again: the connection, a
/// timeout, or the transfer, never a refused redirect or a bad request.
pub(crate) fn transient(error: &reqwest::Error) -> bool {
    !error.is_redirect()
        && !error.is_builder()
        && (error.is_connect() || error.is_timeout() || error.is_request() || error.is_body())
}

/// Send the request `build` makes, asking again after a transient failure
/// ([`transient`], [`transient_status`]) up to [`ATTEMPTS`] times. Returns
/// the first answer that is not transient, whatever its status: callers
/// check it. `what` names the request in the error.
pub(crate) fn send(what: &str, build: impl Fn() -> RequestBuilder) -> Result<Response> {
    let mut wait = first_wait();
    let mut attempt = 1;
    loop {
        let failure = match build().send() {
            Ok(response) if transient_status(response.status()) && attempt < ATTEMPTS => {
                format!("{} answered {}", host_of(response.url()), response.status())
            }
            Ok(response) => return Ok(response),
            Err(error) if transient(&error) && attempt < ATTEMPTS => explain(&error),
            Err(error) => {
                let tries = match attempt {
                    1 => String::new(),
                    n => format!(" after {n} attempts"),
                };
                return Err(anyhow!("{what} failed{tries}: {}", explain(&error)));
            }
        };
        eprintln!(
            "note: {what}: {failure}; trying again in {} s",
            wait.as_secs().max(1)
        );
        std::thread::sleep(wait);
        wait *= 2;
        attempt += 1;
    }
}

fn host_of(url: &reqwest::Url) -> String {
    url.host_str().unwrap_or("the server").to_owned()
}

/// `error` with every cause under it, and what a user can do about the
/// kinds of failure a network causes.
pub(crate) fn explain(error: &reqwest::Error) -> String {
    explain_causes(error, error.is_timeout())
}

/// Whether `cause` is the operating system finding no way to the server. Its
/// kind says so on every system; its text is the system's own, which Windows
/// writes in the user's language.
fn unreachable(cause: &(dyn Error + 'static)) -> bool {
    cause.downcast_ref::<io::Error>().is_some_and(|error| {
        matches!(
            error.kind(),
            io::ErrorKind::ConnectionRefused
                | io::ErrorKind::NetworkUnreachable
                | io::ErrorKind::HostUnreachable
        )
    })
}

/// Whether a TLS failure's lower-case text says the certificate's dates do
/// not cover this computer's clock: rustls (`certificate expired`,
/// `certificate not valid yet`, `Expired`, `NotValidYet`), OpenSSL
/// (`certificate has expired`, `certificate is not yet valid`), SChannel
/// (`not within its validity period`) and Security.framework (`expired`).
fn clock_wrong(lower: &str) -> bool {
    let tls = lower.contains("certificate") || lower.contains("tls") || lower.contains("ssl");
    tls && [
        "expired",
        "not valid yet",
        "not yet valid",
        "notvalidyet",
        "validity period",
    ]
    .iter()
    .any(|words| lower.contains(words))
}

/// [`explain`] for any error, one that ran out of time when `timed_out`.
pub(crate) fn explain_causes(error: &(dyn Error + 'static), timed_out: bool) -> String {
    let mut text = error.to_string();
    let mut no_way = unreachable(error);
    let mut source = error.source();
    while let Some(cause) = source {
        let cause_text = cause.to_string();
        if !text.contains(&cause_text) {
            text.push_str(": ");
            text.push_str(&cause_text);
        }
        no_way |= unreachable(cause);
        source = cause.source();
    }
    let lower = text.to_ascii_lowercase();
    let hint = if lower.contains("dns error")
        || lower.contains("failed to lookup address")
        || lower.contains("name or service not known")
        || lower.contains("no such host")
        || lower.contains("nodename nor servname")
    {
        Some("this computer cannot look up the server's name: check its network connection")
    } else if clock_wrong(&lower) {
        Some(
            "the server's HTTPS certificate is not valid at this computer's date and time: check \
             that its clock is right",
        )
    } else if lower.contains("certificate") || lower.contains("unknownissuer") {
        Some(
            "the server's HTTPS certificate is not one this computer trusts. A network that \
             intercepts HTTPS does this: sign in to the network (a hotel or guest network's \
             page), or set HTTPS_PROXY to the proxy that network requires",
        )
    } else if timed_out {
        Some("the server did not answer in time: check the network connection, or try again later")
    } else if no_way {
        Some("this computer cannot reach the server: check its network connection and proxy")
    } else {
        None
    };
    match hint {
        Some(hint) => format!("{text} ({hint})"),
        None => text,
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use reqwest::blocking::Client;
    use std::{
        io::{Read, Write},
        net::TcpListener,
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
    };

    /// Serve `replies` in order, one connection each, then close; returns
    /// the URL and how many requests arrived.
    pub(crate) fn serve(replies: Vec<String>) -> (String, Arc<AtomicUsize>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap();
        let served = Arc::new(AtomicUsize::new(0));
        let count = served.clone();
        std::thread::spawn(move || {
            for reply in replies {
                let Ok((mut stream, _)) = listener.accept() else {
                    return;
                };
                stream
                    .set_read_timeout(Some(Duration::from_secs(2)))
                    .unwrap();
                let mut request = Vec::new();
                let mut buffer = [0; 4096];
                while !request.windows(4).any(|w| w == b"\r\n\r\n") {
                    match stream.read(&mut buffer) {
                        Ok(0) | Err(_) => break,
                        Ok(n) => request.extend_from_slice(&buffer[..n]),
                    }
                }
                count.fetch_add(1, Ordering::SeqCst);
                let _ = stream.write_all(reply.as_bytes());
            }
        });
        (format!("http://{address}/file"), served)
    }

    fn reply(status: &str, body: &str) -> String {
        format!(
            "HTTP/1.1 {status}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
            body.len()
        )
    }

    #[test]
    fn server_errors_and_rate_limits_are_asked_again() {
        let (url, served) = serve(vec![
            reply("503 Service Unavailable", ""),
            reply("429 Too Many Requests", ""),
            reply("200 OK", "done"),
        ]);
        let client = Client::new();
        let response = send("the test request", || client.get(&url)).unwrap();
        assert_eq!(response.text().unwrap(), "done");
        assert_eq!(served.load(Ordering::SeqCst), 3);
    }

    #[test]
    fn a_refusal_is_returned_at_once_and_lasting_errors_stop() {
        let (url, served) = serve(vec![reply("404 Not Found", "")]);
        let client = Client::new();
        let response = send("the test request", || client.get(&url)).unwrap();
        assert_eq!(response.status(), StatusCode::NOT_FOUND);
        assert_eq!(served.load(Ordering::SeqCst), 1);
        // Every attempt answered 500: the last answer is returned.
        let (url, served) = serve(vec![reply("500 Internal Server Error", ""); 5]);
        let response = send("the test request", || client.get(&url)).unwrap();
        assert_eq!(response.status(), StatusCode::INTERNAL_SERVER_ERROR);
        assert_eq!(served.load(Ordering::SeqCst), ATTEMPTS as usize);
    }

    #[test]
    fn a_dropped_connection_is_asked_again_and_a_dead_server_named() {
        // The first connection closes without an answer.
        let (url, served) = serve(vec![String::new(), reply("200 OK", "done")]);
        let client = Client::new();
        let response = send("the test request", || client.get(&url)).unwrap();
        assert_eq!(response.text().unwrap(), "done");
        assert_eq!(served.load(Ordering::SeqCst), 2);
        // Nothing listens: every attempt fails, and the error says so.
        let port = TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let error = send("the test request", || {
            client.get(format!("http://127.0.0.1:{port}/file"))
        })
        .unwrap_err()
        .to_string();
        assert!(
            error.starts_with("the test request failed after 5 attempts"),
            "{error}"
        );
        assert!(error.contains("cannot reach the server"), "{error}");
    }

    #[test]
    fn a_certificate_outside_its_dates_names_the_clock_not_the_network() {
        /// The TLS library's text, below the HTTP client's.
        #[derive(Debug)]
        struct Cause(String);
        impl std::fmt::Display for Cause {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(&self.0)
            }
        }
        impl Error for Cause {}
        #[derive(Debug)]
        struct Request(Cause);
        impl std::fmt::Display for Request {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str("error sending request")
            }
        }
        impl Error for Request {
            fn source(&self) -> Option<&(dyn Error + 'static)> {
                Some(&self.0)
            }
        }
        for text in [
            // rustls, with and without the dates.
            "invalid peer certificate: certificate expired: verification time 2582000000 \
             (UNIX), but certificate is not valid after 1790000000 (792000000 seconds ago)",
            "invalid peer certificate: certificate not valid yet: verification time 946684800 \
             (UNIX), but certificate is not valid before 1760000000 (813315200 seconds in future)",
            "invalid peer certificate: Expired",
            "invalid peer certificate: NotValidYet",
            // OpenSSL, through native-tls.
            "error:0A000086:SSL routines:tls_post_process_server_certificate:certificate verify \
             failed:ssl/statem/statem_clnt.c:1889: (certificate has expired)",
            "certificate verify failed: (certificate is not yet valid)",
            // SChannel.
            "A required certificate is not within its validity period when verifying against \
             the current system clock or the timestamp in the signed file. (os error -2146762495)",
        ] {
            let explained = explain_causes(&Request(Cause(text.into())), false);
            assert!(
                explained.contains("check that its clock is right"),
                "{explained}"
            );
            assert!(!explained.contains("intercepts HTTPS"), "{explained}");
        }
        // A certificate this computer does not trust is still the network's.
        let explained = explain_causes(
            &Request(Cause("invalid peer certificate: UnknownIssuer".into())),
            false,
        );
        assert!(explained.contains("intercepts HTTPS"), "{explained}");
    }

    #[test]
    fn an_unreachable_server_is_named_whatever_the_system_calls_it() {
        /// A cause above the system's own, as the HTTP client reports one.
        #[derive(Debug)]
        struct Connect(io::Error);
        impl std::fmt::Display for Connect {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str("tcp connect error")
            }
        }
        impl Error for Connect {
            fn source(&self) -> Option<&(dyn Error + 'static)> {
                Some(&self.0)
            }
        }
        // The text a Windows set to German gives a refused connection.
        let words = "Der Zielcomputer hat die Verbindung verweigert";
        for kind in [
            io::ErrorKind::ConnectionRefused,
            io::ErrorKind::NetworkUnreachable,
            io::ErrorKind::HostUnreachable,
        ] {
            let text = explain_causes(&Connect(io::Error::new(kind, words)), false);
            assert!(
                text.starts_with(&format!("tcp connect error: {words} (")),
                "{text}"
            );
            assert!(text.contains("cannot reach the server"), "{text}");
        }
        // Another failure of the connection gets no such hint.
        let reset = Connect(io::Error::new(io::ErrorKind::ConnectionReset, words));
        assert_eq!(
            explain_causes(&reset, false),
            format!("tcp connect error: {words}")
        );
    }
}
