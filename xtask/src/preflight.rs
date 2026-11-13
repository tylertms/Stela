//! What `cargo xtask dist` and `check` need of this computer, checked before
//! either starts, so one message lists everything to install or set.
use crate::tools;
use anyhow::{Result, bail};

/// Every problem at once, or nothing.
pub fn run() -> Result<()> {
    let mut problems = Vec::new();
    if let Err(error) = crate::clang::toolset() {
        problems.extend(error.to_string().lines().map(str::to_owned));
    }
    if let Err(error) = crate::clang::target_toolset() {
        let text = error.to_string();
        for line in text.lines() {
            if !problems.iter().any(|problem| problem == line) {
                problems.push(line.to_owned());
            }
        }
    }
    if tools::which("git").is_none() {
        problems.push("install git, which checks out the pinned sources".into());
    }
    if let Some(problem) = crate::gpu::toolchain::python_problem() {
        problems.push(problem);
    }
    if tools::toolchain_llvm_tool("llvm-objcopy").is_none()
        && tools::which("llvm-objcopy").is_none()
    {
        problems.push(
            "install the Rust llvm-tools component (rustup component add llvm-tools), whose \
             llvm-objcopy prepares the boot image"
                .into(),
        );
    }
    report(&problems)
}

fn report(problems: &[String]) -> Result<()> {
    if problems.is_empty() {
        return Ok(());
    }
    bail!(
        "this computer cannot build Stela yet:\n  - {}",
        problems.join("\n  - ")
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_problem_is_listed_once() {
        assert!(report(&[]).is_ok());
        let text = report(&["install git".into(), "install libclang 21".into()])
            .unwrap_err()
            .to_string();
        assert_eq!(
            text,
            "this computer cannot build Stela yet:\n  - install git\n  - install libclang 21"
        );
    }
}
