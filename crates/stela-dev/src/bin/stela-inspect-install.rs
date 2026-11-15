//! Read-only inspection of a guest disk inside an explicitly bounded host region.
//! No write handle, repair, mount, protocol resume, or NVRAM mutation exists here.
use anyhow::{Context, Result};
use clap::Parser;
use serde_json::json;
use std::{fs::File, path::PathBuf};
use stela_formats::{
    apfs::{self, Container, FileKind},
    payload::ReadAt,
};

#[derive(Parser)]
struct Args {
    image: PathBuf,
    #[arg(long)]
    offset: u64,
    #[arg(long)]
    bytes: u64,
}
#[path = "../install_region.rs"]
mod install_region;
use install_region::Region;
impl ReadAt for Region {
    type Error = std::io::Error;
    fn len(&self) -> u64 {
        self.len()
    }
    fn read_at(&mut self, at: u64, out: &mut [u8]) -> std::io::Result<()> {
        self.read(at, out)
    }
}
fn main() -> Result<()> {
    let args = Args::parse();
    let mut source = Region::new(
        File::open(&args.image).context("open read-only source")?,
        args.offset,
        args.bytes,
    )?;
    let offsets = apfs::containers(&mut source)?;
    println!(
        "{}",
        json!({"read_only":true,"container_count":offsets.len(),"region_bytes":args.bytes})
    );
    for at in offsets {
        let source = Region::new(File::open(&args.image)?, args.offset, args.bytes)?;
        let mut container = Container::open(source, at)?;
        println!(
            "{}",
            json!({"container_offset":at,"xid":container.superblock.xid})
        );
        for volume in container.volumes()? {
            println!(
                "{}",
                json!({"volume":volume.name,"role":volume.role,"encrypted":volume.encrypted,"sealed":volume.sealed,"uuid":apfs::uuid_text(&volume.uuid)})
            );
            if volume.encrypted {
                println!(
                    "{}",
                    json!({"volume":volume.name,"inspection":"skipped_encrypted_volume"})
                );
                continue;
            }
            // Only installer/system metadata. No recursive user-data traversal.
            let mut pending = vec![(String::from("/"), 0usize)];
            let mut visited = 0;
            while let Some((path, depth)) = pending.pop() {
                if visited >= 600 {
                    println!(
                        "{}",
                        json!({"volume":volume.name,"inspection":"directory_budget_reached"})
                    );
                    break;
                }
                visited += 1;
                let inode = match volume.lookup(&mut container.device, &path, true) {
                    Ok(i) => i,
                    Err(e) => {
                        println!(
                            "{}",
                            json!({"volume":volume.name,"path":path,"error":e.to_string()})
                        );
                        continue;
                    }
                };
                let entries = match volume.entries(&mut container.device, inode.oid) {
                    Ok(e) => e,
                    Err(e) => {
                        println!(
                            "{}",
                            json!({"volume":volume.name,"path":path,"error":e.to_string()})
                        );
                        continue;
                    }
                };
                for entry in entries {
                    let child = format!("{}/{}", path.trim_end_matches('/'), entry.name);
                    let inode = match volume.inode(&mut container.device, entry.oid) {
                        Ok(i) => i,
                        Err(_) => continue,
                    };
                    println!(
                        "{}",
                        json!({"volume":volume.name,"path":child,"kind":format!("{:?}",entry.kind),"bytes":inode.size})
                    );
                    let metadata_volume = volume.role == apfs::volume::APFS_VOL_ROLE_PREBOOT
                        || volume.role == apfs::volume::APFS_VOL_ROLE_RECOVERY
                        || volume.role == apfs::volume::APFS_VOL_ROLE_UPDATE;
                    let selected_system = matches!(
                        child.as_str(),
                        "/System"
                            | "/System/Library"
                            | "/System/Library/CoreServices"
                            | "/System/Library/Caches"
                            | "/System/Library/Caches/com.apple.kernelcaches"
                    );
                    if entry.kind == FileKind::Directory
                        && depth < 6
                        && (metadata_volume || selected_system)
                    {
                        pending.push((child, depth + 1));
                    }
                }
            }
        }
        match stela_loader::installed::select(&mut container, &Default::default()) {
            Ok((_, set, evidence)) => println!(
                "{}",
                json!({"boot_candidate":set.directory(),"evidence":evidence})
            ),
            Err(e) => println!("{}", json!({"boot_candidate_error":e.to_string()})),
        }
    }
    Ok(())
}
