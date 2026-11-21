# Lab changes

This file tracks what the `lab` branch carries on top of `upstream/main` (tylertms/Stela), grouped into the pull requests (PRs) it should become.

Keep this file out of upstream PRs. Cherry-pick the listed commits, not the commit that adds this file. To list what `lab` still adds:

```fish
git log --no-merges upstream/main..lab
```

## 🔀 Merged in from others

| Source | Commits | Status |
|---|---|---|
| [tylertms/Stela#65](https://github.com/tylertms/Stela/pull/65): Proxmox VM support | `467837433` `ac134fb77` `52812b6a0` `f21ed79bf` `52f6adf37` | Open upstream |

## 📤 Mine, to submit

### 💿 Unattended install

Commit `5aa83e074`. Not submitted. Untested on hardware or a virtual machine (VM).

- If the USB stick holds `\EFI\stela\auto-install` and the machine can only be set up, the boot menu starts Install macOS once, without a key press (`crates/stela-boot/src/choice.rs`).
- `installer::choose(auto)` takes the one blank disk whole by replaying a user's key presses. With zero or several blank disks, or a refused plan, it logs `INSTALLER auto refused: <why>` and shows the picker (`crates/stela-boot/src/installer.rs`).
- The bundle repair's last optional step looks for a raw tar disk whose first member is `stela-seed`. It unpacks it onto `/System/Volumes/Data`, keeping owners and modes, and logs `stela-seed: <disk|none>` (`crates/stela-boot-core/src/bundle_repair.rs`).

Before submitting:

- [ ] Add a way to create the `auto-install` file and the seed disk, through the command-line tool or documented steps.
- [ ] Document both in `README.md` or `PROXMOX.md`.
- [ ] Run it end to end in the Proxmox VM.
