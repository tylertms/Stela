# H610I 非官方修改版 — 改动清单（v22 分支）

> **这是非官方修改版，与上游作者 [tylertms](https://github.com/tylertms) 无关。**
>
> 本分支 `h610i-v22` 是 `main`（上游 v0.4.1）之上**一个提交**，内容为
> 「上游 v0.4.1 完整源码 + 针对一台 JGINYUE H610I-GAMING / i5-12400F /
> RTX 4060 (AD107) / Kingston SNV3S1000G / 32 GB 测试机的本地修改」。
>
> ✅ **已编译并写入 U 盘**（2026-11-15 09:35，`cargo xtask boot build --release`，用时 7m53s）：
> `stela-deploy.efi` 49,181,184 字节，sha256
> `6536817f1a9b0304fa2d3926b17a79cd7252de3f0f2dcb5d081b5c13528f7f85`。
> ⚠️ **但尚未实机启动验证**，也没跑过测试全量。

## 许可与合规

| 组件 | 许可证 |
| --- | --- |
| EFI（`stela.efi` / `stela-deploy.efi`） | **GPL-2.0-only** |
| 命令行工具（`stela` / `stela.exe`） | **GPL-3.0-or-later** |

上游的 `LICENSE`、`NOTICE`、`LICENSES/` 与第三方许可材料**原样保留，未做任何改动**。
本分支只新增了源码修改与本文档。若分发编译产物，必须同时提供对应版本的完整源码、
依赖版本与构建脚本。

## v22 做了什么

- 基线：上游 **v0.4.1**（commit `984088526f5f6e5dd551ed717ee84c005b6f2774`，
  树 `fc7ce24b25eb9c40b079e51e33a1f56a98a280f2`）。
- 本地层来源：v21 分支（`h610i-v21`，基于上游 `25f661c`）的全部本地改动的**残留部分**。
- 相对 v0.4.1 的差异：**57 个文件（10 新增、
  47 修改，+3030/−90 行）**。
  两个子模块指针（`third_party/linux`、`third_party/mesa`）**与上游 v0.4.1 相同**。
- 合并方式：以 `25f661c` 为 base、v21 为 ours、v0.4.1 为 theirs 做 git 原生三方合并；
  25 个冲突文件共 43 个冲突块逐块判定（9 处手工拼接、2 个文件取上游全文后插回本地行）。

一个值得说明的发现：v21 与 v0.4.1 在 **9 个文件上逐字节相同**
（`stela-boot-core/src/ioapic.rs`、`ioapic/tests.rs`、`madt.rs`、`page_window.rs`、
`native_irq/madt.rs`、`driver-base` 的 `amd/.../capability.rs`、`pci/aspm/native.rs`、
`pci/.../vmd/endpoint/capabilities.rs`、`linux/compat/kernel/pci_host.c`），
说明 v21 在驱动/引导层一直跟着上游走。真正没跟上的只有 CLI/setup 一侧
（上游把 `repair`/`update` 的命令结构整个重排了）。

## ⚠️ CLI 用法变了

| v21 | v22（跟上游 v0.4.1） |
| --- | --- |
| `stela setup update stela` | `stela update --efi stela-deploy.efi` |
| `stela setup` | `stela create <盘> --confirm-disk` |

本地在 `repair` 上保留了一个 `--installer offline|online` 开关
（切换已有 U 盘的离线/在线安装器而不格式化），这是 v21 就有的本地功能。

## 编译阶段修掉的两个问题

源码合并本身只有一处语义错误，但它说明自动合并不能盲信：

1. `crates/stela-driver-net/src/realtek/ring.rs`：git 把「本地算 `good` 再计数」与
   「上游抽出 `received_length()`」拼在了一起，上游删掉了 `good` 绑定、本地那段
   `if good Ellipsis` 却留了下来 → `E0425: cannot find value good`。
   改为保留上游的 `received_length(status)`、计数用 `frame.is_some()`，
   本地 `packets`/`discarded` 计数器与其测试完整保留。
2. `third_party/linux`、`third_party/mesa` 指针：v21 记录的 `f00d83de…` / `a6e21227…`
   是 xtask 应用补丁系列时产生的**本地提交**（committer `stela@localhost`），
   **任何 clone 都 fetch 不到**（对 v21 跑 `git submodule update --init` 会失败）。
   v0.4.1 的 `ensure_upstream_pin` 会直接拒绝构建。已改回上游 pin
   `cc9de5d4…` / `ddaf85e3…`。

## 关键本地修复（与 v21 逐字节相同）

这些是本次合并最不能动的部分，全部 100% 保留：

- `crates/stela-driver-gpu/src/metal/compiler.rs` — `AirCompiler::llvm` 遇到已有延后任务时
  保持 Pending 重试，不再持服务锁等待 LLVM（设置界面冻结修复之一）
- `crates/stela-driver-gpu/src/metal/service/shared.rs` — `SharedService::run_job`
  先发布完成状态再释放编译锁，避免旧工作者清掉新工作者的运行标记
- `crates/stela-boot-core/src/stall_watch.rs` — `BOOT STALL` 分阶段后台诊断
- `crates/stela-runtime/src/restore/progress.rs` — 逐文件传输进度计量
- `crates/stela-driver-storage/src/nvme/takeover.rs` — NVMe 控制器接管
- `crates/stela-drivers/src/probe/nvme_path.rs` — NVMe 路径探测
- `crates/stela-driver-net/src/realtek/recovery.rs` — Realtek 有界接收恢复
- `crates/stela-runtime/src/platform/net/stack/dns_tcp.rs` — DNS over TCP 回退
- `crates/stela-boot-core/src/root_transition.rs`
- `crates/stela-dev/src/bin/stela-inspect-install.rs` — 只读安装状态检查

## 被上游更优实现取代、因此没有移植的本地改动

这些是**有意取舍**，不是漏合并：

1. `pci/capability.rs`：本地针对 `8086:7ae0`（Alder Lake-S xHCI）零长度 vendor 能力项的
   特例，被上游通用规则 `0..3 => (2, Header)` 取代（上游注释说明其跟随 Linux 语义）。
2. `pci/capability/tests.rs`：上述特例的测试随之移除（保留字节数语义已变）。
3. `metal/service/commands/render.rs`：本地 `PASS REFUSED v20 target=…` 文本被上游
   `Fault::from(error)` 分类取代。
4. `metal/device/tests.rs`：本地 `window_server_boot_pass_formats_begin_and_submit`
   被上游更完整的 `compositor_passes_with_readable_memoryless_attachments_begin` 取代。
5. `metal/kms.rs`：本地三次直调 `buffer()` 被上游「出错即释放已分配 buffer」取代。
6. `pci/requester.rs` / `requester/tests.rs`：本地 `MSI_EXTENDED_DATA_ENABLE` 特判
   被上游重写覆盖。
7. `restore/asr.rs`：本地写法与上游 `matches!` 守卫语义等价，取上游写法。

## 改动文件清单

每个 crate 下的条目为「相对上游 v0.4.1」的差异；★ 表示该文件与 v21 分支逐字节相同。

### `crates/stela-boot`

- `crates/stela-boot/src/engine.rs` — 修改 (+3/−0)
- `crates/stela-boot/src/engine/display.rs` — 修改 (+14/−0) ★
- `crates/stela-boot/src/engine/gpu.rs` — 修改 (+33/−2)
- `crates/stela-boot/src/engine/host/local/context.rs` — 修改 (+24/−1) ★
- `crates/stela-boot/src/engine/kernel_work.rs` — 修改 (+12/−5) ★
- `crates/stela-boot/src/engine/network.rs` — 修改 (+31/−0)
- `crates/stela-boot/src/engine/remote.rs` — 修改 (+16/−0) ★
- `crates/stela-boot/src/engine/restore.rs` — 修改 (+119/−15)
- `crates/stela-boot/src/engine/run.rs` — 修改 (+47/−0)
- `crates/stela-boot/src/engine/service.rs` — 修改 (+4/−4)
- `crates/stela-boot/src/engine/start.rs` — 修改 (+40/−5)
- `crates/stela-boot/src/engine/storage.rs` — 修改 (+23/−0)
- `crates/stela-boot/src/native_irq.rs` — 修改 (+22/−6)
- `crates/stela-boot/src/native_mmio/shared.rs` — 修改 (+39/−2)
- `crates/stela-boot/src/native_smp/executor.rs` — 修改 (+10/−0) ★
- `crates/stela-boot/src/output.rs` — 修改 (+5/−0)
- `crates/stela-boot/src/status.rs` — 修改 (+34/−5)
- `crates/stela-boot/src/system.rs` — 修改 (+26/−1)

### `crates/stela-boot-core`

- `crates/stela-boot-core/src/lib.rs` — 修改 (+1/−0)
- `crates/stela-boot-core/src/root_transition.rs` — 修改 (+39/−0) ★
- `crates/stela-boot-core/src/stall_watch.rs` — 新增 (+55/−0) ★

### `crates/stela-cli`

- `crates/stela-cli/src/boot_log.rs` — 修改 (+7/−7)
- `crates/stela-cli/src/lib.rs` — 修改 (+1/−0)
- `crates/stela-cli/src/setup/command.rs` — 修改 (+5/−0)
- `crates/stela-cli/src/setup/tests.rs` — 修改 (+4/−2)

### `crates/stela-dev`

- `crates/stela-dev/src/bin/stela-inspect-install.rs` — 新增 (+134/−0) ★
- `crates/stela-dev/src/install_region.rs` — 新增 (+136/−0) ★

### `crates/stela-driver-base`

- `crates/stela-driver-base/src/bus/pci/requester/roles.rs` — 修改 (+93/−1)

### `crates/stela-driver-gpu`

- `crates/stela-driver-gpu/src/metal/compiler.rs` — 修改 (+9/−5) ★
- `crates/stela-driver-gpu/src/metal/compiler/cache_tests.rs` — 修改 (+20/−0) ★
- `crates/stela-driver-gpu/src/metal/service/shared.rs` — 修改 (+82/−1) ★

### `crates/stela-driver-net`

- `crates/stela-driver-net/src/realtek.rs` — 修改 (+76/−2)
- `crates/stela-driver-net/src/realtek/recovery.rs` — 新增 (+73/−0) ★
- `crates/stela-driver-net/src/realtek/ring.rs` — 修改 (+20/−0)
- `crates/stela-driver-net/src/realtek/tests.rs` — 修改 (+2/−0)
- `crates/stela-driver-net/src/realtek/tests/packet_io.rs` — 修改 (+32/−0)
- `crates/stela-driver-net/src/realtek/tests/receive_recovery.rs` — 新增 (+108/−0) ★

### `crates/stela-driver-storage`

- `crates/stela-driver-storage/src/nvme.rs` — 修改 (+27/−4)
- `crates/stela-driver-storage/src/nvme/admin.rs` — 修改 (+104/−6)
- `crates/stela-driver-storage/src/nvme/features.rs` — 修改 (+1/−1)
- `crates/stela-driver-storage/src/nvme/owned.rs` — 修改 (+1/−0)
- `crates/stela-driver-storage/src/nvme/recovery.rs` — 修改 (+43/−1)
- `crates/stela-driver-storage/src/nvme/takeover.rs` — 新增 (+153/−0) ★
- `crates/stela-driver-storage/src/nvme/tests.rs` — 修改 (+218/−0)

### `crates/stela-drivers`

- `crates/stela-drivers/src/probe.rs` — 修改 (+13/−1)
- `crates/stela-drivers/src/probe/nvme_path.rs` — 新增 (+224/−0) ★

### `crates/stela-runtime`

- `crates/stela-runtime/src/platform/net/stack.rs` — 修改 (+31/−11)
- `crates/stela-runtime/src/platform/net/stack/dns_tcp.rs` — 新增 (+239/−0) ★
- `crates/stela-runtime/src/platform/net/stack/tests.rs` — 修改 (+88/−1)
- `crates/stela-runtime/src/platform/net/stack/tests/dns_fallback.rs` — 新增 (+179/−0) ★
- `crates/stela-runtime/src/restore.rs` — 修改 (+1/−0) ★
- `crates/stela-runtime/src/restore/asr.rs` — 修改 (+63/−0)
- `crates/stela-runtime/src/restore/driver.rs` — 修改 (+13/−1)
- `crates/stela-runtime/src/restore/driver/tests.rs` — 修改 (+9/−0)
- `crates/stela-runtime/src/restore/progress.rs` — 新增 (+75/−0) ★
- `crates/stela-runtime/src/restore/session.rs` — 修改 (+49/−0) ★
- `crates/stela-runtime/src/restore/session/tests.rs` — 修改 (+100/−0) ★

## 状态与提醒

- **已编译、未实机启动。** `cargo check --workspace --all-targets` 0 error；
  EFI 构建成功（sha256 见上）。GPU/NVMe/Realtek 测试与实机启动**尚未做**。
- 建议先在独立 U 盘试，**不要直接覆盖能进系统的 v21 U 盘**。
- v21 仍完整保留在 `h610i-v21` 分支，可随时对照或回退。
- 早期 21 版的逐版改动记录不再重复列在这里；本文件只说明 v22 相对上游的差异。
