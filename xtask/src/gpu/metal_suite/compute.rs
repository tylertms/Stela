//! corpus/compute.metal: dispatch builtins, threadgroup memory, atomics,
//! SIMD-group and quad-group functions, SIMD-group matrices, memory
//! layouts and control flow, each asserted exactly. SIMD-group results
//! assume 32-lane SIMD groups formed from consecutive thread indices, which
//! Apple's GPUs and NVIDIA's warps both have; the builtins case records the
//! width. The order in which atomics return their values is the device's
//! choice, so those outputs are compared sorted.
use super::plan::{Buffer, Case, Output, Plan, Step, Texture, Tolerance};
use super::values::{Dtype, Random, bits32, f16_bits, f16r, f32r, f32s, floats, ints, sum, u32s};
use serde_json::json;

const SIMD: usize = 32;
const SOURCE: &str = "compute.metal";

fn values(seed: u64, count: usize, limit: u64) -> Vec<u64> {
    let mut rng = Random::new(seed);
    (0..count).map(|_| u64::from(rng.u32()) % limit).collect()
}

fn u32w(v: i128) -> u32 {
    v as u32
}

fn words(v: &[u64]) -> Vec<u8> {
    v.iter().flat_map(|&x| (x as u32).to_le_bytes()).collect()
}

fn builtins(plan: &mut Plan, library: &str) {
    let (grid, group) = ([13usize, 7, 3], [4usize, 4, 2]);
    let mut expected = vec![];
    for z in 0..grid[2] {
        for y in 0..grid[1] {
            for x in 0..grid[0] {
                let pos = [x, y, z];
                let gpos: Vec<usize> = (0..3).map(|i| pos[i] / group[i]).collect();
                let lpos: Vec<usize> = (0..3).map(|i| pos[i] % group[i]).collect();
                let actual: Vec<usize> = (0..3)
                    .map(|i| group[i].min(grid[i] - gpos[i] * group[i]))
                    .collect();
                let groups: Vec<usize> = (0..3).map(|i| grid[i].div_ceil(group[i])).collect();
                let local = (lpos[2] * actual[1] + lpos[1]) * actual[0] + lpos[0];
                let count = actual[0] * actual[1] * actual[2];
                expected.extend(pos);
                expected.extend(&gpos);
                expected.extend(&lpos);
                expected.extend([
                    actual[0] * 10000 + actual[1] * 100 + actual[2],
                    groups[0] * 10000 + groups[1] * 100 + groups[2],
                    grid[0] * 10000 + grid[1] * 100 + grid[2],
                    local,
                    local,
                    SIMD,
                    count.div_ceil(SIMD),
                ]);
            }
        }
    }
    let total = grid.iter().product::<usize>();
    let expected: Vec<u32> = expected.into_iter().map(|v| v as u32).collect();
    plan.add(
        Case::compute(
            "compute.builtins",
            library,
            "builtins",
            [13, 7, 3],
            [4, 4, 2],
        )
        .dispatch_threads()
        .buffer(Buffer::zeroed("out", 0, total * 64))
        .output(Output::exact("out", Dtype::U32, u32s(&expected)))
        .note("non-uniform threadgroups at the grid edges"),
    );
}

fn reduction(plan: &mut Plan, library: &str) {
    let (size, groups) = (256, 8);
    let data = values(11, size * groups, 1 << 20);
    let mut sums = vec![];
    let mut partials = vec![];
    for g in 0..groups {
        let mut scratch: Vec<u64> = data[g * size..(g + 1) * size].to_vec();
        let mut stride = size / 2;
        while stride > 0 {
            for i in 0..stride {
                scratch[i] = (scratch[i] + scratch[i + stride]) & 0xFFFF_FFFF;
            }
            stride /= 2;
        }
        sums.push(scratch[0]);
        partials.extend(scratch);
    }
    plan.add(
        Case::compute(
            "compute.threadgroup.reduce",
            library,
            "reduce_threadgroup",
            [(size * groups) as u32, 1, 1],
            [size as u32, 1, 1],
        )
        .buffer(Buffer::data("in", 0, words(&data)))
        .buffer(Buffer::zeroed("sums", 1, groups * 4))
        .buffer(Buffer::zeroed("partials", 2, data.len() * 4))
        .output(Output::exact("sums", Dtype::U32, words(&sums)))
        .output(Output::exact("partials", Dtype::U32, words(&partials)))
        .set(
            "threadgroup_memory",
            json!([{"index": 0, "length": size * 4}]),
        ),
    );
    let mut rng = Random::new(12);
    let image: Vec<f64> = (0..64 * 64 * 4)
        .map(|_| f32r(rng.uniform(-8.0, 8.0)))
        .collect();
    let mut out = vec![];
    for y in 0..64 {
        for x in 0..64 {
            let (bx, by, lx, ly) = (x / 16 * 16, y / 16 * 16, x % 16, y % 16);
            let src = ((by + lx) * 64 + bx + ly) * 4;
            let extra = [lx as f64, ly as f64, 0.0, 1.0];
            out.extend((0..4).map(|c| f32r(image[src + c] * 2.0 + extra[c])));
        }
    }
    plan.add(
        Case::compute(
            "compute.threadgroup.transpose",
            library,
            "threadgroup_transpose",
            [64, 64, 1],
            [16, 16, 1],
        )
        .buffer(Buffer::data("in", 0, floats(Dtype::F32, &image)))
        .buffer(Buffer::zeroed("out", 1, image.len() * 4))
        .output(Output::exact("out", Dtype::F32, floats(Dtype::F32, &out))),
    );
}

fn atomics(plan: &mut Plan, library: &str) {
    let n = 4096;
    let data = values(21, n, 1 << 32);
    let initial_u: Vec<u64> = [0, 0, 0xFFFF_FFFF, 0, 0, 0xFFFF_FFFF, 0, 0].repeat(8);
    let initial_s: Vec<i128> = [0, 0x7FFF_FFFF, -0x8000_0000, 0].repeat(8);
    let mut u = initial_u.clone();
    let mut s = initial_s.clone();
    for (i, &v) in data.iter().enumerate() {
        let k = i % 8;
        u[k * 8] = (u[k * 8] + v) & 0xFFFF_FFFF;
        u[k * 8 + 1] = u[k * 8 + 1].wrapping_sub(v) & 0xFFFF_FFFF;
        u[k * 8 + 2] &= v | 0x0F0F_0F0F;
        u[k * 8 + 3] |= v & 0x1010_1010;
        u[k * 8 + 4] ^= v;
        u[k * 8 + 5] = u[k * 8 + 5].min(v);
        u[k * 8 + 6] = u[k * 8 + 6].max(v);
        u[k * 8 + 7] = u[k * 8 + 7].max(v);
        let w = v as i128 - 0x8000_0000;
        s[k * 4] += w >> 8;
        s[k * 4 + 1] = s[k * 4 + 1].min(w);
        s[k * 4 + 2] = s[k * 4 + 2].max(w);
        s[k * 4 + 3] -= w >> 8;
    }
    let wrap32 = |v: i128| (v + (1 << 31)).rem_euclid(1 << 32) - (1 << 31);
    let s: Vec<i128> = s.into_iter().map(wrap32).collect();
    plan.add(
        Case::compute1(
            "compute.atomics.device",
            library,
            "atomics_device",
            n as u32,
        )
        .buffer(Buffer::data("in", 0, words(&data)))
        .buffer(Buffer::data("u", 1, words(&initial_u)))
        .buffer(Buffer::data("s", 2, ints(Dtype::I32, &initial_s)))
        .output(Output::exact("u", Dtype::U32, words(&u)))
        .output(Output::exact("s", Dtype::I32, ints(Dtype::I32, &s))),
    );
    let total: u32 = (0..n).map(|i| (i % 5) as u32 + 1).sum();
    plan.add(
        Case::compute1(
            "compute.atomics.returns",
            library,
            "atomics_returns",
            n as u32,
        )
        .buffer(Buffer::data("counter", 0, u32s(&[0])))
        .buffer(Buffer::zeroed("seen", 1, n * 4))
        .output(Output::exact("counter", Dtype::U32, u32s(&[total])).set("length", 4))
        .output(
            Output::exact("seen", Dtype::U32, u32s(&vec![1; n]))
                .set("sorted_compare", true)
                .set("derive", "fetch_add"),
        )
        .note("seen holds each fetch-add return in the order the threads ran"),
    );
    // Values published in acquire-release order: the last of the threads
    // to take a ticket sees every thread's value.
    let published = (0..n as u32).fold(0u32, |sum, id| sum.wrapping_add(id * 3 + 1));
    for (name, function, cells) in [
        (
            "compute.atomics.ordered",
            "atomics_ordered",
            [n as u32, published, n as u32],
        ),
        (
            "compute.atomics.fenced",
            "atomics_fenced",
            [n as u32, published, 0],
        ),
    ] {
        if plan.has(SOURCE, function) {
            plan.add(
                Case::compute1(name, library, function, n as u32)
                    .buffer(Buffer::data("cells", 0, u32s(&[0; 3])))
                    .buffer(Buffer::zeroed("values", 1, n * 4))
                    .output(Output::exact("cells", Dtype::U32, u32s(&cells))),
            );
        }
    }
    let all: Vec<u32> = (0..=n as u32).collect();
    plan.add(
        Case::compute1(
            "compute.atomics.exchange",
            library,
            "atomics_exchange",
            n as u32,
        )
        .buffer(Buffer::data("cell", 0, u32s(&[0])))
        .buffer(Buffer::zeroed("seen", 1, n * 4))
        .output(
            Output::exact("seen", Dtype::U32, u32s(&all))
                .set("sorted_compare", true)
                .set("append", "cell"),
        )
        .output(
            Output::new("cell", Dtype::U32, Tolerance::default())
                .set("length", 4)
                .set("compare", false),
        )
        .note("the returns and the final value are each value 0..n once"),
    );
    let (groups, size) = (16, 128);
    let data = values(22, groups * size, 2000);
    let mut hist = vec![];
    for g in 0..groups {
        let chunk = &data[g * size..(g + 1) * size];
        let mut bins = [0u64; 16];
        for &v in chunk {
            bins[(v % 16) as usize] += 1;
        }
        hist.extend(bins);
        hist.push(chunk.iter().min().unwrap().wrapping_sub(1000) & 0xFFFF_FFFF);
        hist.push(chunk.iter().max().unwrap().wrapping_sub(1000) & 0xFFFF_FFFF);
    }
    plan.add(
        Case::compute(
            "compute.atomics.threadgroup",
            library,
            "atomics_threadgroup",
            [(groups * size) as u32, 1, 1],
            [size as u32, 1, 1],
        )
        .buffer(Buffer::data("in", 0, words(&data)))
        .buffer(Buffer::zeroed("out", 1, groups * 18 * 4))
        .output(Output::exact("out", Dtype::U32, words(&hist))),
    );
    let fl: Vec<f64> = values(23, n, 1 << 20)
        .iter()
        .map(|&v| (v % 100) as f64)
        .collect();
    let sums: Vec<f64> = (0..4)
        .map(|k| sum(fl.iter().skip(k).step_by(4).copied()))
        .collect();
    if plan.has(SOURCE, "atomics_float") {
        plan.add(
            Case::compute1("compute.atomics.float", library, "atomics_float", n as u32)
                .buffer(Buffer::data("in", 0, floats(Dtype::F32, &fl)))
                .buffer(Buffer::data("sums", 1, floats(Dtype::F32, &[0.0; 4])))
                .output(Output::exact("sums", Dtype::F32, floats(Dtype::F32, &sums))),
        );
    }
}

/// The word `store_linear` stores at an index.
fn linear_word(i: u32) -> u32 {
    i.wrapping_mul(2_654_435_761).wrapping_add(0x9e37_79b9)
}

/// Plain device stores: their bandwidth into Shared and Private buffers
/// (gpu_ms of the two cases), and their visibility to the next dispatch
/// and, through a fence or a barrier, to the last threadgroup.
fn store_order(plan: &mut Plan, library: &str) {
    // 4 MiB stored, then read back by 4096 threads in a strided sum.
    let (words_n, threads) = (1u32 << 20, 4096u32);
    let sums: Vec<u32> = (0..threads)
        .map(|t| {
            (t..words_n)
                .step_by(threads as usize)
                .fold(0u32, |s, i| s.wrapping_add(linear_word(i)))
        })
        .collect();
    for mode in ["shared", "private"] {
        plan.add(
            Case::sequence(
                &format!("compute.store_bandwidth.{mode}"),
                library,
                vec![
                    Step::compute("store_linear", [words_n, 1, 1], [256, 1, 1])
                        .bind(&[("data", 0)]),
                    Step::compute("sum_strided", [threads, 1, 1], [256, 1, 1]).bind(&[
                        ("data", 0),
                        ("sums", 1),
                        ("words", 2),
                    ]),
                ],
            )
            .buffer(
                Buffer::zeroed("data", 0, words_n as usize * 4)
                    .unbound()
                    .set("storage", mode),
            )
            .buffer(Buffer::zeroed("sums", 0, threads as usize * 4).unbound())
            .buffer(Buffer::data("words", 0, u32s(&[words_n])).unbound())
            .output(Output::exact("sums", Dtype::U32, u32s(&sums)))
            .note("the store kernel's gpu_ms is the store cost into this storage mode"),
        );
    }
    let (groups, size) = (64usize, 256usize);
    let data = values(24, groups * size, 1 << 16);
    let total = data.iter().fold(0u64, |s, &v| (s + v) & 0xFFFF_FFFF);
    for (name, function) in [
        ("compute.last_group_reduction", "last_group_sum"),
        ("compute.last_group_reduction.plain", "last_group_sum_plain"),
    ] {
        let mut case = Case::compute(
            name,
            library,
            function,
            [(groups * size) as u32, 1, 1],
            [size as u32, 1, 1],
        )
        .buffer(Buffer::data("in", 0, words(&data)))
        .buffer(Buffer::zeroed("partials", 1, groups * 4))
        .buffer(Buffer::data("cells", 2, u32s(&[0, 0])))
        .output(Output::exact("cells", Dtype::U32, words(&[groups as u64, total])).set("length", 8))
        .set(
            "threadgroup_memory",
            json!([{"index": 0, "length": size * 4}]),
        );
        if function.ends_with("plain") {
            case = case
                .informational()
                .note("plain stores and a device barrier: Metal does not promise the last group sees them");
        }
        plan.add(case);
    }
}

fn simd(plan: &mut Plan, library: &str) {
    let n = 256;
    let data = values(31, n, 600);
    let mut out: Vec<u32> = vec![];
    for (i, &v) in data.iter().enumerate() {
        let base = i - i % SIMD;
        let g = &data[base..base + SIMD];
        let lane = i % SIMD;
        let s: Vec<i64> = g.iter().map(|&x| x as i64 - 50).collect();
        let f: Vec<f64> = g.iter().map(|&x| (x % 64) as f64).collect();
        let mut prod = 1u64;
        for &x in g {
            prod = (prod * (x % 3 + 1)) & 0xFFFF_FFFF;
        }
        let ex_prod: u64 = g[..lane].iter().map(|&x| x % 2 + 1).product();
        let sign_prod: i64 = s[..=lane]
            .iter()
            .map(|&x| if x.rem_euclid(2) == 0 { 1 } else { -1 })
            .product();
        let ballot: u64 = g
            .iter()
            .enumerate()
            .filter(|(_, x)| *x & 1 == 1)
            .map(|(k, _)| 1u64 << k)
            .sum();
        let flags = u32::from(g.iter().all(|&x| x < 1000))
            | u32::from(g.iter().any(|&x| x == g[0])) << 1
            | u32::from(lane == 0) << 2;
        let total: u64 = g.iter().sum();
        out.extend([
            total as u32,
            prod as u32,
            *g.iter().min().unwrap() as u32,
            *g.iter().max().unwrap() as u32,
        ]);
        let (mut and, mut or, mut xor) = (0xFFFF_FFFFu64, 0u64, 0u64);
        for &x in g {
            and &= x | 0xF0;
            or |= x & 0x30;
            xor ^= x;
        }
        let inclusive: u64 = g[..=lane].iter().sum();
        let exclusive: u64 = g[..lane].iter().sum();
        out.extend([
            and as u32,
            or as u32,
            xor as u32,
            inclusive as u32,
            exclusive as u32,
            ex_prod as u32,
        ]);
        out.extend([
            g[(lane * 7 + 3) % SIMD] as u32,
            if lane >= 3 { g[lane - 3] } else { v } as u32,
            if lane + 5 < SIMD { g[lane + 5] } else { v } as u32,
            g[lane ^ 9] as u32,
            g[17] as u32,
            g[0] as u32,
            ballot as u32,
            flags,
        ]);
        let fmax = f.iter().copied().fold(f[0], f64::max);
        out.extend([
            bits32(sum(f.iter().copied())),
            bits32(sum(f[..=lane].iter().copied())),
            bits32(fmax),
            u32w(i128::from(*s.iter().min().unwrap())),
            u32w(i128::from(s.iter().sum::<i64>())),
            u32w(i128::from(sign_prod)),
        ]);
    }
    plan.add(
        Case::compute1(
            "compute.simd.functions",
            library,
            "simd_functions",
            n as u32,
        )
        .buffer(Buffer::data("in", 0, words(&data)))
        .buffer(Buffer::zeroed("out", 1, n * 96))
        .output(Output::exact("out", Dtype::U32, u32s(&out))),
    );
    let mut rot = vec![];
    for (i, _) in data.iter().enumerate() {
        let base = i - i % SIMD;
        let g = &data[base..base + SIMD];
        let lane = i % SIMD;
        let up = if lane >= 4 {
            g[lane - 4]
        } else {
            g[SIMD - 4 + lane] + 1000
        };
        let down = if lane + 6 < SIMD {
            g[lane + 6]
        } else {
            g[lane + 6 - SIMD] + 1000
        };
        rot.extend([g[(lane + SIMD - 5) % SIMD], g[(lane + 11) % SIMD], up, down]);
    }
    if plan.has(SOURCE, "simd_rotate") {
        plan.add(
            Case::compute1("compute.simd.rotate", library, "simd_rotate", n as u32)
                .buffer(Buffer::data("in", 0, words(&data)))
                .buffer(Buffer::zeroed("out", 1, n * 16))
                .output(Output::exact("out", Dtype::U32, words(&rot))),
        );
    }
    let mut div = vec![];
    for i in 0..n {
        let base = i - i % SIMD;
        let lane = i % SIMD;
        let active: Vec<usize> = (0..SIMD).filter(|k| k % 3 == 0).collect();
        let g: Vec<u64> = active.iter().map(|&k| data[base + k]).collect();
        if !lane.is_multiple_of(3) {
            div.extend([0xDEAD_BEEFu64; 6]);
            continue;
        }
        let mask: u64 = active.iter().map(|&k| 1u64 << k).sum();
        let pos = active.iter().position(|&k| k == lane).unwrap();
        div.extend([
            g.iter().sum::<u64>() & 0xFFFF_FFFF,
            g[..pos].iter().sum::<u64>() & 0xFFFF_FFFF,
            mask,
            mask,
            g[0],
            *g.iter().max().unwrap(),
        ]);
    }
    plan.add(
        Case::compute1(
            "compute.simd.divergent",
            library,
            "simd_divergent",
            n as u32,
        )
        .buffer(Buffer::data("in", 0, words(&data)))
        .buffer(Buffer::zeroed("out", 1, n * 24))
        .output(Output::exact("out", Dtype::U32, words(&div))),
    );
    let mut quad = vec![];
    for i in 0..n {
        let base = i - i % 4;
        let q = &data[base..base + 4];
        let lane = i % 4;
        let ballot: u64 = q
            .iter()
            .enumerate()
            .filter(|(_, x)| *x & 1 == 1)
            .map(|(k, _)| 1u64 << k)
            .sum();
        quad.extend([
            q.iter().sum::<u64>(),
            *q.iter().min().unwrap(),
            *q.iter().max().unwrap(),
            q[..=lane].iter().sum::<u64>(),
            q[(lane + 1) % 4],
            q[lane ^ 2],
            q[3],
            ballot | (lane as u64) << 8,
        ]);
    }
    plan.add(
        Case::compute1(
            "compute.quad.functions",
            library,
            "quad_functions",
            n as u32,
        )
        .buffer(Buffer::data("in", 0, words(&data)))
        .buffer(Buffer::zeroed("out", 1, n * 32))
        .output(Output::exact("out", Dtype::U32, words(&quad))),
    );
    if plan.has(SOURCE, "matrix_filled") {
        plan.add(
            Case::compute(
                "compute.matrix.filled",
                library,
                "matrix_filled",
                [6, 1, 1],
                [6, 1, 1],
            )
            .buffer(Buffer::zeroed("out", 0, 24))
            .output(Output::exact("out", Dtype::F32, f32s(&[2.5; 6]))),
        );
    }
    if plan.has(SOURCE, "address_spaces") {
        plan.add(
            Case::compute(
                "compute.address_spaces",
                library,
                "address_spaces",
                [1, 1, 1],
                [1, 1, 1],
            )
            .buffer(Buffer::zeroed("out", 0, 16))
            .set("threadgroup_memory", json!([{"index": 0, "length": 16}]))
            .output(Output::exact("out", Dtype::U32, u32s(&[1, 2, 4, 0]))),
        );
    }
    if plan.has(SOURCE, "generic_pointers") {
        // Each thread's pointer is into its own device, threadgroup or thread
        // memory by id % 3; it adds 7 to the next word into its word.
        let mut out = vec![0u32; 64 + 64 * 4];
        for id in 0..64u32 {
            let words = 64 + id as usize * 4;
            for i in 0..4 {
                out[words + i] = id * 100 + i as u32;
            }
            let (next, word) = ((id + 1) & 3, (id & 3) as usize);
            let (value, space) = match id % 3 {
                0 => (id * 100 + next, 1),
                1 => (id * 10 + next, 2),
                _ => (id + next, 4),
            };
            out[id as usize] = value + 7 + space * 1000;
            if id % 3 == 0 {
                out[words + word] = value + 7;
            }
        }
        plan.add(
            Case::compute(
                "compute.generic_pointers",
                library,
                "generic_pointers",
                [64, 1, 1],
                [32, 1, 1],
            )
            .buffer(Buffer::zeroed("out", 0, out.len() * 4))
            .set("threadgroup_memory", json!([{"index": 0, "length": 512}]))
            .output(Output::exact("out", Dtype::U32, u32s(&out))),
        );
    }
    plan.add(
        Case::compute(
            "compute.array_ref",
            library,
            "array_refs",
            [1, 1, 1],
            [1, 1, 1],
        )
        .buffer(Buffer::data("in", 0, u32s(&[4, 5, 6, 7])))
        .buffer(Buffer::zeroed("out", 1, 32))
        .output(Output::exact(
            "out",
            Dtype::U32,
            u32s(&[3, 6, 15, 26, 11, 0, 0, 0]),
        )),
    );
    if plan.has(SOURCE, "simdgroup_elements") {
        // Small integers, so the sum is exact in any order.
        let m: Vec<f64> = (0..64).map(|k| f64::from(k % 13) - 6.0).collect();
        let mut expected: Vec<f64> = m.iter().map(|v| v * 2.0).collect();
        expected.push(m.iter().sum());
        plan.add(
            Case::compute(
                "compute.simdgroup_matrix.thread_elements",
                library,
                "simdgroup_elements",
                [32, 1, 1],
                [32, 1, 1],
            )
            .buffer(Buffer::data("in", 0, f32s(&m)))
            .buffer(Buffer::zeroed("out", 1, 65 * 4))
            .output(Output::exact("out", Dtype::F32, f32s(&expected)))
            .requires(&[], &["simd_reduction"]),
        );
    }
    if plan.has(SOURCE, "simdgroup_element_layout") {
        // Apple GPUs give lane l row 4 (l / 16) + (l / 2) % 4 and columns
        // 4 ((l / 8) % 2) + 2 (l % 2) and the next.
        let indices: Vec<f64> = (0..64).map(f64::from).collect();
        let expected: Vec<f64> = (0..32u32)
            .flat_map(|l| {
                let row = 4 * (l / 16) + (l / 2) % 4;
                let column = 4 * ((l / 8) % 2) + 2 * (l % 2);
                [row * 8 + column, row * 8 + column + 1].map(f64::from)
            })
            .collect();
        plan.add(
            Case::compute(
                "compute.simdgroup_matrix.element_layout",
                library,
                "simdgroup_element_layout",
                [32, 1, 1],
                [32, 1, 1],
            )
            .buffer(Buffer::data("in", 0, f32s(&indices)))
            .buffer(Buffer::zeroed("out", 1, 64 * 4))
            .output(Output::exact("out", Dtype::F32, f32s(&expected))),
        );
    }
    if plan.has(SOURCE, "simdgroup_matrix_bfloat") {
        simdgroup_matrix_bfloat(plan, library);
    }
    if plan.has(SOURCE, "simdgroup_matrix") {
        let mut rng = Random::new(33);
        let a: Vec<f64> = (0..64).map(|_| (rng.u32() % 7) as f64 - 3.0).collect();
        let b: Vec<f64> = (0..64).map(|_| (rng.u32() % 7) as f64 - 3.0).collect();
        let mut prod = vec![];
        let mut half_prod = vec![];
        for r in 0..8 {
            for c in 0..8 {
                let p = sum((0..8).map(|k| a[r * 8 + k] * b[k * 8 + c]));
                prod.push(p + a[c * 8 + r]);
                half_prod.push(f16r(p));
            }
        }
        plan.add(
            Case::compute(
                "compute.simdgroup_matrix",
                library,
                "simdgroup_matrix",
                [32, 1, 1],
                [32, 1, 1],
            )
            .buffer(Buffer::data("a", 0, floats(Dtype::F32, &a)))
            .buffer(Buffer::data("b", 1, floats(Dtype::F32, &b)))
            .buffer(Buffer::zeroed("out", 2, 256))
            .buffer(Buffer::zeroed("out_half", 3, 128))
            .buffer(Buffer::data("ah", 4, floats(Dtype::F16, &a)))
            .buffer(Buffer::data("bh", 5, floats(Dtype::F16, &b)))
            .output(Output::exact("out", Dtype::F32, floats(Dtype::F32, &prod)))
            .output(Output::exact(
                "out_half",
                Dtype::F16,
                floats(Dtype::F16, &half_prod),
            ))
            .requires(&[], &["simd_reduction"]),
        );
    }
}

/// An 8x8 bfloat SIMD-group matrix product of small integers, exact in
/// bfloat: A * B + A.
fn simdgroup_matrix_bfloat(plan: &mut Plan, library: &str) {
    let mut rng = Random::new(34);
    let a: Vec<f64> = (0..64).map(|_| (rng.u32() % 7) as f64 - 3.0).collect();
    let b: Vec<f64> = (0..64).map(|_| (rng.u32() % 7) as f64 - 3.0).collect();
    let bf = |v: &[f64]| -> Vec<u8> {
        v.iter()
            .flat_map(|&x| (((x as f32).to_bits() >> 16) as u16).to_le_bytes())
            .collect()
    };
    let prod: Vec<f64> = (0..64)
        .map(|i| {
            let (r, c) = (i / 8, i % 8);
            sum((0..8).map(|k| a[r * 8 + k] * b[k * 8 + c])) + a[i]
        })
        .collect();
    plan.add(
        Case::compute(
            "compute.simdgroup_matrix.bfloat",
            library,
            "simdgroup_matrix_bfloat",
            [32, 1, 1],
            [32, 1, 1],
        )
        .buffer(Buffer::data("a", 0, bf(&a)))
        .buffer(Buffer::data("b", 1, bf(&b)))
        .buffer(Buffer::zeroed("out", 2, 128))
        .output(Output::exact("out", Dtype::U16, bf(&prod)))
        .requires(&[], &["simd_reduction"]),
    );
}

/// struct Mixed: char, short, float3 at 16, packed_float3 at 32, half at
/// 44, uchar4 at 48, float2x2 at 56, 80 bytes with its alignment.
#[allow(clippy::too_many_arguments)]
fn mixed(c: i64, short: i64, v: &[f64], p: &[f64], h: f64, u: &[u64], m: &[f64]) -> Vec<u8> {
    let mut out = vec![0u8; 80];
    out[0] = c as u8;
    out[2..4].copy_from_slice(&(short as i16).to_le_bytes());
    for (k, &x) in v.iter().enumerate() {
        out[16 + 4 * k..20 + 4 * k].copy_from_slice(&(x as f32).to_le_bytes());
    }
    for (k, &x) in p.iter().enumerate() {
        out[32 + 4 * k..36 + 4 * k].copy_from_slice(&(x as f32).to_le_bytes());
    }
    out[44..46].copy_from_slice(&floats(Dtype::F16, &[h]));
    for (k, &x) in u.iter().enumerate() {
        out[48 + k] = x as u8;
    }
    for (k, &x) in m.iter().enumerate() {
        out[56 + 4 * k..60 + 4 * k].copy_from_slice(&(x as f32).to_le_bytes());
    }
    out
}

/// The bytes a copy of Mixed defines: its members without padding, nor
/// float3's fourth lane.
fn mixed_defined() -> Vec<u8> {
    (0..80)
        .map(|b| {
            u8::from(
                b == 0
                    || b == 2
                    || b == 3
                    || (16..28).contains(&b)
                    || (32..46).contains(&b)
                    || (48..52).contains(&b)
                    || (56..72).contains(&b),
            )
        })
        .collect()
}

fn memory(plan: &mut Plan, library: &str) {
    let n = 64;
    let mut rng = Random::new(41);
    let mut records = vec![];
    let mut expected = vec![];
    for i in 0..n as i64 {
        let c = rng.below(100) - 50;
        let short = rng.below(60000) - 30000;
        let v: Vec<f64> = (0..4).map(|_| f32r(rng.uniform(-4.0, 4.0))).collect();
        let p: Vec<f64> = (0..3).map(|_| f32r(rng.uniform(-4.0, 4.0))).collect();
        let h = f16r(rng.uniform(-4.0, 4.0));
        let u: Vec<u64> = (0..4).map(|_| u64::from(rng.u32() % 256)).collect();
        let m: Vec<f64> = (0..4).map(|_| f32r(rng.uniform(-4.0, 4.0))).collect();
        records.extend(mixed(c, short, &v, &p, h, &u, &m));
        let k = 1.5;
        let nv = [f32r(v[2] * k), f32r(v[0] * k), f32r(v[1] * k), v[3]];
        let nc = (c * 3 + 1 + 128).rem_euclid(256) - 128;
        let ns = (short - i + 32768).rem_euclid(65536) - 32768;
        let np: Vec<f64> = (0..3).map(|j| f32r(p[j] + nv[j])).collect();
        let nu: Vec<u64> = (0..4).map(|j| (u[3 - j] + i as u64) % 256).collect();
        let nm = [m[0], m[2], m[1], m[3]];
        expected.extend(mixed(nc, ns, &nv, &np, f16r(h * 2.0), &nu, &nm));
    }
    let data: Vec<u8> = (0..n * 3).map(|_| (rng.u32() % 256) as u8).collect();
    let shorts: Vec<i128> = (0..n)
        .map(|i| i128::from(data[i * 3]) | i128::from(data[i * 3 + 1]) << 8)
        .collect();
    let table = [1.5, -2.0, 3.25, 0.0, -7.0, 11.0, 0.125, f32r(1e10)];
    let scalars: Vec<f64> = (0..n)
        .map(|i| {
            let local: Vec<f64> = (0..16)
                .map(|j| f32r((j * i) as f64 + table[j % 8]))
                .collect();
            f32r(f32r(local[i % 16] + f32r(local[(i * 7) % 16] * 0.25)) + table[i % 8])
        })
        .collect();
    let one = Tolerance::ulp(1.0);
    plan.add(
        Case::compute(
            "compute.memory.layouts",
            library,
            "memory_layouts",
            [n as u32, 1, 1],
            [32, 1, 1],
        )
        .buffer(Buffer::data("in", 0, records))
        .buffer(Buffer::zeroed("out", 1, n * 80))
        .buffer(Buffer::data("bytes", 2, data))
        .buffer(Buffer::zeroed("shorts", 3, n * 2))
        .buffer(Buffer::data(
            "k",
            4,
            floats(Dtype::F32, &[1.5, 0.25, 0.0, 0.0]),
        ))
        .buffer(Buffer::zeroed("scalars", 5, n * 4))
        .output(Output::exact("out", Dtype::U8, expected).mask(mixed_defined()))
        .output(Output::exact(
            "shorts",
            Dtype::U16,
            ints(Dtype::U16, &shorts),
        ))
        .output(
            Output::new("scalars", Dtype::F32, one.clone())
                .expect(floats(Dtype::F32, &scalars), one),
        )
        .note("padding and the float3 pad lane are not compared"),
    );
}

fn collatz(mut n: u64) -> u64 {
    let mut steps = 0;
    while n != 1 && steps < 500 {
        n = if n & 1 == 1 { n * 3 + 1 } else { n / 2 };
        steps += 1;
    }
    steps
}

fn control(plan: &mut Plan, library: &str) {
    let n = 1024;
    let data = values(51, n, 100_000);
    let mut out = vec![];
    for &v in &data {
        if v % 17 == 0 {
            out.extend([0xFFFF_FFFF, 0, 0, 0]);
            continue;
        }
        let mut acc = 0u64;
        for i in 0..v % 13 {
            if i == 7 {
                break;
            }
            for j in 0..5 {
                if (i + j) % 3 == 0 {
                    continue;
                }
                acc += i * j + 1;
            }
        }
        let s = match v % 6 {
            0 => 10,
            1 => acc,
            2 | 3 => acc * 2 + v,
            _ => 99,
        };
        let branch = if v > 500 {
            if v < 700 { 1 } else { 2 }
        } else {
            3
        };
        out.extend([acc, s & 0xFFFF_FFFF, collatz(v % 1000 + 1), branch]);
    }
    plan.add(
        Case::compute1("compute.control_flow", library, "control_flow", n as u32)
            .buffer(Buffer::data("in", 0, words(&data)))
            .buffer(Buffer::zeroed("out", 1, n * 16))
            .output(Output::exact("out", Dtype::U32, words(&out))),
    );
}

/// RGBA texels of a width-wide texture, zero but where `texel` gives one,
/// as half or float bytes.
fn texels(
    width: u32,
    height: u32,
    half: bool,
    texel: impl Fn(u32, u32) -> Option<[f64; 4]>,
) -> Vec<u8> {
    let mut bytes = vec![];
    for y in 0..height {
        for x in 0..width {
            for v in texel(x, y).unwrap_or([0.0; 4]) {
                if half {
                    bytes.extend(f16_bits(v).to_le_bytes());
                } else {
                    bytes.extend((v as f32).to_le_bytes());
                }
            }
        }
    }
    bytes
}

/// Compute kernels' explicit imageblocks: elements in tile memory that one
/// thread writes to a texture as a slice.
fn imageblocks(plan: &mut Plan, library: &str) {
    if plan.has(SOURCE, "imageblock_tiles") {
        let expected = texels(16, 16, true, |x, y| {
            let (group_x, group_y) = (f64::from(x / 8), f64::from(y / 8));
            Some([
                f64::from(x % 8) / 8.0,
                f64::from(y % 8) / 8.0,
                group_x / 2.0 + group_y / 4.0,
                1.0,
            ])
        });
        // The static threadgroup memory, indirect command support and
        // required threadgroup size, then two sizes' imageblock memory of
        // its 8-byte elements.
        let fixed = [0, 0, 0, 0, 0, 8 * 8 * 8, 32 * 32 * 8, 0];
        plan.add(
            Case::compute(
                "compute.imageblock.tiles",
                library,
                "imageblock_tiles",
                [16, 16, 1],
                [8, 8, 1],
            )
            .set("imageblock", json!([8, 8]))
            .set("imageblock_lengths", json!([[8, 8], [32, 32]]))
            .set(
                "pipeline_properties",
                json!({"fixed": "fixed", "limits": "limits"}),
            )
            .buffer(Buffer::zeroed("fixed", 0, 32).unbound())
            .buffer(Buffer::zeroed("limits", 0, 16).unbound())
            .output(Output::exact("fixed", Dtype::U32, u32s(&fixed)))
            .output(Output::new("t", Dtype::U8, Tolerance::exact()).expect_same(expected))
            .texture(
                Texture::d2("t", "rgba16float", 16, 16)
                    .index(0)
                    .usage(&["read", "write"]),
            ),
        );
    }
    if plan.has(SOURCE, "imageblock_unsized_slice") {
        // Without the encoder's dimensions the block is 0 x 0 on Apple GPUs,
        // and a slice of its own size writes nothing.
        let expected = vec![0; 16 * 16 * 8];
        plan.add(
            Case::compute(
                "compute.imageblock.unsized_slice",
                library,
                "imageblock_unsized_slice",
                [16, 16, 1],
                [8, 8, 1],
            )
            .output(Output::new("t", Dtype::U8, Tolerance::exact()).expect_same(expected))
            .texture(
                Texture::d2("t", "rgba16float", 16, 16)
                    .index(0)
                    .usage(&["read", "write"])
                    .data(vec![0; 16 * 16 * 8]),
            ),
        );
    }
    if plan.has(SOURCE, "imageblock_dimensions") {
        // Apple GPUs give a kernel's imageblock no dimensions of their own:
        // without the encoder's it is 0 x 0, not the threadgroup's size.
        for (name, size) in [("unsized", None), ("sized", Some([16, 8]))] {
            let mut case = Case::compute(
                &format!("compute.imageblock.dimensions.{name}"),
                library,
                "imageblock_dimensions",
                [1, 1, 1],
                [16, 16, 1],
            )
            .buffer(Buffer::zeroed("out", 0, 8))
            .output(Output::exact(
                "out",
                Dtype::U32,
                u32s(&size.unwrap_or([0, 0])),
            ));
            if let Some(size) = size {
                case = case.set("imageblock", json!(size));
            }
            plan.add(case);
        }
        // An indirect compute command's own dimensions, or 0 x 0 without
        // them.
        for (name, size) in [("unsized", None), ("sized", Some([16, 8]))] {
            let mut command = json!({"function": "imageblock_dimensions",
                                     "buffers": [{"name": "out", "index": 0, "offset": 0}],
                                     "threadgroups": [1], "threadgroup": [16, 16]});
            if let Some(size) = size {
                command["imageblock"] = json!(size);
            }
            plan.add(
                Case::compute(
                    &format!("compute.imageblock.dimensions.indirect.{name}"),
                    library,
                    "imageblock_dimensions",
                    [1, 1, 1],
                    [16, 16, 1],
                )
                .set("supports_icb", true)
                .set("execute_icb", json!({"icb": "icb", "count": 1}))
                .set(
                    "icbs",
                    json!([{"name": "icb", "type": "dispatch", "count": 1, "max_buffers": 1,
                            "commands": [command]}]),
                )
                .buffer(Buffer::zeroed("out", 0, 8))
                .output(Output::exact(
                    "out",
                    Dtype::U32,
                    u32s(&size.unwrap_or([0, 0])),
                )),
            );
        }
    }
    if plan.has(SOURCE, "imageblock_member") {
        let expected = texels(8, 4, true, |x, y| {
            ((2..5).contains(&x) && (1..3).contains(&y))
                .then(|| [f64::from(x - 2) / 4.0, f64::from(y - 1) / 4.0, 1.0, 0.5])
        });
        plan.add(
            Case::compute(
                "compute.imageblock.member_region",
                library,
                "imageblock_member",
                [4, 4, 1],
                [4, 4, 1],
            )
            .set("imageblock", json!([4, 4]))
            .output(Output::new("t", Dtype::U8, Tolerance::exact()).expect_same(expected))
            .texture(
                Texture::d2("t", "rgba16float", 8, 4)
                    .index(0)
                    .usage(&["read", "write"])
                    .data(vec![0; 8 * 4 * 8]),
            ),
        );
    }
    if plan.has(SOURCE, "imageblock_clipped") {
        let expected = texels(8, 8, false, |x, y| {
            (x >= 4 && y >= 4).then(|| [f64::from(x - 4) / 8.0, f64::from(y - 4) / 8.0, 0.25, 1.0])
        });
        plan.add(
            Case::compute(
                "compute.imageblock.clipped",
                library,
                "imageblock_clipped",
                [8, 8, 1],
                [8, 8, 1],
            )
            .set("imageblock", json!([8, 8]))
            .output(Output::new("t", Dtype::U8, Tolerance::exact()).expect_same(expected))
            .texture(
                Texture::d2("t", "rgba32float", 8, 8)
                    .index(0)
                    .usage(&["read", "write"])
                    .data(vec![0; 8 * 8 * 16]),
            ),
        );
    }
}

pub fn build(plan: &mut Plan) {
    let library = plan.library("compute", SOURCE, &[], super::plan::LATEST);
    builtins(plan, &library);
    reduction(plan, &library);
    atomics(plan, &library);
    store_order(plan, &library);
    simd(plan, &library);
    memory(plan, &library);
    control(plan, &library);
    imageblocks(plan, &library);
}
