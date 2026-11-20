#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Print the recipes and families of the RDNA 2, 3, 3.5 and 4 chips beside
Navi 21, Navi 48 and Raphael, for recipes.rs's AMDGPU and AMDGPU_FAMILIES.

FIRMWARE is a linux-firmware checkout at recipes::GSP_REVISION; RECIPES is
recipes.rs, whose pinned files are not printed again. Each chip's members are
the files whose names start with one of its prefixes; every prefix must match
a file. A chip carries every file of its graphics core and the PSP, SMU, SDMA,
VCN, DCN and VPE files of the versions its family uses; where a family's chips
differ in a block version, each carries all of them, as amdgpu asks only for
its own.

Usage: python3 -I amdgpu_chips.py FIRMWARE RECIPES
"""
import hashlib, os, re, sys

fw, recipes = sys.argv[1], sys.argv[2]
files = sorted(f for f in os.listdir(os.path.join(fw, 'amdgpu')) if f.endswith('.bin'))
pinned = set(re.findall(r'amdgpu\(\s*"amdgpu/([a-z0-9_]+\.bin)"', open(recipes).read()))

STRIX = ['psp_14_0_0_', 'psp_14_0_1_', 'psp_14_0_4_', 'psp_14_0_5_', 'sdma_6_1_0', 'sdma_6_1_1',
         'sdma_6_1_2', 'sdma_6_1_3', 'sdma_6_1_4', 'vcn_4_0_5', 'vcn_4_0_6', 'dcn_3_5_', 'dcn_3_6_',
         'vpe_6_1_0', 'vpe_6_1_1', 'vpe_6_1_3']
NAVI3 = ['dcn_3_2_0_', 'dcn_3_2_1_']


def span(first, last):
    return list(range(first, last + 1))


# (family name, PCI device IDs, member prefixes), the IDs as amdgpu's PCI
# table lists each chip's boards.
CHIPS = [
    ('Radeon RX 6700, 6700 XT and 6750 XT (Navi 22)', [0x73C0, 0x73C1, 0x73C3] + span(0x73DA, 0x73DF),
     ['navy_flounder_']),
    ('Radeon RX 6600, 6600 XT and 6650 XT (Navi 23)',
     span(0x73E0, 0x73E3) + span(0x73E8, 0x73ED) + [0x73EF, 0x73FF], ['dimgrey_cavefish_']),
    ('Radeon RX 6400 and 6500 XT (Navi 24)', span(0x7420, 0x7424) + [0x743F], ['beige_goby_']),
    ('Steam Deck graphics (Van Gogh)', [0x163F], ['vangogh_']),
    ('Ryzen 6000 graphics, Radeon 660M and 680M (Rembrandt)', [0x164D, 0x1681], ['yellow_carp_']),
    ('Ryzen 7020 graphics (Mendocino)', [0x1506],
     ['gc_10_3_7_', 'psp_13_0_8_', 'sdma_5_2_7', 'vcn_3_1_2', 'dcn_3_1_6_']),
    ('Radeon RX 7900 GRE, XT and XTX (Navi 31)', span(0x7448, 0x744C) + [0x745E],
     ['gc_11_0_0_', 'psp_13_0_0_', 'smu_13_0_0', 'sdma_6_0_0', 'vcn_4_0_0'] + NAVI3),
    ('Ryzen 7040 and 8040 graphics, Radeon 760M and 780M (Phoenix)', [0x15BF, 0x164F],
     ['gc_11_0_1_', 'psp_13_0_4_', 'sdma_6_0_1', 'vcn_4_0_2', 'dcn_3_1_4_']),
    ('Radeon RX 7600 and 7600 XT (Navi 33)',
     [0x73F0, 0x7480, 0x7481, 0x7483, 0x7487, 0x7489, 0x748B, 0x7499, 0x749F],
     ['gc_11_0_2_', 'psp_13_0_7_', 'smu_13_0_7', 'sdma_6_0_2', 'vcn_4_0_4'] + NAVI3),
    ('Radeon RX 7700 XT and 7800 XT (Navi 32)', [0x7460, 0x7461, 0x7470, 0x747E],
     ['gc_11_0_3_', 'psp_13_0_10_', 'smu_13_0_10', 'sdma_6_0_3', 'vcn_4_0_0'] + NAVI3),
    ('Ryzen 7040 and 8040 graphics, Radeon 740M (Phoenix 2)', [0x15C8],
     ['gc_11_0_4_', 'psp_13_0_11_', 'sdma_6_0_1', 'vcn_4_0_2', 'dcn_3_1_4_']),
    ('Ryzen AI 300 graphics, Radeon 880M and 890M (Strix Point)', [0x150E], ['gc_11_5_0_'] + STRIX),
    ('Ryzen AI Max graphics, Radeon 8050S and 8060S (Strix Halo)', [0x1586], ['gc_11_5_1_'] + STRIX),
    ('Ryzen AI 5 and 7 300 graphics, Radeon 840M and 860M (Krackan Point)', [0x1114],
     ['gc_11_5_2_'] + STRIX),
    ('Radeon RX 9060 XT (Navi 44)', [0x7590],
     ['gc_12_0_0_', 'psp_14_0_2_', 'smu_14_0_2', 'sdma_7_0_0', 'vcn_5_0_0', 'dcn_4_0_1_']),
]


def members(prefixes):
    found = [f for f in files if any(f.startswith(p) for p in prefixes)]
    for p in prefixes:
        assert any(f.startswith(p) for f in found), p
    return found


recipes_out, families_out, new = [], [], {}
for name, ids, prefixes in CHIPS:
    chip = members(prefixes)
    for f in chip:
        if f in pinned or f in new:
            continue
        data = open(os.path.join(fw, 'amdgpu', f), 'rb').read()
        new[f] = len(data)
        recipes_out.append(
            f'    amdgpu(\n        "amdgpu/{f}",\n        {len(data)},\n'
            f'        "{hashlib.sha256(data).hexdigest()}",\n    ),')
    ids_text = ', '.join(f'0x{i:04x}' for i in ids)
    files_text = ''.join(f'            "amdgpu/{f}",\n' for f in chip)
    families_out.append(f'    (\n        "{name}",\n        &[{ids_text}],\n        &[\n{files_text}        ],\n    ),')
print('// RECIPES')
print('\n'.join(recipes_out))
print('// FAMILIES')
print('\n'.join(families_out))
print(f'// {len(new)} new files, {sum(new.values())} bytes', file=sys.stderr)
