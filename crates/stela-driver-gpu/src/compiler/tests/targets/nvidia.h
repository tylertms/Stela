/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef STELA_TARGETS_NVIDIA_H
#define STELA_TARGETS_NVIDIA_H
/* NVIDIA compile targets for every Stela compiler probe. Mesa has no SM table
 * to derive them from, so these are the SM models Stela qualifies, each with
 * its original compute class. They are CPU compiler models, never devices.
 */
#include "nouveau/headers/nv_device_info.h"
#include <stdio.h>
#include <string.h>

#define STELA_NVIDIA_TARGETS(X)          \
   X(60, 0xc0c0)                     \
   X(61, 0xc1c0)                     \
   X(75, TURING_COMPUTE_A)           \
   X(80, AMPERE_COMPUTE_A)           \
   X(86, AMPERE_COMPUTE_B)           \
   X(87, AMPERE_COMPUTE_B)           \
   X(89, ADA_COMPUTE_A)              \
   X(90, HOPPER_COMPUTE_A)           \
   X(100, BLACKWELL_COMPUTE_A)       \
   X(104, BLACKWELL_COMPUTE_A)       \
   X(120, BLACKWELL_COMPUTE_B)

/* Expand STELA_NVIDIA_TARGETS(STELA_NVIDIA_TARGET) where the class headers are
 * included; stela_nvidia_sms lists the models alone.
 */
#define STELA_NVIDIA_TARGET(sm, compute_class) {sm, compute_class},
#define STELA_NVIDIA_SM(sm, compute_class) sm,
static const unsigned stela_nvidia_sms[] = {STELA_NVIDIA_TARGETS(STELA_NVIDIA_SM)};

/* Occupancy and shared-memory carveouts from each architecture's tuning
 * guide, as Nouveau's device table records them for these SM models. NAK
 * itself reads only the SM and occupancy.
 */
static inline struct nv_device_info
stela_nvidia_compiler_device(unsigned sm)
{
   static const uint16_t carveouts_kB[] = {0, 8, 16, 32, 64, 100, 132, 164, 196, 228};
   struct nv_device_info device = {.sm = (uint8_t)sm};
   if (sm == 75) {
      device.max_warps_per_mp = 32;
      device.sm_smem_size_count = 2;
      device.sm_smem_sizes_kB[0] = 32;
      device.sm_smem_sizes_kB[1] = 64;
   } else {
      device.max_warps_per_mp = sm == 86 || sm == 87 || sm == 89 || sm == 120 ? 48 : 64;
      device.sm_smem_size_count = sm == 80 || sm == 87 ? 8 : sm >= 90 && sm < 120 ? 10 : 6;
      memcpy(device.sm_smem_sizes_kB, carveouts_kB,
             device.sm_smem_size_count * sizeof(carveouts_kB[0]));
   }
   device.max_smem_per_wg_kB = device.sm_smem_sizes_kB[device.sm_smem_size_count - 1];
   return device;
}

/* The compiler model with its compute class, for launch descriptors. */
static inline struct nv_device_info
stela_nvidia_target_device(unsigned sm, unsigned compute_class)
{
   struct nv_device_info device = stela_nvidia_compiler_device(sm);
   device.cls_compute = (uint16_t)compute_class;
   return device;
}

static inline void
stela_nvidia_target_name(char name[16], unsigned sm)
{
   snprintf(name, 16, "sm%u", sm);
}

#endif
