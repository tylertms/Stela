/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "compute.h"
#include "nouveau/headers/nv_push.h"
#include "nv_push_cla0c0.h"
#include "nv_push_clc6c0.h"
#include "clc5c0.h"
#include "clc7c0.h"
#include "clc9c0.h"
#include "clcbc0.h"
#include "clcdc0.h"
#include "clcec0.h"
#include "clc3c0qmd.h"
#include "clc6c0qmd.h"
#include "clcbc0qmd.h"
#include "clcdc0qmd.h"

#define MW_BITS(field) DRF_BITS(DRF_MW(field))
#define MW_MASK(field) DRF_MASK(DRF_MW(field))

/* These limits match the native compute limits published by the existing NVK
 * backend. Descriptor widths below are derived from NVIDIA's class headers. */
#define MAX_GROUP_X INT32_MAX
#define MAX_GROUP_YZ 65535u
#define MAX_LOCAL_XY 1024u
#define MAX_LOCAL_Z 64u
#define MAX_THREADS 1024u
#define CODE_ALIGNMENT 128u /* Original nvk_shader_get_shader_alignment(). */
#define CBUF_ALIGNMENT 64u /* Original Turing+ nvk_min_cbuf_alignment(). */
#define MAX_CBUF_BYTES (1u << 16) /* Original NVK_MAX_CBUF_SIZE. */

struct qmd_limits {
   unsigned program_bits;
   unsigned smem_shift;
   uint32_t smem_mask;
   uint32_t smem_config_mask;
};

static bool
reject(enum stela_nvidia_compute_error *error, enum stela_nvidia_compute_error kind)
{
   if (error)
      *error = kind;
   return false;
}

static bool
device_limits(const struct nv_device_info *dev, struct qmd_limits *limits)
{
   bool matches;
   switch (dev->cls_compute) {
   case 0xc0c0: matches = dev->sm == 60; break;
   case 0xc1c0: matches = dev->sm == 61 || dev->sm == 62; break;
   case TURING_COMPUTE_A: matches = dev->sm == 75; break;
   case AMPERE_COMPUTE_A: matches = dev->sm == 80; break;
   case AMPERE_COMPUTE_B: matches = dev->sm == 86 || dev->sm == 87; break;
   case ADA_COMPUTE_A: matches = dev->sm == 89; break;
   case HOPPER_COMPUTE_A: matches = dev->sm == 90; break;
   case BLACKWELL_COMPUTE_A: matches = dev->sm == 100 || dev->sm == 104; break;
   case BLACKWELL_COMPUTE_B: matches = dev->sm == 120; break;
   default: return false;
   }
   if (!matches)
      return false;
   if (dev->cls_compute >= BLACKWELL_COMPUTE_A) {
      *limits = (struct qmd_limits) {
         .program_bits = MW_BITS(NVCDC0_QMDV05_00_PROGRAM_ADDRESS_LOWER_SHIFTED4) +
                         MW_BITS(NVCDC0_QMDV05_00_PROGRAM_ADDRESS_UPPER_SHIFTED4) + 4,
         .smem_shift = 7,
         .smem_mask = MW_MASK(NVCDC0_QMDV05_00_SHARED_MEMORY_SIZE_SHIFTED7),
         .smem_config_mask = MW_MASK(NVCDC0_QMDV05_00_MAX_SM_CONFIG_SHARED_MEM_SIZE),
      };
   } else if (dev->cls_compute >= HOPPER_COMPUTE_A) {
      *limits = (struct qmd_limits) {
         .program_bits = MW_BITS(NVCBC0_QMDV04_00_PROGRAM_ADDRESS_LOWER) +
                         MW_BITS(NVCBC0_QMDV04_00_PROGRAM_ADDRESS_UPPER),
         .smem_mask = MW_MASK(NVCBC0_QMDV04_00_SHARED_MEMORY_SIZE),
         .smem_config_mask = MW_MASK(NVCBC0_QMDV04_00_MAX_SM_CONFIG_SHARED_MEM_SIZE),
      };
   } else if (dev->cls_compute >= AMPERE_COMPUTE_A) {
      *limits = (struct qmd_limits) {
         .program_bits = MW_BITS(NVC6C0_QMDV03_00_PROGRAM_ADDRESS_LOWER) +
                         MW_BITS(NVC6C0_QMDV03_00_PROGRAM_ADDRESS_UPPER),
         .smem_mask = MW_MASK(NVC6C0_QMDV03_00_SHARED_MEMORY_SIZE),
         .smem_config_mask = MW_MASK(NVC6C0_QMDV03_00_MAX_SM_CONFIG_SHARED_MEM_SIZE),
      };
   } else {
      *limits = (struct qmd_limits) {
         .program_bits = MW_BITS(NVC3C0_QMDV02_02_PROGRAM_ADDRESS_LOWER) +
                         MW_BITS(NVC3C0_QMDV02_02_PROGRAM_ADDRESS_UPPER),
         .smem_mask = MW_MASK(NVC3C0_QMDV02_02_SHARED_MEMORY_SIZE),
         .smem_config_mask = MW_MASK(NVC3C0_QMDV02_02_MAX_SM_CONFIG_SHARED_MEM_SIZE),
      };
   }
   return true;
}

static bool
address_range(uint64_t address, uint64_t bytes, unsigned bits, unsigned alignment)
{
   if (!address || !bytes || bits == 0 || bits > 63 || address % alignment)
      return false;
   uint64_t last = (UINT64_C(1) << bits) - 1;
   return address <= last && bytes - 1 <= last - address;
}

static bool
overlap(uint64_t a, uint64_t a_size, uint64_t b, uint64_t b_size)
{
   /* All caller ranges were checked against <=57-bit VA limits. */
   return a < b + b_size && b < a + a_size;
}

bool
stela_nvidia_compute_encode(const struct nv_device_info *dev,
                        const struct nak_shader_bin *shader,
                        const struct stela_nvidia_compute_dispatch *dispatch,
                        struct stela_nvidia_compute_encoded *encoded,
                        enum stela_nvidia_compute_error *error)
{
   if (error)
      *error = STELA_NVIDIA_COMPUTE_OK;
   if (!dev || !shader || !dispatch || !encoded)
      return reject(error, STELA_NVIDIA_COMPUTE_INPUT);
   struct qmd_limits limits;
   if (!device_limits(dev, &limits) || !dev->max_warps_per_mp || dev->max_warps_per_mp > 64 ||
       !dev->sm_smem_size_count || dev->sm_smem_size_count > ARRAY_SIZE(dev->sm_smem_sizes_kB))
      return reject(error, STELA_NVIDIA_COMPUTE_DEVICE);
   const struct nak_shader_info *info = &shader->info;
   if (!shader->code || !shader->code_size || info->stage != MESA_SHADER_COMPUTE ||
       info->sm != dev->sm || !info->num_gprs || !info->max_warps_per_sm ||
       info->max_warps_per_sm > dev->max_warps_per_mp ||
       info->num_control_barriers > MW_MASK(NVC3C0_QMDV02_02_BARRIER_COUNT))
      return reject(error, STELA_NVIDIA_COMPUTE_SHADER);
   if (info->slm_size || info->crs_size)
      return reject(error, STELA_NVIDIA_COMPUTE_SCRATCH_UNSUPPORTED);

   const struct nak_qmd_info *qmd = &dispatch->qmd;
   if (!info->cs.local_size[0] || info->cs.local_size[0] > MAX_LOCAL_XY ||
       !info->cs.local_size[1] || info->cs.local_size[1] > MAX_LOCAL_XY ||
       !info->cs.local_size[2] || info->cs.local_size[2] > MAX_LOCAL_Z ||
       !qmd->global_size[0] || qmd->global_size[0] > MAX_GROUP_X ||
       !qmd->global_size[1] || qmd->global_size[1] > MAX_GROUP_YZ ||
       !qmd->global_size[2] || qmd->global_size[2] > MAX_GROUP_YZ)
      return reject(error, STELA_NVIDIA_COMPUTE_GEOMETRY);
   uint64_t threads = (uint64_t)info->cs.local_size[0] * info->cs.local_size[1] * info->cs.local_size[2];
   if (threads > MAX_THREADS || (threads + NAK_SUBGROUP_SIZE - 1) / NAK_SUBGROUP_SIZE > info->max_warps_per_sm)
      return reject(error, STELA_NVIDIA_COMPUTE_GEOMETRY);

   /* nak_fill_qmd derives an occupancy carveout from this exact ordered table.
    * Validate each precondition that would otherwise assert/index/panic. */
   for (unsigned i = 0; i < dev->sm_smem_size_count; i++) {
      unsigned size = dev->sm_smem_sizes_kB[i];
      if (size % 4 || size / 4 + 1 > limits.smem_config_mask ||
          (i && size <= dev->sm_smem_sizes_kB[i - 1]))
         return reject(error, STELA_NVIDIA_COMPUTE_SHARED_MEMORY);
   }
   unsigned max_smem_kB = dev->sm_smem_sizes_kB[dev->sm_smem_size_count - 1];
   uint64_t rounded_smem = ((uint64_t)qmd->smem_size + 255) & ~UINT64_C(255);
   if (!max_smem_kB || !dev->max_smem_per_wg_kB || dev->max_smem_per_wg_kB > max_smem_kB ||
       qmd->smem_size < info->cs.smem_size ||
       rounded_smem > (uint32_t)dev->max_smem_per_wg_kB * 1024 ||
       (rounded_smem >> limits.smem_shift) > limits.smem_mask)
      return reject(error, STELA_NVIDIA_COMPUTE_SHARED_MEMORY);

   unsigned va_bits = dispatch->va_bits;
   if (!va_bits || va_bits > limits.program_bits)
      return reject(error, STELA_NVIDIA_COMPUTE_ADDRESS);
   uint32_t qmd_size_B = nak_qmd_size_B(dev);
   unsigned qmd_bits = DRF_BITS(NVA0C0_SEND_PCAS_A_QMD_ADDRESS_SHIFTED8) + 8;
   if (!address_range(qmd->addr, shader->code_size, va_bits, CODE_ALIGNMENT) ||
       !address_range(dispatch->qmd_addr, qmd_size_B, MIN2(va_bits, qmd_bits), NAK_QMD_ALIGN_B) ||
       overlap(qmd->addr, shader->code_size, dispatch->qmd_addr, qmd_size_B))
      return reject(error, STELA_NVIDIA_COMPUTE_ADDRESS);

   if (qmd->num_cbufs > ARRAY_SIZE(qmd->cbufs))
      return reject(error, STELA_NVIDIA_COMPUTE_CONSTANT_BUFFER);
   const unsigned cbuf_align = dev->cls_compute >= TURING_COMPUTE_A ? CBUF_ALIGNMENT : 256u;
   unsigned cbuf_mask = 0;
   for (unsigned i = 0; i < qmd->num_cbufs; i++) {
      const struct nak_qmd_cbuf *cb = &qmd->cbufs[i];
      if (cb->index >= ARRAY_SIZE(qmd->cbufs) || cbuf_mask & (1u << cb->index) ||
          !cb->size || cb->size > MAX_CBUF_BYTES || cb->size % cbuf_align)
         return reject(error, STELA_NVIDIA_COMPUTE_CONSTANT_BUFFER);
      cbuf_mask |= 1u << cb->index;
      struct nak_qmd_cbuf_desc_layout layout = nak_get_qmd_cbuf_desc_layout(dev, cb->index);
      unsigned bits = layout.addr_shift + layout.addr_lo_end - layout.addr_lo_start +
                      layout.addr_hi_end - layout.addr_hi_start;
      if (!address_range(cb->addr, cb->size, MIN2(va_bits, bits), cbuf_align) ||
          overlap(cb->addr, cb->size, dispatch->qmd_addr, qmd_size_B))
         return reject(error, STELA_NVIDIA_COMPUTE_ADDRESS);
   }

   struct stela_nvidia_compute_encoded result = {0};
   result.qmd_size_B = qmd_size_B;
   nak_fill_qmd(dev, info, qmd, result.qmd, qmd_size_B);

   /* This is the original nvk_CmdDispatchBase launch sequence after descriptor
    * upload/state flush, using generated method packers. No queue is touched. */
   struct nv_push push;
   nv_push_init(&push, result.push, ARRAY_SIZE(result.push), BITFIELD_BIT(SUBC_NVA0C0));
   P_MTHD(&push, NVA0C0, SEND_PCAS_A);
   P_NVA0C0_SEND_PCAS_A(&push, dispatch->qmd_addr >> 8);
   if (dev->cls_compute <= TURING_COMPUTE_A) {
      P_IMMD(&push, NVA0C0, SEND_SIGNALING_PCAS_B, {
         .invalidate = INVALIDATE_TRUE, .schedule = SCHEDULE_TRUE,
      });
   } else {
      P_IMMD(&push, NVC6C0, SEND_SIGNALING_PCAS2_B,
             PCAS_ACTION_INVALIDATE_COPY_SCHEDULE);
   }
   result.push_dw = nv_push_dw_count(&push);
   nv_push_validate(&push);
   *encoded = result;
   return true;
}
