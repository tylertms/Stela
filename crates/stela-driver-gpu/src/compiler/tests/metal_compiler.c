/* SPDX-License-Identifier: GPL-2.0-or-later */
/* The Metal device's compiler service: reflection and compilation of
 * kernels, linked vertex and fragment functions, and object and mesh
 * functions. Original LLVM IR fixtures, one AIR module per function as
 * libraries hold them; no Apple output. */
#include "../metal_compiler.h"
#include "air_effect_oracle.h"
#include "air_test_fixture.h"
#include "compiler/nir/nir_serialize.h"
#include "util/blob.h"
#include "llvm-c/BitReader.h"
#include "util/ralloc.h"
#include "vulkan/vulkan_core.h"
#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef NDEBUG
#error "These tests require active assertions"
#endif

static const nir_shader_compiler_options nir_options = {.max_unroll_iterations = 32};

static const char *module =
   "target datalayout = \"e-p:64:64-i32:32-i64:64-f32:32-v128:128-n8:16:32\"\n"
   "target triple = \"air64_v29-apple-macosx27.2.0\"\n";

#define IDENT "Apple metal version 32023.922 (metalfe-32023.922.4)"

static const char *versions =
   "!air.version = !{!20}\n!air.language_version = !{!21}\n!llvm.ident = !{!22}\n"
   "!air.compile_options = !{!23, !24, !25}\n"
   "!20 = !{i32 2, i32 9, i32 0}\n!21 = !{!\"Metal\", i32 4, i32 1, i32 0}\n"
   "!22 = !{!\"" IDENT "\"}\n"
   "!23 = !{!\"air.compile.denorms_disable\"}\n!24 = !{!\"air.compile.fast_math_enable\"}\n"
   "!25 = !{!\"air.compile.framebuffer_fetch_enable\"}\n";

/* Each thread stores its index to a static and a dynamic threadgroup array
 * and copies it out after a barrier. */
static const char *kernel =
   "@scratch = internal addrspace(3) global [16 x i32] undef, align 16\n"
   "declare void @air.wg.barrier(i32, i32) convergent\n"
   "define void @kernel(ptr addrspace(1) %data, ptr addrspace(3) %shared, i32 %tid) {\n"
   "%i = zext i32 %tid to i64\n"
   "%p = getelementptr i32, ptr addrspace(3) %shared, i64 %i\n"
   "store i32 %tid, ptr addrspace(3) %p, align 4\n"
   "store i32 %tid, ptr addrspace(3) @scratch, align 16\n"
   "call void @air.wg.barrier(i32 2, i32 1) convergent\n"
   "%v = load i32, ptr addrspace(3) %p, align 4\n"
   "%q = getelementptr i32, ptr addrspace(1) %data, i64 %i\n"
   "store i32 %v, ptr addrspace(1) %q, align 4\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @kernel, !3, !4, !5}\n!3 = !{}\n!5 = !{!\"air.max_work_group_size\", i32 64}\n"
   "!4 = !{!40, !41, !42}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 2, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 3, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"shared\"}\n"
   "!42 = !{i32 2, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* Reads buffer 1 on every thread and buffer 4 on thread 0 alone. */
static const char *guarded =
   "define void @guarded(ptr addrspace(1) %out, ptr addrspace(1) %extra, i32 %tid) {\n"
   "%i = zext i32 %tid to i64\n"
   "%q = getelementptr i32, ptr addrspace(1) %out, i64 %i\n"
   "%v = load i32, ptr addrspace(1) %q, align 4\n"
   "%first = icmp eq i32 %tid, 0\n"
   "br i1 %first, label %more, label %done\n"
   "more:\n"
   "%e = load i32, ptr addrspace(1) %extra, align 4\n"
   "%sum = add i32 %v, %e\n"
   "br label %done\n"
   "done:\n"
   "%r = phi i32 [ %v, %0 ], [ %sum, %more ]\n"
   "store i32 %r, ptr addrspace(1) %q, align 4\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @guarded, !3, !4}\n!3 = !{}\n"
   "!4 = !{!40, !41, !42}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 1, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"out\"}\n"
   "!41 = !{i32 1, !\"air.buffer\", !\"air.location_index\", i32 4, i32 1, !\"air.read\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"extra\"}\n"
   "!42 = !{i32 2, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* Stores its stage-in value at its thread position; it never reads its
 * index. */
static const char *staged =
   "define void @staged(<4 x float> %value, <2 x i32> %index, ptr addrspace(1) %out, i32 %tid) {\n"
   "%i = zext i32 %tid to i64\n"
   "%q = getelementptr <4 x float>, ptr addrspace(1) %out, i64 %i\n"
   "store <4 x float> %value, ptr addrspace(1) %q, align 16\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @staged, !3, !4}\n!3 = !{}\n!4 = !{!40, !41, !42, !43}\n"
   "!40 = !{i32 0, !\"air.stage_in\", !\"air.location_index\", i32 0, i32 1,"
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"value\"}\n"
   "!41 = !{i32 1, !\"air.stage_in\", !\"air.location_index\", i32 1, i32 1,"
   " !\"air.arg_type_name\", !\"uint2\", !\"air.arg_name\", !\"index\"}\n"
   "!42 = !{i32 2, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 16, !\"air.arg_type_align_size\","
   " i32 16, !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"out\"}\n"
   "!43 = !{i32 3, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* Stores its position in its threadgroup, the threadgroup's shape and its
 * position in the grid, and its column to its threadgroup buffer. */
static const char *shaped =
   "define void @shaped(ptr addrspace(1) %out, ptr addrspace(3) %shared, <3 x i32> %local,"
   " <3 x i32> %shape, <3 x i32> %grid) {\n"
   "store <3 x i32> %local, ptr addrspace(1) %out, align 16\n"
   "%s = getelementptr i8, ptr addrspace(1) %out, i64 16\n"
   "store <3 x i32> %shape, ptr addrspace(1) %s, align 16\n"
   "%g = getelementptr i8, ptr addrspace(1) %out, i64 32\n"
   "store <3 x i32> %grid, ptr addrspace(1) %g, align 16\n"
   "%x = extractelement <3 x i32> %local, i32 0\n"
   "store i32 %x, ptr addrspace(3) %shared, align 4\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @shaped, !3, !4}\n!3 = !{}\n!4 = !{!40, !41, !42, !43, !44}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 16, !\"air.arg_type_align_size\","
   " i32 16, !\"air.arg_type_name\", !\"uint3\", !\"air.arg_name\", !\"out\"}\n"
   "!41 = !{i32 1, !\"air.buffer\", !\"air.location_index\", i32 1, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 3, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"shared\"}\n"
   "!42 = !{i32 2, !\"air.thread_position_in_threadgroup\", !\"air.arg_type_name\", !\"uint3\","
   " !\"air.arg_name\", !\"local\"}\n"
   "!43 = !{i32 3, !\"air.threads_per_threadgroup\", !\"air.arg_type_name\", !\"uint3\","
   " !\"air.arg_name\", !\"shape\"}\n"
   "!44 = !{i32 4, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint3\","
   " !\"air.arg_name\", !\"grid\"}\n";

/* Stores the scale constant where it is defined and the required enabled
 * constant is set, in the form Metal.jl's library builder reads. */
static const char *constants =
   "@scale = external addrspace(2) constant i32\n"
   "@enabled = external addrspace(2) constant i8\n"
   "declare i1 @air.is_function_constant_defined(ptr addrspace(2))\n"
   "define void @constants(ptr addrspace(1) %data, i32 %tid) {\n"
   "%defined = call i1 @air.is_function_constant_defined(ptr addrspace(2) @scale)\n"
   "%e = load i8, ptr addrspace(2) @enabled, align 1\n"
   "%on = icmp ne i8 %e, 0\n"
   "%both = and i1 %defined, %on\n"
   "br i1 %both, label %scaled, label %done\n"
   "scaled:\n%s = load i32, ptr addrspace(2) @scale, align 4\n"
   "%i = zext i32 %tid to i64\n"
   "%q = getelementptr i32, ptr addrspace(1) %data, i64 %i\n"
   "store i32 %s, ptr addrspace(1) %q, align 4\n"
   "br label %done\n"
   "done:\nret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @constants, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n"
   "!air.function_constants = !{!50, !51}\n"
   "!50 = !{ptr addrspace(2) @scale, !\"uint\", !\"scale\", i32 0, i1 false}\n"
   "!51 = !{ptr addrspace(2) @enabled, !\"bool\", !\"enabled\", i32 3, i1 true}\n";

/* Stores a program-scope constant derived from width, as Apple's compiler
 * records `constant int derived = is_function_constant_defined(width) ?
 * width * 2 : 8;`: twice width when it is defined, else eight. */
#define DERIVED(twice)                                                                      \
   "@width = linkonce_odr hidden addrspace(2) externally_initialized constant i32 undef,"   \
   " section \"air.fc_initializer\", align 4\n"                                             \
   "@twice_width = internal unnamed_addr addrspace(2) global i32 0, align 4\n"              \
   "@llvm.global_ctors = appending global [1 x { i32, ptr, ptr }]"                          \
   " [{ i32, ptr, ptr } { i32 65535, ptr @init, ptr null }]\n"                              \
   "declare i1 @air.is_function_constant_defined(ptr addrspace(2))\n"                       \
   "declare i32 @air.unknown(i32)\n"                                                        \
   "declare float @air.convert.f.f32.s.i32(i32)\n"                                          \
   "declare i32 @air.convert.s.i32.f.f32(float)\n"                                          \
   "define void @derived(ptr addrspace(1) %data) {\n"                                       \
   "%v = load i32, ptr addrspace(2) @twice_width, align 4\n"                                \
   "store i32 %v, ptr addrspace(1) %data, align 4\n"                                        \
   "ret void\n}\n"                                                                          \
   "define internal void @init() section \"air.static_init\" {\n"                           \
   "%w = load i32, ptr addrspace(2) @width, align 4\n"                                      \
   "%defined = call i1 @air.is_function_constant_defined(ptr addrspace(2) @width)\n"        \
   twice                                                                                    \
   "%v = select i1 %defined, i32 %x, i32 8\n"                                               \
   "store i32 %v, ptr addrspace(2) @twice_width, align 4\n"                                 \
   "ret void\n}\n"                                                                          \
   "!air.kernel = !{!2}\n"                                                                  \
   "!2 = !{ptr @derived, !3, !4}\n!3 = !{}\n!4 = !{!40}\n"                                  \
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\"," \
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\"," \
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"              \
   "!air.function_constants = !{!50}\n"                                                     \
   "!50 = !{ptr addrspace(2) @width, !\"int\", !\"width\", i32 0}\n"

static const char *derived = DERIVED("%x = shl nsw i32 %w, 1\n");
/* The same through AIR's conversions, as `int(float(width) * 2)`. */
static const char *converted = DERIVED("%f = call float @air.convert.f.f32.s.i32(i32 %w)\n"
                                       "%d = fmul fast float %f, 2.000000e+00\n"
                                       "%x = call i32 @air.convert.s.i32.f.f32(float %d)\n");
/* A constructor whose helper needs the ordinary inlining path. */
static const char *derived_with_helper = DERIVED("%x = call i32 @twice(i32 %w)\n")
   "define internal i32 @twice(i32 %w) {\n%x = shl nsw i32 %w, 1\nret i32 %x\n}\n";
/* A constructor that calls an AIR function, which no constant folds. */
static const char *called = DERIVED("%x = call i32 @air.unknown(i32 %w)\n");
/* One that bounds width with an LLVM intrinsic, which the global
 * optimizer's evaluator does not run but constant folding does. */
static const char *bounded = DERIVED("%x = call i32 @llvm.umin.i32(i32 %w, i32 4)\n")
   "declare i32 @llvm.umin.i32(i32, i32)\n";
/* A derived bool the global optimizer shrank to an i1, as Apple's compiler
 * leaves MPS's program-scope flags: a byte holding 0 or 1. */
static const char *narrowed =
   "@width = linkonce_odr hidden addrspace(2) externally_initialized constant i32 undef,"
   " section \"air.fc_initializer\", align 4\n"
   "@small = internal unnamed_addr addrspace(2) global i1 false, align 2\n"
   "@llvm.global_ctors = appending global [1 x { i32, ptr, ptr }]"
   " [{ i32, ptr, ptr } { i32 65535, ptr @init, ptr null }]\n"
   "define void @narrowed(ptr addrspace(1) %data) {\n"
   "%b = load i1, ptr addrspace(2) @small, align 2\n"
   "%v = select i1 %b, i32 7, i32 3\n"
   "store i32 %v, ptr addrspace(1) %data, align 4\n"
   "ret void\n}\n"
   "define internal void @init() section \"air.static_init\" {\n"
   "%w = load i32, ptr addrspace(2) @width, align 4\n"
   "%s = icmp ult i32 %w, 8\n"
   "store i1 %s, ptr addrspace(2) @small, align 2\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @narrowed, !3, !4}\n!3 = !{}\n!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"
   "!air.function_constants = !{!50}\n"
   "!50 = !{ptr addrspace(2) @width, !\"int\", !\"width\", i32 0}\n";

/* A texture whose presence and location its constructor derives from index,
 * as VideoToolbox's transfer kernels declare `[[texture(index),
 * function_constant(has_image)]]` with has_image = index != -1. The kernel
 * reads its width when `branch` says, on %on. */
#define LOCATED(branch)                                                                     \
   "@index.init = linkonce_odr hidden addrspace(2) externally_initialized constant i32"     \
   " undef, section \"air.fc_initializer\", align 4\n"                                      \
   "@has_image = internal addrspace(2) global i8 0, align 1\n"                              \
   "@image_index = internal addrspace(2) global i32 0, align 4\n"                           \
   "@llvm.global_ctors = appending global [1 x { i32, ptr, ptr }]"                          \
   " [{ i32, ptr, ptr } { i32 65535, ptr @init, ptr null }]\n"                              \
   "@llvm.compiler.used = appending global [2 x ptr] [ptr addrspacecast"                    \
   " (ptr addrspace(2) @has_image to ptr), ptr addrspacecast (ptr addrspace(2) @image_index" \
   " to ptr)], section \"llvm.metadata\"\n"                                                 \
   "declare i8 @air.normalize_function_constant_predicate.i8(i8)\n"                         \
   "declare i32 @air.get_width_texture_2d(ptr addrspace(1), i32)\n"                         \
   "define void @located(ptr addrspace(1) %data, ptr addrspace(1) %image) {\n"              \
   "%u = load i8, ptr addrspace(2) @has_image, align 1\n"                                   \
   "%on = icmp ne i8 %u, 0\n"                                                               \
   branch                                                                                   \
   "read:\n%w = call i32 @air.get_width_texture_2d(ptr addrspace(1) %image, i32 0)\n"       \
   "store i32 %w, ptr addrspace(1) %data, align 4\nret void\n"                              \
   "done:\nret void\n}\n"                                                                   \
   "define internal void @init() section \"air.static_init\" {\n"                           \
   "%i = load i32, ptr addrspace(2) @index.init, align 4\n"                                 \
   "%has = icmp ne i32 %i, -1\n"                                                            \
   "%b = zext i1 %has to i8\n"                                                              \
   "%p = call i8 @air.normalize_function_constant_predicate.i8(i8 %b)\n"                    \
   "store i8 %p, ptr addrspace(2) @has_image, align 1\n"                                    \
   "store i32 %i, ptr addrspace(2) @image_index, align 4\n"                                 \
   "ret void\n}\n"                                                                          \
   "!air.kernel = !{!2}\n"                                                                  \
   "!2 = !{ptr @located, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"                             \
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1,"                 \
   " !\"air.read_write\", !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4,"    \
   " !\"air.arg_type_align_size\", i32 4, !\"air.arg_type_name\", !\"int\","                \
   " !\"air.arg_name\", !\"data\"}\n"                                                       \
   "!41 = !{i32 1, !\"air.function_constant\", !43, !\"air.texture\","                      \
   " !\"air.location_index\", ptr addrspace(2) @image_index, i32 1, !\"air.read\","         \
   " !\"air.arg_type_name\", !\"texture2d<float, read>\", !\"air.arg_name\", !\"image\"}\n" \
   "!43 = !{ptr addrspace(2) @has_image, !\"bool\", !\"has_image\"}\n"                      \
   "!air.function_constants = !{!50}\n"                                                     \
   "!50 = !{ptr addrspace(2) @index.init, !\"int\", !\"index\", i32 0}\n"

static const char *located = LOCATED("br i1 %on, label %read, label %done\n");
/* One reading the texture whatever index is. */
static const char *unguarded_texture = LOCATED("br label %read\n");

/* A [[function_constant(use_tint)]] buffer and one for !use_tint at the
 * same index, as Apple's compiler records them: each argument names an
 * internal predicate the constructor computes and llvm.compiler.used keeps.
 * The kernel reads the buffer whose constant holds, branching as `branch`
 * says on use_tint, %on. */
#define CONDITIONAL(branch, predicate)                                                     \
   "@use_tint.init = linkonce_odr hidden addrspace(2) externally_initialized constant i8"  \
   " undef, section \"air.fc_initializer\", align 1\n"                                     \
   "@use_tint = internal unnamed_addr addrspace(2) global i8 undef, align 1\n"             \
   "@tinted = internal addrspace(2) global i8 0, align 1\n"                                \
   "@untinted = internal addrspace(2) global i8 0, align 1\n"                              \
   "@llvm.global_ctors = appending global [1 x { i32, ptr, ptr }]"                         \
   " [{ i32, ptr, ptr } { i32 65535, ptr @init, ptr null }]\n"                             \
   "@llvm.compiler.used = appending global [2 x ptr] [ptr addrspacecast"                   \
   " (ptr addrspace(2) @tinted to ptr), ptr addrspacecast (ptr addrspace(2) @untinted"     \
   " to ptr)], section \"llvm.metadata\"\n"                                                \
   "declare i8 @air.normalize_function_constant_predicate.i8(i8)\n"                        \
   "define void @conditional(ptr addrspace(1) %data, ptr addrspace(1) %tint,"              \
   " ptr addrspace(1) %plain) {\n"                                                         \
   "%u = load i8, ptr addrspace(2) @use_tint, align 1\n"                                   \
   "%on = icmp ne i8 %u, 0\n"                                                              \
   branch                                                                                  \
   "tinted:\n%t = load i32, ptr addrspace(1) %tint, align 4\n"                             \
   "store i32 %t, ptr addrspace(1) %data, align 4\nret void\n"                             \
   "untinted:\n%p = load i32, ptr addrspace(1) %plain, align 4\n"                          \
   "store i32 %p, ptr addrspace(1) %data, align 4\nret void\n}\n"                          \
   "define internal void @init() section \"air.static_init\" {\n"                          \
   "%v = load i8, ptr addrspace(2) @use_tint.init, align 1\n"                              \
   "store i8 %v, ptr addrspace(2) @use_tint, align 1\n"                                    \
   "%t = call i8 @air.normalize_function_constant_predicate.i8(i8 %v)\n"                   \
   "store i8 %t, ptr addrspace(2) @tinted, align 1\n"                                      \
   "%n = xor i8 %t, 1\n"                                                                   \
   "store i8 %n, ptr addrspace(2) @untinted, align 1\n"                                    \
   "ret void\n}\n"                                                                         \
   "!air.kernel = !{!2}\n"                                                                 \
   "!2 = !{ptr @conditional, !3, !4}\n!3 = !{}\n!4 = !{!40, !41, !42}\n"                   \
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1,"                \
   " !\"air.read_write\", !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4,"   \
   " !\"air.arg_type_align_size\", i32 4, !\"air.arg_type_name\", !\"int\","               \
   " !\"air.arg_name\", !\"data\"}\n"                                                      \
   "!41 = !{i32 1, !\"air.function_constant\", !43, !\"air.buffer\","                      \
   " !\"air.location_index\", i32 1, i32 1, !\"air.read\", !\"air.address_space\", i32 1," \
   " !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"                  \
   " !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"tint\"}\n"                    \
   "!42 = !{i32 2, !\"air.function_constant\", !44, !\"air.buffer\","                      \
   " !\"air.location_index\", i32 1, i32 1, !\"air.read_write\", !\"air.address_space\","  \
   " i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"           \
   " !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"plain\"}\n"                   \
   "!43 = !{ptr addrspace(2) @tinted, !\"" predicate "\", !\"use_tint\"}\n"                \
   "!44 = !{ptr addrspace(2) @untinted, !\"bool\", !\"untinted\"}\n"                       \
   "!air.function_constants = !{!50}\n"                                                    \
   "!50 = !{ptr addrspace(2) @use_tint.init, !\"bool\", !\"use_tint\", i32 0}\n"

static const char *conditional =
   CONDITIONAL("br i1 %on, label %tinted, label %untinted\n", "bool");
/* A shader helper uses the same function constant as its leaf constructor. */
static const char *conditional_with_helper =
   CONDITIONAL("%choose = call i1 @choose_tint(i1 %on)\n"
               "br i1 %choose, label %tinted, label %untinted\n", "bool")
   "define internal i1 @choose_tint(i1 %enabled) {\n"
   "%raw = load i8, ptr addrspace(2) @use_tint.init, align 1\n"
   "%selected = icmp ne i8 %raw, 0\n"
   "%both = and i1 %selected, %enabled\nret i1 %both\n}\n";
/* One reading tint whatever use_tint is. */
static const char *unguarded = CONDITIONAL("br label %tinted\n", "bool");
/* One whose read Apple's compiler hoists ahead of what decides it. */
static const char *hoisted =
   CONDITIONAL("%h = load i32, ptr addrspace(1) %tint, align 4\n"
               "%s = select i1 %on, i32 %h, i32 undef\n"
               "store i32 %s, ptr addrspace(1) %data, align 4\n"
               "br i1 %on, label %tinted, label %untinted\n", "bool");
/* One whose predicate is not a bool. */
static const char *mistyped =
   CONDITIONAL("br i1 %on, label %tinted, label %untinted\n", "int");

/* A [[function_constant(splits)]] buffer of a ushort constant, which Metal
 * tests against zero: its predicate is a ushort, as MPS's matrix kernels'
 * are. The kernel adds the buffer's value when splits is not zero. */
static const char *counted =
   "@splits.init = linkonce_odr hidden addrspace(2) externally_initialized constant i16"
   " undef, section \"air.fc_initializer\", align 2\n"
   "@splits = internal unnamed_addr addrspace(2) global i16 undef, align 2\n"
   "@split = internal addrspace(2) global i16 0, align 2\n"
   "@llvm.global_ctors = appending global [1 x { i32, ptr, ptr }]"
   " [{ i32, ptr, ptr } { i32 65535, ptr @init, ptr null }]\n"
   "@llvm.compiler.used = appending global [1 x ptr] [ptr addrspacecast"
   " (ptr addrspace(2) @split to ptr)], section \"llvm.metadata\"\n"
   "declare i16 @air.normalize_function_constant_predicate.i16(i16)\n"
   "define void @counted(ptr addrspace(1) %data, ptr addrspace(1) %extra) {\n"
   "%u = load i16, ptr addrspace(2) @splits, align 2\n"
   "%on = icmp ne i16 %u, 0\n"
   "br i1 %on, label %added, label %done\n"
   "added:\n%e = load i32, ptr addrspace(1) %extra, align 4\n"
   "store i32 %e, ptr addrspace(1) %data, align 4\nret void\n"
   "done:\nstore i32 7, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "define internal void @init() section \"air.static_init\" {\n"
   "%v = load i16, ptr addrspace(2) @splits.init, align 2\n"
   "store i16 %v, ptr addrspace(2) @splits, align 2\n"
   "%t = call i16 @air.normalize_function_constant_predicate.i16(i16 %v)\n"
   "store i16 %t, ptr addrspace(2) @split, align 2\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @counted, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1,"
   " !\"air.read_write\", !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4,"
   " !\"air.arg_type_align_size\", i32 4, !\"air.arg_type_name\", !\"int\","
   " !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.function_constant\", !43, !\"air.buffer\","
   " !\"air.location_index\", i32 1, i32 1, !\"air.read\", !\"air.address_space\", i32 1,"
   " !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"extra\"}\n"
   "!43 = !{ptr addrspace(2) @split, !\"ushort\", !\"splits\"}\n"
   "!air.function_constants = !{!50}\n"
   "!50 = !{ptr addrspace(2) @splits.init, !\"ushort\", !\"splits\", i32 0}\n";

/* A fragment function that reads a [[function_constant(use_tint)]] buffer on
 * one side of a run-time test. With use_tint false that side cannot run, and
 * the test and its read go with it. */
static const char *absent_path =
   "@use_tint.init = linkonce_odr hidden addrspace(2) externally_initialized constant i8"
   " undef, section \"air.fc_initializer\", align 1\n"
   "@tinted = internal addrspace(2) global i8 0, align 1\n"
   "@llvm.global_ctors = appending global [1 x { i32, ptr, ptr }]"
   " [{ i32, ptr, ptr } { i32 65535, ptr @init, ptr null }]\n"
   "@llvm.compiler.used = appending global [1 x ptr] [ptr addrspacecast"
   " (ptr addrspace(2) @tinted to ptr)], section \"llvm.metadata\"\n"
   "declare i8 @air.normalize_function_constant_predicate.i8(i8)\n"
   "define <4 x float> @absent_path(<4 x float> %position, ptr addrspace(2) %tint) {\n"
   "%x = extractelement <4 x float> %position, i64 0\n"
   "%left = fcmp olt float %x, 0.0\n"
   "br i1 %left, label %read, label %done\n"
   "read:\n%t = load <4 x float>, ptr addrspace(2) %tint, align 16\nbr label %done\n"
   "done:\n%c = phi <4 x float> [ %t, %read ], [ %position, %0 ]\n"
   "ret <4 x float> %c\n}\n"
   "define internal void @init() section \"air.static_init\" {\n"
   "%v = load i8, ptr addrspace(2) @use_tint.init, align 1\n"
   "%t = call i8 @air.normalize_function_constant_predicate.i8(i8 %v)\n"
   "store i8 %t, ptr addrspace(2) @tinted, align 1\n"
   "ret void\n}\n"
   "!air.fragment = !{!2}\n"
   "!2 = !{ptr @absent_path, !3, !4}\n!3 = !{!30}\n"
   "!30 = !{!\"air.render_target\", i32 1, i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"color\"}\n"
   "!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.position\", !\"air.center\", !\"air.no_perspective\","
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"position\"}\n"
   "!41 = !{i32 1, !\"air.function_constant\", !43, !\"air.buffer\","
   " !\"air.location_index\", i32 0, i32 1, !\"air.read\", !\"air.address_space\", i32 2,"
   " !\"air.arg_type_size\", i32 16, !\"air.arg_type_align_size\", i32 16,"
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"tint\"}\n"
   "!43 = !{ptr addrspace(2) @tinted, !\"bool\", !\"use_tint\"}\n"
   "!air.function_constants = !{!50}\n"
   "!50 = !{ptr addrspace(2) @use_tint.init, !\"bool\", !\"use_tint\", i32 0}\n";

/* A kernel as Metal 2.4 records one, in a Debug build: its buffer leaves its
 * address space to its type, and its simdgroup barrier has the older scope. */
static const char *older_kernel =
   "declare void @air.simdgroup.barrier(i32, i32) convergent\n"
   "define void @older(ptr addrspace(1) %data, i32 %tid) !dbg !62 {\n"
   "call void @air.simdgroup.barrier(i32 0, i32 1) convergent, !dbg !63\n"
   "store i32 %tid, ptr addrspace(1) %data, align 4, !dbg !63\n"
   "ret void, !dbg !63\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @older, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n"
   "!llvm.dbg.cu = !{!60}\n!llvm.module.flags = !{!64, !67}\n!llvm_utils.workingdir = !{!68}\n"
   "!60 = distinct !DICompileUnit(language: DW_LANG_C_plus_plus_14, file: !61,"
   " emissionKind: LineTablesOnly)\n"
   "!61 = !DIFile(filename: \"a.metal\", directory: \"/\")\n"
   "!62 = distinct !DISubprogram(name: \"older\", scope: !61, file: !61, line: 1,"
   " type: !65, unit: !60, spFlags: DISPFlagDefinition)\n"
   "!63 = !DILocation(line: 2, scope: !62)\n"
   "!64 = !{i32 2, !\"Debug Info Version\", i32 3}\n"
   "!65 = !DISubroutineType(types: !66)\n!66 = !{}\n"
   "!67 = !{i32 7, !\"Dwarf Version\", i32 4}\n!68 = !{!\"/\"}\n";

/* A kernel as -O0 records one, keeping its values in memory, its matrices in
 * aggregates and its switch to be lowered. */
static const char *unoptimized_kernel =
   "define void @unoptimized(ptr addrspace(1) %data, i32 %tid) noinline optnone {\n"
   "entry:\n"
   "%slot = alloca i32, align 4\n"
   "store i32 %tid, ptr %slot, align 4\n"
   "%id = load i32, ptr %slot, align 4\n"
   "%flags = or i32 1, 2\n"
   "call void @air.wg.barrier(i32 %flags, i32 1) convergent\n"
   "%first = insertvalue {[2 x <2 x float>]} poison, <2 x float> zeroinitializer, 0, 0\n"
   "%second = insertvalue {[2 x <2 x float>]} %first, <2 x float> <float 1.0, float 2.0>, 0, 1\n"
   "%rows = extractvalue {[2 x <2 x float>]} %second, 0\n"
   "%row = extractvalue [2 x <2 x float>] %rows, 1\n"
   "%lane = extractelement <2 x float> %row, i32 %id\n"
   "%written = insertelement <2 x float> %row, float %lane, i32 %id\n"
   "switch i32 %id, label %done [i32 1, label %one]\n"
   "one:\n"
   "store <2 x float> %written, ptr addrspace(1) %data, align 8\n"
   "store <2 x i16> bitcast (<1 x float> splat (float 1.0) to <2 x i16>), ptr addrspace(1) %data, align 4\n"
   "br label %done\n"
   "done:\n"
   "ret void\n}\n"
   "declare void @air.wg.barrier(i32, i32) convergent\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @unoptimized, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.arg_type_size\", i32 8, !\"air.arg_type_align_size\", i32 8,"
   " !\"air.arg_type_name\", !\"float2\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* A position and a varying only a true function constant keeps. */
static const char *optional_output =
   "@fog = internal addrspace(2) global i8 undef, align 1\n"
   "@fog.init = linkonce_odr hidden addrspace(2) externally_initialized constant i8 undef,"
   " section \"air.fc_initializer\", align 1\n"
   "@fogged = internal addrspace(2) global i8 0, align 1\n"
   "@llvm.global_ctors = appending global [1 x { i32, ptr, ptr }]"
   " [{ i32, ptr, ptr } { i32 65535, ptr @init, ptr null }]\n"
   "@llvm.compiler.used = appending global [1 x ptr] [ptr addrspacecast (ptr addrspace(2) @fogged"
   " to ptr)], section \"llvm.metadata\"\n"
   "declare i8 @air.normalize_function_constant_predicate.i8(i8)\n"
   "define <{ <4 x float>, float }> @optional(<4 x float> %p) {\n"
   "%a = insertvalue <{ <4 x float>, float }> undef, <4 x float> %p, 0\n"
   "%b = insertvalue <{ <4 x float>, float }> %a, float 1.0, 1\n"
   "ret <{ <4 x float>, float }> %b\n}\n"
   "define internal void @init() section \"air.static_init\" {\n"
   "%v = load i8, ptr addrspace(2) @fog.init, align 1\n"
   "store i8 %v, ptr addrspace(2) @fog, align 1\n"
   "%t = call i8 @air.normalize_function_constant_predicate.i8(i8 %v)\n"
   "store i8 %t, ptr addrspace(2) @fogged, align 1\n"
   "ret void\n}\n"
   "!air.vertex = !{!2}\n"
   "!2 = !{ptr @optional, !3, !4}\n!3 = !{!30, !31}\n!4 = !{!40}\n"
   "!30 = !{!\"air.position\", !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"p\"}\n"
   "!31 = !{!\"air.function_constant\", !32, !\"air.vertex_output\", !\"generated(f)\","
   " !\"air.arg_type_name\", !\"float\", !\"air.arg_name\", !\"f\"}\n"
   "!32 = !{ptr addrspace(2) @fogged, !\"bool\", !\"fog\"}\n"
   "!40 = !{i32 0, !\"air.vertex_input\", !\"air.location_index\", i32 0, i32 1,"
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"in\"}\n"
   "!air.function_constants = !{!50}\n"
   "!50 = !{ptr addrspace(2) @fog.init, !\"bool\", !\"fog\", i32 0}\n";

/* Lanes Apple's compiler leaves poison in a splat, and a value it freezes. */
static const char *undefined_lanes =
   "define void @lanes(ptr addrspace(1) %data, i32 %tid) {\n"
   "%f = uitofp i32 %tid to float\n"
   "%a = insertelement <4 x float> poison, float %f, i64 0\n"
   "%s = shufflevector <4 x float> %a, <4 x float> undef, <4 x i32> <i32 0, i32 0, i32 0, i32 poison>\n"
   "%m = fmul fast <4 x float> %s, %s\n"
   "store <4 x float> %m, ptr addrspace(1) %data, align 16\n"
   "%n = extractelement <4 x float> %s, i64 3\n"
   "%g = freeze <4 x float> %s\n"
   "%x = extractelement <4 x float> %g, i64 3\n"
   "%y = fadd fast float %n, %x\n"
   "%c = icmp eq i32 %tid, 0\n"
   "%z = select i1 %c, float undef, float %y\n"
   "store float %z, ptr addrspace(1) %data, align 4\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @lanes, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 16, !\"air.arg_type_align_size\","
   " i32 16, !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* Texture and sampler array arguments, which a function reflects slot by
 * slot, reading the width of the second texture. */
static const char *arrays =
   "declare i32 @air.get_width_texture_2d(ptr addrspace(1), i32)\n"
   "define void @arrays(ptr byval([3 x ptr addrspace(1)]) %t, ptr byval([2 x ptr addrspace(2)]) %s,"
   " ptr addrspace(1) %data) {\n"
   "%h = getelementptr inbounds [3 x ptr addrspace(1)], ptr %t, i64 0, i64 1\n"
   "%x = load ptr addrspace(1), ptr %h, align 8\n"
   "%w = call i32 @air.get_width_texture_2d(ptr addrspace(1) %x, i32 0)\n"
   "store i32 %w, ptr addrspace(1) %data, align 4\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @arrays, !3, !4}\n!3 = !{}\n!4 = !{!5, !6, !7}\n"
   "!5 = !{i32 0, !\"air.texture\", !\"air.location_index\", i32 2, i32 3, !\"air.read\","
   " !\"air.arg_type_name\", !\"array<texture2d<half, read>, 3>\", !\"air.arg_name\", !\"t\"}\n"
   "!6 = !{i32 1, !\"air.sampler\", !\"air.location_index\", i32 1, i32 2,"
   " !\"air.arg_type_name\", !\"array<sampler, 2>\", !\"air.arg_name\", !\"s\"}\n"
   "!7 = !{i32 2, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n";

/* A kernel binding a uniform structure, which inlines an array of another,
 * an argument buffer of a texture array, a buffer and a constant, a texture,
 * a sampler and a threadgroup buffer; it reads only the uniforms. */
/* A tile function: its implicit imageblock, which only names the color
 * attachments its calls access, and a buffer. */
static const char *tiled =
   "%block_t = type { ptr addrspace(4) }\n"
   "declare <4 x half> @air.load.implicit_imageblock.v4f16(i32, <2 x i16>, i32, i16)\n"
   "define void @tiled(%block_t %block, ptr addrspace(1) %out, <2 x i16> %tid) {\n"
   "%v = call <4 x half> @air.load.implicit_imageblock.v4f16(i32 0, <2 x i16> %tid, i32 0, i16 0)\n"
   "store <4 x half> %v, ptr addrspace(1) %out, align 8\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @tiled, !3, !4}\n!3 = !{}\n!4 = !{!5, !6, !7}\n"
   "!5 = !{i32 0, !\"air.imageblock\", !\"implicit\", !\"air.arg_type_align_size\", i32 8,"
   " !\"air.arg_type_name\", !\"imageblock<Color, layout_implicit>\", !\"air.arg_name\", !\"block\"}\n"
   "!6 = !{i32 1, !\"air.buffer\", !\"air.location_index\", i32 3, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 8, !\"air.arg_type_align_size\","
   " i32 8, !\"air.arg_type_name\", !\"half4\", !\"air.arg_name\", !\"out\"}\n"
   "!7 = !{i32 2, !\"air.thread_position_in_threadgroup\", !\"air.arg_type_name\", !\"ushort2\","
   " !\"air.arg_name\", !\"tid\"}\n";

static const char *bound =
   "define void @bound(ptr addrspace(2) %u, ptr addrspace(2) %args, ptr addrspace(1) %rw,"
   " ptr addrspace(2) %s, ptr addrspace(3) %tg, ptr addrspace(1) %out) {\n"
   "%v = load float, ptr addrspace(2) %u, align 16\n"
   "store float %v, ptr addrspace(1) %out, align 4\n"
   "ret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @bound, !3, !4}\n!3 = !{}\n!4 = !{!5, !6, !7, !8, !9, !10}\n"
   "!5 = !{i32 0, !\"air.buffer\", !\"air.buffer_size\", i32 128, !\"air.location_index\", i32 0,"
   " i32 1, !\"air.read\", !\"air.address_space\", i32 2, !\"air.struct_type_info\", !30,"
   " !\"air.arg_type_size\", i32 128, !\"air.arg_type_align_size\", i32 16,"
   " !\"air.arg_type_name\", !\"Uniforms\", !\"air.arg_name\", !\"u\"}\n"
   "!30 = !{i32 0, i32 64, i32 0, !\"float4x4\", !\"mvp\", !\"air.struct_type_info\", !31,"
   " i32 64, i32 16, i32 2, !\"Inner\", !\"inner\", i32 96, i32 4, i32 3, !\"int\", !\"k\"}\n"
   "!31 = !{i32 0, i32 8, i32 0, !\"float2\", !\"uv\", i32 8, i32 2, i32 0, !\"half\", !\"h\"}\n"
   "!6 = !{i32 1, !\"air.indirect_buffer\", !\"air.buffer_size\", i32 32, !\"air.location_index\","
   " i32 1, i32 1, !\"air.read\", !\"air.address_space\", i32 2, !\"air.struct_type_info\", !32,"
   " !\"air.arg_type_size\", i32 32, !\"air.arg_type_align_size\", i32 8,"
   " !\"air.arg_type_name\", !\"Args\", !\"air.arg_name\", !\"args\"}\n"
   "!32 = !{i32 0, i32 16, i32 0, !\"array<texture2d<float, sample>, 2>\", !\"ts\","
   " !\"air.indirect_argument\", !33, i32 16, i32 8, i32 0, !\"float\", !\"d\","
   " !\"air.indirect_argument\", !34, i32 24, i32 4, i32 0, !\"float\", !\"c\","
   " !\"air.indirect_argument\", !35}\n"
   "!33 = !{i32 0, !\"air.texture\", !\"air.location_index\", i32 0, i32 2, !\"air.sample\","
   " !\"air.arg_type_name\", !\"array<texture2d<float, sample>, 2>\", !\"air.arg_name\", !\"ts\"}\n"
   "!34 = !{i32 1, !\"air.buffer\", !\"air.location_index\", i32 2, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"float\", !\"air.arg_name\", !\"d\"}\n"
   "!35 = !{i32 2, !\"air.indirect_constant\", !\"air.location_index\", i32 3, i32 1,"
   " !\"air.arg_type_name\", !\"float\", !\"air.arg_name\", !\"c\"}\n"
   "!7 = !{i32 2, !\"air.texture\", !\"air.location_index\", i32 1, i32 1, !\"air.read_write\","
   " !\"air.arg_type_name\", !\"texture2d<float, read_write>\", !\"air.arg_name\", !\"rw\"}\n"
   "!8 = !{i32 3, !\"air.sampler\", !\"air.location_index\", i32 0, i32 1,"
   " !\"air.arg_type_name\", !\"sampler\", !\"air.arg_name\", !\"s\"}\n"
   "!9 = !{i32 4, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 3, !\"air.arg_type_size\", i32 16, !\"air.arg_type_align_size\","
   " i32 16, !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"tg\"}\n"
   "!10 = !{i32 5, !\"air.buffer\", !\"air.location_index\", i32 2, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"float\", !\"air.arg_name\", !\"out\"}\n";

/* A kernel calling a [[noinline]] helper, which calls a visible function, and
 * one calling a function that recurses. */
static const char *calls =
   "define i32 @doubled(i32 %x) {\n%y = shl i32 %x, 1\nret i32 %y\n}\n"
   "define internal i32 @helper(i32 %x) noinline {\n"
   "%d = call i32 @doubled(i32 %x)\n%y = add i32 %d, 1\nret i32 %y\n}\n"
   "define internal i32 @countdown(i32 %x) noinline {\n"
   "entry:\n%zero = icmp eq i32 %x, 0\nbr i1 %zero, label %done, label %next\n"
   "next:\n%less = sub i32 %x, 1\n%r = call i32 @countdown(i32 %less)\nbr label %done\n"
   "done:\n%v = phi i32 [0, %entry], [%r, %next]\nret i32 %v\n}\n"
   "define void @calls(ptr addrspace(1) %data, i32 %tid) {\n"
   "%v = call i32 @helper(i32 %tid)\n"
   "store i32 %v, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "define void @recurses(ptr addrspace(1) %data, i32 %tid) {\n"
   "%v = call i32 @countdown(i32 %tid)\n"
   "store i32 %v, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "!air.kernel = !{!2, !5}\n!air.visible = !{!6}\n"
   "!2 = !{ptr @calls, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n!5 = !{ptr @recurses, !3, !4}\n"
   "!6 = !{ptr @doubled, !7, !8, !9}\n!7 = !{!70}\n!8 = !{!80}\n"
   "!9 = !{!\"air.stitching_info\"}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n"
   "!70 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"int\"}\n"
   "!80 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"x\"}\n";

/* A kernel calling handle tid & 1 of its visible function table, then an
 * external function a linked module defines, and a function of its own the
 * table can name. */
#define TABLES(call)                                                                        \
   "declare ptr @air.get_function_pointer_visible_function_table(ptr addrspace(1), i32)\n"  \
   "declare i32 @air.get_size_visible_function_table(ptr addrspace(1))\n"                  \
   "declare i32 @external(i32)\n"                                                         \
   "define i32 @doubled(i32 %x) {\n%y = shl i32 %x, 1\nret i32 %y\n}\n"                     \
   "define void @tables(ptr addrspace(1) %table, ptr addrspace(1) %data, i32 %tid) {\n"     \
   "%which = and i32 %tid, 1\n"                                                           \
   "%f = call ptr @air.get_function_pointer_visible_function_table(ptr addrspace(1) %table," \
   " i32 %which)\n"                                                                       \
   call                                                                                   \
   "%w = call i32 @external(i32 %v)\n"                                                    \
   "store i32 %w, ptr addrspace(1) %data, align 4\nret void\n}\n"                         \
   "!air.kernel = !{!2}\n!air.visible = !{!5}\n"                                          \
   "!2 = !{ptr @tables, !3, !4}\n!3 = !{}\n!4 = !{!40, !41, !42}\n"                        \
   "!5 = !{ptr @doubled, !6, !7}\n!6 = !{!60}\n!7 = !{!70}\n"                              \
   "!40 = !{i32 0, !\"air.visible_function_table\", !\"air.location_index\", i32 0, i32 1," \
   " !\"air.read\", !\"air.arg_type_name\", !\"visible_function_table\", !\"air.arg_name\"," \
   " !\"table\"}\n"                                                                       \
   "!41 = !{i32 1, !\"air.buffer\", !\"air.location_index\", i32 1, i32 1, !\"air.read_write\"," \
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\"," \
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"              \
   "!42 = !{i32 2, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","    \
   " !\"air.arg_name\", !\"tid\"}\n"                                                       \
   "!60 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"int\"}\n"                   \
   "!70 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"int\", !\"air.arg_name\"," \
   " !\"x\"}\n"

static const char *tables = TABLES("%v = call i32 %f(i32 %tid)\n");
/* A call through a bitcast of the pointer to its own type, as Apple's
 * compiler emits it. */
static const char *recast = TABLES("%g = bitcast ptr %f to ptr\n%v = call i32 %g(i32 %tid)\n");
/* A call with the table's size. */
static const char *sized =
   TABLES("%n = call i32 @air.get_size_visible_function_table(ptr addrspace(1) %table)\n"
          "%v = call i32 %f(i32 %n)\n");
/* A call with whether the table is null, and the size of a null table. */
static const char *tested =
   "declare i1 @air.is_null_visible_function_table(ptr addrspace(1))\n"
   "declare ptr addrspace(1) @air.get_null_visible_function_table()\n"
   TABLES("%null = call i1 @air.is_null_visible_function_table(ptr addrspace(1) %table)\n"
          "%none = call ptr addrspace(1) @air.get_null_visible_function_table()\n"
          "%empty = call i32 @air.get_size_visible_function_table(ptr addrspace(1) %none)\n"
          "%n = zext i1 %null to i32\n%m = add i32 %n, %empty\n%v = call i32 %f(i32 %m)\n");
/* A table's pointer used as other than a callee. */
static const char *escaping = TABLES("%v = ptrtoint ptr %f to i32\n");
/* The library linked with it: the external function and two visible
 * functions, one of another type. */
static const char *linked =
   "define i32 @external(i32 %x) {\n%y = add i32 %x, 7\nret i32 %y\n}\n"
   "define i32 @negated(i32 %x) {\n%y = sub i32 0, %x\nret i32 %y\n}\n"
   "define i64 @wide(i64 %x) {\nret i64 %x\n}\n"
   "!air.visible = !{}\n";

/* A kernel storing over the value its data holds what handle 1 of its
 * table returns for it: @down adds 2 per call it makes through the table
 * until its argument is zero, and @split adds 1 per call it makes twice. */
static const char *recursive =
   "declare ptr @air.get_function_pointer_visible_function_table(ptr addrspace(1), i32)\n"
   "define i32 @down(ptr addrspace(1) %table, i32 %x) {\n"
   "entry:\n%zero = icmp eq i32 %x, 0\nbr i1 %zero, label %done, label %next\n"
   "next:\n%f = call ptr @air.get_function_pointer_visible_function_table(ptr addrspace(1)"
   " %table, i32 0)\n%less = sub i32 %x, 1\n"
   "%r = call i32 %f(ptr addrspace(1) %table, i32 %less)\n%s = add i32 %r, 2\nbr label %done\n"
   "done:\n%v = phi i32 [0, %entry], [%s, %next]\nret i32 %v\n}\n"
   "define i32 @split(ptr addrspace(1) %table, i32 %x) {\n"
   "entry:\n%zero = icmp eq i32 %x, 0\nbr i1 %zero, label %done, label %next\n"
   "next:\n%f = call ptr @air.get_function_pointer_visible_function_table(ptr addrspace(1)"
   " %table, i32 0)\n%less = sub i32 %x, 1\n"
   "%a = call i32 %f(ptr addrspace(1) %table, i32 %less)\n"
   "%b = call i32 %f(ptr addrspace(1) %table, i32 %less)\n"
   "%s = add i32 %a, %b\n%t = add i32 %s, 1\nbr label %done\n"
   "done:\n%v = phi i32 [0, %entry], [%t, %next]\nret i32 %v\n}\n"
   "define void @recursive(ptr addrspace(1) %table, ptr addrspace(1) %data) {\n"
   "%f = call ptr @air.get_function_pointer_visible_function_table(ptr addrspace(1) %table,"
   " i32 0)\n%n = load i32, ptr addrspace(1) %data, align 4\n"
   "%v = call i32 %f(ptr addrspace(1) %table, i32 %n)\n"
   "store i32 %v, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @recursive, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.visible_function_table\", !\"air.location_index\", i32 0, i32 1,"
   " !\"air.read\", !\"air.arg_type_name\", !\"visible_function_table\", !\"air.arg_name\","
   " !\"table\"}\n"
   "!41 = !{i32 1, !\"air.buffer\", !\"air.location_index\", i32 1, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n";

/* Stores the external function of its bias constant: a library with an
 * offset constant of the same index defines it, adding the offset. */
static const char *biased =
   "@bias = external addrspace(2) constant i32\n"
   "declare i32 @external(i32)\n"
   "define void @biased(ptr addrspace(1) %data) {\n"
   "%b = load i32, ptr addrspace(2) @bias, align 4\n"
   "%w = call i32 @external(i32 %b)\n"
   "store i32 %w, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @biased, !3, !4}\n!3 = !{}\n!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"
   "!air.function_constants = !{!50}\n"
   "!50 = !{ptr addrspace(2) @bias, !\"int\", !\"bias\", i32 0, i1 true}\n";
static const char *offsets =
   "@offset = external addrspace(2) constant i32\n"
   "define i32 @external(i32 %x) {\n"
   "%o = load i32, ptr addrspace(2) @offset, align 4\n"
   "%y = add i32 %x, %o\nret i32 %y\n}\n"
   "!air.visible = !{}\n"
   "!air.function_constants = !{!90}\n"
   "!90 = !{ptr addrspace(2) @offset, !\"int\", !\"offset\", i32 0, i1 true}\n";

/* A kernel that makes one call on command 0 of an argument buffer's
 * indirect command buffer. */
#define COMMANDS(command) \
   "declare void @air." command "_render_command(ptr addrspace(1), i32)\n" \
   "define void @commands(ptr addrspace(1) %icb) {\n" \
   "%cb = load ptr addrspace(1), ptr addrspace(1) %icb, align 8\n" \
   "call void @air." command "_render_command(ptr addrspace(1) %cb, i32 0)\n" \
   "ret void\n}\n" \
   "!air.kernel = !{!0}\n!0 = !{ptr @commands, !1, !2}\n!1 = !{}\n!2 = !{!3}\n" \
   "!3 = !{i32 0, !\"air.indirect_buffer\", !\"air.buffer_size\", i32 8, !\"air.location_index\"," \
   " i32 0, i32 1, !\"air.read_write\", !\"air.address_space\", i32 1, !\"air.struct_type_info\", !4," \
   " !\"air.arg_type_size\", i32 8, !\"air.arg_type_align_size\", i32 8, !\"air.arg_type_name\"," \
   " !\"Commands\", !\"air.arg_name\", !\"icb\"}\n" \
   "!4 = !{i32 0, i32 8, i32 0, !\"command_buffer\", !\"commands\", !\"air.indirect_argument\", !5}\n" \
   "!5 = !{i32 0, !\"air.command_buffer\", !\"air.location_index\", i32 0, i32 1," \
   " !\"air.arg_type_name\", !\"command_buffer\", !\"air.arg_name\", !\"commands\"}\n"
static const char *resets = COMMANDS("reset");
static const char *barriers = COMMANDS("set_barrier");

#define VERTEX_RESULT "<{ <4 x float>, <2 x float> }>"

static const char *vertex =
   "define " VERTEX_RESULT " @vertex(<4 x float> %p) {\n"
   "%uv = shufflevector <4 x float> %p, <4 x float> %p, <2 x i32> <i32 0, i32 1>\n"
   "%a0 = insertvalue " VERTEX_RESULT " undef, <4 x float> %p, 0\n"
   "%a1 = insertvalue " VERTEX_RESULT " %a0, <2 x float> %uv, 1\n"
   "ret " VERTEX_RESULT " %a1\n}\n"
   "!air.vertex = !{!2}\n"
   "!2 = !{ptr @vertex, !3, !4}\n!3 = !{!30, !31}\n"
   "!30 = !{!\"air.position\", !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"position\"}\n"
   "!31 = !{!\"air.vertex_output\", !\"user(uv)\", !\"air.arg_type_name\", !\"float2\","
   " !\"air.arg_name\", !\"uv\"}\n"
   "!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.vertex_input\", !\"air.location_index\", i32 3, i32 1,"
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"p\"}\n";

/* Returns its first vertex input; it never reads the second. */
static const char *unread =
   "define <4 x float> @unread(<4 x float> %p, <4 x float> %q) {\n"
   "ret <4 x float> %p\n}\n"
   "!air.vertex = !{!2}\n"
   "!2 = !{ptr @unread, !3, !4}\n!3 = !{!30}\n"
   "!30 = !{!\"air.position\", !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"position\"}\n"
   "!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.vertex_input\", !\"air.location_index\", i32 0, i32 1,"
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"p\"}\n"
   "!41 = !{i32 1, !\"air.vertex_input\", !\"air.location_index\", i32 1, i32 1,"
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"q\"}\n";

#define CONTROL_POINT "{ <4 x float>, <2 x float> }"

/* A triangle patch's function at the position of a control point its
 * patch ID selects; it never reads the points' second member. */
static const char *patch =
   "define <4 x float> @patch(ptr %points, <3 x float> %at, i32 %id) {\n"
   "%point = call " CONTROL_POINT " @control_point(i32 %id, ptr %points)\n"
   "%position = extractvalue " CONTROL_POINT " %point, 0\n"
   "ret <4 x float> %position\n}\n"
   "declare " CONTROL_POINT " @control_point(i32, ptr) section \"air.externally_defined\"\n"
   "!air.vertex = !{!2}\n"
   "!2 = !{ptr @patch, !3, !4, !5}\n!3 = !{!30}\n"
   "!30 = !{!\"air.position\", !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"position\"}\n"
   "!4 = !{!40, !41, !42}\n"
   "!40 = !{i32 0, !\"air.patch_control_point_input\", !43, !44, !45}\n"
   "!43 = !{!\"air.patch_control_point_function\", ptr @control_point}\n"
   "!44 = !{!\"air.location_index\", i32 1, i32 1, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"position\"}\n"
   "!45 = !{!\"air.location_index\", i32 2, i32 1, !\"air.arg_type_name\", !\"float2\","
   " !\"air.arg_name\", !\"uv\", !\"air.arg_unused\"}\n"
   "!41 = !{i32 1, !\"air.position_in_patch\", !\"air.arg_type_name\", !\"float3\","
   " !\"air.arg_name\", !\"at\"}\n"
   "!42 = !{i32 2, !\"air.patch_id\", !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"id\"}\n"
   "!5 = !{!\"air.patch\", !\"triangle\", !\"air.patch_control_point\", i32 3}\n";

/* Discards fragments left of the origin. */
static const char *fragment =
   "declare void @air.discard_fragment()\n"
   "define <4 x float> @fragment(<2 x float> %uv) {\n"
   "%x = extractelement <2 x float> %uv, i64 0\n"
   "%left = fcmp olt float %x, 0.0\n"
   "br i1 %left, label %discard, label %shade\n"
   "discard:\ncall void @air.discard_fragment()\nbr label %shade\n"
   "shade:\n%c = shufflevector <2 x float> %uv, <2 x float> %uv, <4 x i32> <i32 0, i32 1, i32 0, i32 1>\n"
   "ret <4 x float> %c\n}\n"
   "!air.fragment = !{!2}\n"
   "!2 = !{ptr @fragment, !3, !4}\n!3 = !{!30}\n"
   "!30 = !{!\"air.render_target\", i32 1, i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"color\"}\n"
   "!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.fragment_input\", !\"user(uv)\", !\"air.center\", !\"air.perspective\","
   " !\"air.arg_type_name\", !\"float2\", !\"air.arg_name\", !\"uv\"}\n";

/* Returns the color its render target holds: programmable blending. */
static const char *blended =
   "define <4 x float> @blended(<4 x float> %previous) {\nret <4 x float> %previous\n}\n"
   "!air.fragment = !{!2}\n"
   "!2 = !{ptr @blended, !3, !4}\n!3 = !{!30}\n"
   "!30 = !{!\"air.render_target\", i32 0, i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"color\"}\n"
   "!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.render_target\", i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"previous\"}\n";

/* Returns its render target's color and half its depth. */
static const char *deepened =
   "define <{ <4 x float>, float }> @deepened(<4 x float> %previous, <4 x float> %position) {\n"
   "%z = extractelement <4 x float> %position, i64 2\n%h = fmul float %z, 0.5\n"
   "%r0 = insertvalue <{ <4 x float>, float }> undef, <4 x float> %previous, 0\n"
   "%r = insertvalue <{ <4 x float>, float }> %r0, float %h, 1\n"
   "ret <{ <4 x float>, float }> %r\n}\n"
   "!air.fragment = !{!2}\n"
   "!2 = !{ptr @deepened, !3, !4}\n!3 = !{!30, !31}\n"
   "!30 = !{!\"air.render_target\", i32 0, i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"color\"}\n"
   "!31 = !{!\"air.depth\", !\"air.depth_qualifier\", !\"air.any\", !\"air.arg_type_name\","
   " !\"float\", !\"air.arg_name\", !\"depth\"}\n"
   "!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.render_target\", i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"previous\"}\n"
   "!41 = !{i32 1, !\"air.position\", !\"air.center\", !\"air.no_perspective\","
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"position\"}\n";

/* Returns its render target's color as both blend sources. */
static const char *dual =
   "define <{ <4 x float>, <4 x float> }> @dual(<4 x float> %previous) {\n"
   "%r0 = insertvalue <{ <4 x float>, <4 x float> }> undef, <4 x float> %previous, 0\n"
   "%r = insertvalue <{ <4 x float>, <4 x float> }> %r0, <4 x float> %previous, 1\n"
   "ret <{ <4 x float>, <4 x float> }> %r\n}\n"
   "!air.fragment = !{!2}\n"
   "!2 = !{ptr @dual, !3, !4}\n!3 = !{!30, !31}\n"
   "!30 = !{!\"air.render_target\", i32 0, i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"color\"}\n"
   "!31 = !{!\"air.render_target\", i32 0, i32 1, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"second\"}\n"
   "!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.render_target\", i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"previous\"}\n";

/* Returns its render target's color in the first column of pixels and twice
 * its position elsewhere: a read that only a phi uses. */
static const char *selected =
   "define <4 x float> @selected(<4 x float> %previous, <4 x float> %position) {\n"
   "entry:\n%x = extractelement <4 x float> %position, i64 0\n%first = fcmp olt float %x, 1.0\n"
   "br i1 %first, label %done, label %other\nother:\n"
   "%twice = fmul <4 x float> %position, <float 2.0, float 2.0, float 2.0, float 2.0>\nbr label %done\n"
   "done:\n%r = phi <4 x float> [ %previous, %entry ], [ %twice, %other ]\n"
   "ret <4 x float> %r\n}\n"
   "!air.fragment = !{!2}\n"
   "!2 = !{ptr @selected, !3, !4}\n!3 = !{!30}\n"
   "!30 = !{!\"air.render_target\", i32 0, i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"color\"}\n"
   "!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.render_target\", i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"previous\"}\n"
   "!41 = !{i32 1, !\"air.position\", !\"air.center\", !\"air.no_perspective\","
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"position\"}\n";

/* Returns its position as its render target's color. */
static const char *positioned =
   "define <4 x float> @positioned(<4 x float> %position) {\nret <4 x float> %position\n}\n"
   "!air.fragment = !{!2}\n"
   "!2 = !{ptr @positioned, !3, !4}\n!3 = !{!30}\n"
   "!30 = !{!\"air.render_target\", i32 0, i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"color\"}\n"
   "!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.position\", !\"air.center\", !\"air.no_perspective\","
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"position\"}\n";

/* Returns twice its render target's color in the first row or column of
 * pixels and the color itself elsewhere, by a branch that either test takes:
 * the structurizer joins the color's paths through a register. */
static const char *bypassed =
   "define <4 x float> @bypassed(<4 x float> %previous, <4 x float> %position) {\n"
   "entry:\n%x = extractelement <4 x float> %position, i64 0\n%left = fcmp olt float %x, 1.0\n"
   "br i1 %left, label %twice, label %row\nrow:\n"
   "%y = extractelement <4 x float> %position, i64 1\n%top = fcmp olt float %y, 1.0\n"
   "br i1 %top, label %twice, label %done\ntwice:\n"
   "%doubled = fmul <4 x float> %previous, <float 2.0, float 2.0, float 2.0, float 2.0>\nbr label %done\n"
   "done:\n%r = phi <4 x float> [ %doubled, %twice ], [ %previous, %row ]\n"
   "ret <4 x float> %r\n}\n"
   "!air.fragment = !{!2}\n"
   "!2 = !{ptr @bypassed, !3, !4}\n!3 = !{!30}\n"
   "!30 = !{!\"air.render_target\", i32 0, i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"color\"}\n"
   "!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.render_target\", i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"previous\"}\n"
   "!41 = !{i32 1, !\"air.position\", !\"air.center\", !\"air.no_perspective\","
   " !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"position\"}\n";

/* Reads its barycentric coordinates and interpolates its uv at the
 * centroid. */
static const char *pulled =
   "declare <2 x float> @air.interpolate_centroid_perspective.v2f32(ptr addrspace(1))\n"
   "define <4 x float> @pulled(ptr addrspace(1) %uv, <3 x float> %bary) {\n"
   "%c = call <2 x float> @air.interpolate_centroid_perspective.v2f32(ptr addrspace(1) %uv)\n"
   "%x = extractelement <2 x float> %c, i64 0\n"
   "%v = insertelement <3 x float> %bary, float %x, i64 0\n"
   "%r = shufflevector <3 x float> %v, <3 x float> %v, <4 x i32> <i32 0, i32 1, i32 2, i32 0>\n"
   "ret <4 x float> %r\n}\n"
   "!air.fragment = !{!2}\n"
   "!2 = !{ptr @pulled, !3, !4}\n!3 = !{!30}\n"
   "!30 = !{!\"air.render_target\", i32 0, i32 0, !\"air.arg_type_name\", !\"float4\","
   " !\"air.arg_name\", !\"color\"}\n"
   "!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.fragment_input\", !\"user(uv)\", !\"air.interpolation_function\","
   " !\"air.perspective\", !\"air.arg_type_name\", !\"float2\", !\"air.arg_name\", !\"uv\"}\n"
   "!41 = !{i32 1, !\"air.barycentric_coord\", !\"air.center\", !\"air.perspective\","
   " !\"air.arg_type_name\", !\"float3\", !\"air.arg_name\", !\"bary\"}\n";

static const char *object =
   "declare void @air.set_threadgroups_per_grid_mesh_properties(ptr addrspace(3), <3 x i32>)\n"
   "define void @object(ptr addrspace(6) %payload, ptr addrspace(3) %grid) {\n"
   "store i32 7, ptr addrspace(6) %payload, align 4\n"
   "call void @air.set_threadgroups_per_grid_mesh_properties(ptr addrspace(3) %grid,"
   " <3 x i32> <i32 1, i32 1, i32 1>)\n"
   "ret void\n}\n"
   "!air.object = !{!2}\n"
   "!2 = !{ptr @object, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.payload\", !\"air.arg_type_size\", i32 16, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"mtl_payload\"}\n"
   "!41 = !{i32 1, !\"air.mesh_grid_properties\", !\"air.arg_type_name\", !\"mesh_grid_properties\","
   " !\"air.arg_name\", !\"grid\"}\n";

static const char *mesh =
   "declare void @air.set_position_mesh(ptr addrspace(7), i32, <4 x float>)\n"
   "declare void @air.set_vertex_data_mesh.v2f32(ptr addrspace(7), i32, i32, <2 x float>)\n"
   "declare void @air.set_primitive_count_mesh(ptr addrspace(7), i32)\n"
   "define void @mesh(ptr addrspace(7) %m, ptr addrspace(6) %payload, i32 %tid) {\n"
   "%s = load i32, ptr addrspace(6) %payload, align 4\n"
   "call void @air.set_position_mesh(ptr addrspace(7) %m, i32 %tid, <4 x float> zeroinitializer)\n"
   "call void @air.set_vertex_data_mesh.v2f32(ptr addrspace(7) %m, i32 0, i32 %tid,"
   " <2 x float> zeroinitializer)\n"
   "call void @air.set_primitive_count_mesh(ptr addrspace(7) %m, i32 %s)\n"
   "ret void\n}\n"
   "!air.mesh = !{!2}\n"
   "!2 = !{ptr @mesh, !3, !4}\n!3 = !{}\n!4 = !{!40, !41, !42}\n"
   "!40 = !{i32 0, !\"air.mesh\", !50, !\"air.arg_type_name\", !\"mesh<>\", !\"air.arg_name\", !\"mtl_mesh\"}\n"
   "!50 = !{!\"air.mesh_type_info\", !51, !52, i32 3, i32 1, !\"air.triangle\"}\n"
   "!51 = !{!60, !61}\n!52 = !{}\n"
   "!60 = !{!\"air.position\", !\"air.arg_type_name\", !\"float4\", !\"air.arg_name\", !\"position\"}\n"
   "!61 = !{!\"air.mesh_vertex_data\", i32 0, !\"user(uv)\", !\"air.arg_type_name\", !\"float2\","
   " !\"air.arg_name\", !\"uv\"}\n"
   "!41 = !{i32 1, !\"air.payload\", !\"air.arg_type_size\", i32 16, !\"air.arg_type_align_size\", i32 4,"
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"mtl_payload\"}\n"
   "!42 = !{i32 2, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

static const struct stela_metal_target target = {
   .nir = &nir_options, .max_instructions = 10000, .max_threadgroup_bytes = 32768,
   .max_private_bytes = 4096, .max_constant_bytes = 65536, .max_varying_slots = 16,
   .max_payload_bytes = 16384, .max_mesh_vertices = 256, .max_mesh_primitives = 256,
};

/* A fixture's bitcode: its code and metadata, then the versions. */
/* fixture's module, with the version metadata of the compiler build that
 * fixed_versions names. */
static LLVMMemoryBufferRef
bitcode_built(const char *fixture, const char *fixed_versions)
{
   const char *metadata = strstr(fixture, "\n!air.") + 1;
   char *body = ralloc_strndup(NULL, fixture, metadata - fixture);
   char *tail = ralloc_asprintf(body, "%s%s", metadata, fixed_versions);
   LLVMMemoryBufferRef input = stela_air_test_custom_bitcode(module, body, tail, "");
   ralloc_free(body);
   return input;
}

static LLVMMemoryBufferRef
bitcode(const char *fixture)
{
   return bitcode_built(fixture, versions);
}

/* Reflects entry specialized with the function constant values, or
 * unspecialized with NULL values. */
static bool
reflect_specialized(const char *fixture, const char *entry,
                    const struct stela_air_constant_value *values, unsigned count,
                    struct stela_metal_reflection *reflection, struct stela_air_error *error)
{
   LLVMMemoryBufferRef input = bitcode(fixture);
   bool reflected = stela_metal_reflect(LLVMGetBufferStart(input), LLVMGetBufferSize(input), entry,
                                        values, count, reflection, error);
   if (!reflected)
      fprintf(stderr, "%s: %u %s\n", entry, error->kind, error->detail);
   LLVMDisposeMemoryBuffer(input);
   return reflected;
}

static bool
count_function(void *data, const char *name, size_t name_bytes,
               enum stela_metal_function_stage stage)
{
   (void)stage;
   *(unsigned *)data += name && name_bytes;
   return true;
}

/* The functions a fixture lists, zero when it cannot be listed. */
static unsigned
listed_functions(const char *fixture)
{
   LLVMMemoryBufferRef input = bitcode(fixture);
   unsigned count = 0;
   bool listed = stela_metal_functions(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                       count_function, &count, NULL);
   LLVMDisposeMemoryBuffer(input);
   return listed ? count : 0;
}

static bool
list_function(void *data, const char *name, size_t name_bytes,
              enum stela_metal_function_stage stage)
{
   (void)name_bytes;
   assert(!strcmp(name, "vertex") && stage == STELA_METAL_VERTEX);
   (*(unsigned *)data)++;
   return true;
}

static bool
reflect(const char *fixture, const char *entry, struct stela_metal_reflection *reflection,
        struct stela_air_error *error)
{
   return reflect_specialized(fixture, entry, NULL, 0, reflection, error);
}

static nir_shader *
compile(const char *fixture, const char *entry, const struct stela_metal_specialization *specialization,
        struct stela_air_error *error)
{
   LLVMMemoryBufferRef input = bitcode(fixture);
   nir_shader *shader = stela_metal_compile(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                            entry, &target, specialization, error);
   LLVMDisposeMemoryBuffer(input);
   return shader;
}

static const char *
name(const struct stela_metal_reflection *reflection, uint32_t offset, uint32_t bytes)
{
   static char text[STELA_METAL_NAME_BYTES + 1];
   memcpy(text, reflection->names + offset, bytes);
   text[bytes] = 0;
   return text;
}

/* How many ALU instructions of op shader holds. */
static unsigned
alu_count(nir_shader *shader, nir_op op)
{
   unsigned count = 0;
   nir_foreach_block(block, nir_shader_get_entrypoint(shader)) {
      nir_foreach_instr(instr, block)
         count += instr->type == nir_instr_type_alu && nir_instr_as_alu(instr)->op == op;
   }
   return count;
}

/* Image stores of any form: deref, bindless or bound. */
static unsigned
image_store_count(nir_shader *shader)
{
   unsigned count = 0;
   nir_foreach_function_impl(impl, shader) {
      nir_foreach_block(block, impl) {
         nir_foreach_instr(instr, block) {
            if (instr->type != nir_instr_type_intrinsic)
               continue;
            nir_intrinsic_op op = nir_instr_as_intrinsic(instr)->intrinsic;
            count += op == nir_intrinsic_image_deref_store || op == nir_intrinsic_bindless_image_store ||
                     op == nir_intrinsic_image_store;
         }
      }
   }
   return count;
}

/* An extended-range attachment's output is clamped to what its codes
 * encode, its alpha to [0, 1], and rounded to its nearest half; other
 * attachments' are left alone. */
static void
extended_range_outputs_clamp(void)
{
   struct stela_air_error error;
   const struct stela_metal_specialization plain = {0};
   nir_shader *shader = compile(positioned, "positioned", &plain, &error);
   assert(shader && !alu_count(shader, nir_op_fmin) && !alu_count(shader, nir_op_fsat) &&
          !alu_count(shader, nir_op_f2f16_rtne));
   unsigned stores = image_store_count(shader);
   stela_metal_release(shader);
   for (uint32_t kind = STELA_AIR_EXTENDED_RANGE_LINEAR; kind <= STELA_AIR_EXTENDED_RANGE_SRGB;
        kind++) {
      const struct stela_metal_specialization clamped = {.extended_range = kind};
      shader = compile(positioned, "positioned", &clamped, &error);
      if (!shader)
         fprintf(stderr, "positioned: %u %s\n", error.kind, error.detail);
      /* Each channel rounds to its nearest code and clamps to the codes. */
      assert(shader && alu_count(shader, nir_op_fround_even) == 4 && alu_count(shader, nir_op_fmin) == 4 &&
             alu_count(shader, nir_op_fmax) == 4);
      assert(alu_count(shader, nir_op_f2f16_rtne) == 4);
      stela_metal_release(shader);
   }
   /* The two-plane formats' blended outputs hold their codes and 8-bit
    * alpha from their halves; their unblended ones are only rounded to
    * halves. */
   for (uint32_t kind = STELA_AIR_EXTENDED_RANGE_PLANES_HELD; kind <= STELA_AIR_EXTENDED_RANGE_PLANES;
        kind++) {
      const struct stela_metal_specialization planes = {.extended_range = kind};
      shader = compile(positioned, "positioned", &planes, &error);
      bool held = kind != STELA_AIR_EXTENDED_RANGE_PLANES;
      assert(shader && alu_count(shader, nir_op_fround_even) == (held ? 4 : 0) &&
             alu_count(shader, nir_op_fmin) == (held ? 3 : 0) &&
             alu_count(shader, nir_op_fsat) == (held ? 1 : 0));
      assert(alu_count(shader, nir_op_f2f16_rtne) == (held ? 8 : 4));
      /* And mark the dirty tile they reach in the working image. */
      assert(image_store_count(shader) == stores + 1);
      stela_metal_release(shader);
   }
   /* Another location's kind leaves location 0 alone. */
   const struct stela_metal_specialization other = {
      .extended_range = STELA_AIR_EXTENDED_RANGE_LINEAR << STELA_AIR_EXTENDED_RANGE_BITS};
   shader = compile(positioned, "positioned", &other, &error);
   assert(shader && !alu_count(shader, nir_op_fmin));
   stela_metal_release(shader);
   /* Unknown kinds, locations past the eighth and other stages are refused. */
   const struct stela_metal_specialization unknown = {.extended_range = 6};
   assert(!compile(positioned, "positioned", &unknown, &error) && error.kind == STELA_AIR_INPUT);
   const struct stela_metal_specialization ninth = {.extended_range = 1u << 24};
   assert(!compile(positioned, "positioned", &ninth, &error) && error.kind == STELA_AIR_INPUT);
   const struct stela_metal_specialization unrendered = {.extended_range = 1};
   assert(!compile(vertex, "vertex", &unrendered, &error) && error.kind == STELA_AIR_INPUT);
   assert(!compile(kernel, "kernel", &unrendered, &error) && error.kind == STELA_AIR_INPUT);
}

static void
kernels_reflect_and_compile_their_threadgroup_memory(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(kernel, "kernel", &reflection, &error));
   assert(reflection.stage == STELA_METAL_KERNEL && reflection.max_total_threads == 64);
   assert(reflection.buffer_count == 1 && reflection.buffers[0].index == 2);
   assert(reflection.buffers[0].access == (STELA_METAL_ACCESS_READ | STELA_METAL_ACCESS_WRITE));
   assert(reflection.threadgroup_count == 1 && reflection.threadgroup[0].index == 0 &&
          reflection.threadgroup[0].bytes == 4);
   assert(reflection.threadgroup_bytes == 64);
   assert(!strcmp(name(&reflection, reflection.ident, reflection.ident_bytes), IDENT));

   const struct stela_metal_threadgroup_length lengths[] = {{0, 256}};
   struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .threadgroup_length_count = 1, .threadgroup_lengths = lengths,
   };
   nir_shader *shader = compile(kernel, "kernel", &specialization, &error);
   if (!shader)
      fprintf(stderr, "kernel: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.stage == MESA_SHADER_COMPUTE);
   assert(shader->info.shared_size >= 256 + 64);
   stela_metal_release(shader);

   /* Each threadgroup buffer needs its dispatch's length. */
   specialization.threadgroup_length_count = 0;
   assert(!compile(kernel, "kernel", &specialization, &error) && error.kind != STELA_AIR_OK);
}

static nir_variable *
varying(nir_shader *shader, nir_variable_mode mode, int location)
{
   nir_foreach_variable_with_modes(var, shader, mode) {
      if (var->data.location == location)
         return var;
   }
   return NULL;
}

/* A library of source has no container to list its functions. */
static void
modules_list_their_functions(void)
{
   LLVMMemoryBufferRef input = bitcode(vertex);
   unsigned count = 0;
   assert(stela_metal_functions(LLVMGetBufferStart(input), LLVMGetBufferSize(input), list_function,
                                &count, NULL));
   LLVMDisposeMemoryBuffer(input);
   assert(count == 1);
}

static bool
list_export(void *data, const char *name, size_t name_bytes, enum stela_metal_function_stage stage)
{
   (void)stage;
   assert(name_bytes == strlen("_Z3dynf") && !memcmp(name, "_Z3dynf", name_bytes));
   (*(unsigned *)data)++;
   return true;
}

/* A dynamic library exports its definitions of default visibility that are
 * not entry points, as Metal's compiler lists them. */
static void
modules_list_their_exports(void)
{
   char *fixture = ralloc_asprintf(NULL,
                                   "%s"
                                   "define float @_Z3dynf(float %%x) {\nret float %%x\n}\n"
                                   "define hidden float @_Z6hiddenf(float %%x) {\nret float %%x\n}\n"
                                   "define internal float @_Z5localf(float %%x) {\nret float %%x\n}\n",
                                   vertex);
   LLVMMemoryBufferRef input = bitcode(fixture);
   ralloc_free(fixture);
   unsigned count = 0;
   assert(stela_metal_exports(LLVMGetBufferStart(input), LLVMGetBufferSize(input), list_export,
                              &count, NULL));
   LLVMDisposeMemoryBuffer(input);
   assert(count == 1);
}

static void
discovery_reports_malformed_metadata(void)
{
   struct stela_air_error error;
   unsigned listed = 0;
   const char header[] = "BC\xc0\xde";
   assert(!stela_metal_functions(header, sizeof(header) - 1, count_function, &listed, &error));
   assert(error.kind == STELA_AIR_BITCODE && !listed);
   uint64_t states[STELA_METAL_SAMPLER_STATES];
   uint32_t count = 1;
   assert(!stela_metal_sampler_states(header, sizeof(header) - 1, states, &count, &error));
   assert(error.kind == STELA_AIR_BITCODE && !count);

   char *fixture = ralloc_asprintf(NULL, "%s!air.visible = !{!90}\n!90 = !{null}\n", kernel);
   LLVMMemoryBufferRef input = bitcode(fixture);
   ralloc_free(fixture);
   assert(!stela_metal_functions(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                 count_function, &listed, &error));
   assert(error.kind == STELA_AIR_METADATA);
   LLVMDisposeMemoryBuffer(input);
}

static void
reflection_requires_a_unique_listed_entry(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   char *fixture = ralloc_asprintf(NULL, "%s!air.visible = !{!2}\n", kernel);
   LLVMMemoryBufferRef input = bitcode(fixture);
   ralloc_free(fixture);
   assert(!stela_metal_reflect(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                               "kernel", NULL, 0, &reflection, &error));
   assert(error.kind == STELA_AIR_METADATA && strstr(error.detail, "entry listed more than once"));
   LLVMDisposeMemoryBuffer(input);

   input = bitcode(kernel);
   assert(!stela_metal_reflect(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                               "air.wg.barrier", NULL, 0, &reflection, &error));
   assert(error.kind == STELA_AIR_ARGUMENT);
   LLVMDisposeMemoryBuffer(input);
}

static void
render_stages_compile_at_their_linked_locations(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(vertex, "vertex", &reflection, &error));
   assert(reflection.stage == STELA_METAL_VERTEX && reflection.varying_count == 1);
   assert(reflection.attribute_count == 1 && reflection.attributes[0].index == 3 &&
          reflection.attributes[0].components == 4 && !reflection.attributes[0].patch_control_point &&
          reflection.attributes[0].active);
   assert(!strcmp(name(&reflection, reflection.attributes[0].name,
                       reflection.attributes[0].name_bytes), "p"));
   const struct stela_metal_varying *uv = &reflection.varyings[0];
   assert(!strcmp(name(&reflection, uv->identifier, uv->identifier_bytes), "user(uv)"));
   assert(uv->primitive == STELA_AIR_FLOAT && uv->components == 2 && !uv->per_primitive);

   assert(reflect(fragment, "fragment", &reflection, &error));
   assert(reflection.stage == STELA_METAL_FRAGMENT && reflection.discards);
   assert(reflection.varying_count == 1 && reflection.color_output_count == 1);
   assert(reflection.color_outputs[0].index == 1 && !reflection.color_outputs[0].dual_source);

   /* user(uv) is the second linked varying. */
   const struct stela_metal_linked_varying linked[] = {
      {"user(color)", 11, 0}, {"user(uv)", 8, 0},
   };
   const struct stela_metal_specialization specialization = {.varyings = linked, .varying_count = 2};
   nir_shader *shaders[2] = {
      compile(vertex, "vertex", &specialization, &error),
      compile(fragment, "fragment", &specialization, &error),
   };
   assert(shaders[0] && shaders[1]);
   assert(varying(shaders[0], nir_var_shader_out, VARYING_SLOT_VAR0 + 1));
   assert(varying(shaders[1], nir_var_shader_in, VARYING_SLOT_VAR0 + 1));
   for (unsigned i = 0; i < 2; i++)
      stela_metal_release(shaders[i]);

   assert(!reflection.barycentrics && !reflection.interpolants);
   /* Interpolants link as varyings. */
   assert(reflect(pulled, "pulled", &reflection, &error));
   assert(reflection.barycentrics && reflection.interpolants && reflection.varying_count == 1);
   nir_shader *pulling = compile(pulled, "pulled", &specialization, &error);
   assert(pulling && varying(pulling, nir_var_shader_in, VARYING_SLOT_VAR0 + 1));
   stela_metal_release(pulling);

   /* An input the function never reads is reflected inactive and not fetched. */
   assert(reflect(unread, "unread", &reflection, &error));
   assert(reflection.attribute_count == 2 && reflection.attributes[0].active &&
          !reflection.attributes[1].active);
   nir_shader *reading = compile(unread, "unread", &specialization, &error);
   assert(reading && varying(reading, nir_var_shader_in, VERT_ATTRIB_GENERIC0) &&
          !varying(reading, nir_var_shader_in, VERT_ATTRIB_GENERIC0 + 1));
   stela_metal_release(reading);

   /* A varying the pipeline did not link is refused. */
   const struct stela_metal_specialization unlinked = {.varyings = linked, .varying_count = 1};
   assert(!compile(fragment, "fragment", &unlinked, &error) && error.kind == STELA_AIR_ARGUMENT);
   /* As are entries no AIR list names. */
   assert(!reflect(vertex, "missing", &reflection, &error) && error.kind == STELA_AIR_ARGUMENT);
   /* A function reading its render target says so, for its pipeline's draws. */
   assert(reflect(blended, "blended", &reflection, &error));
   assert(reflection.attachment_reads == 1u);
}

/* The intrinsics of shader, in order. */
static unsigned
intrinsics(nir_shader *shader, nir_intrinsic_op *ops, unsigned capacity)
{
   unsigned count = 0;
   nir_foreach_block(block, nir_shader_get_entrypoint(shader)) {
      nir_foreach_instr(instr, block) {
         if (instr->type != nir_instr_type_intrinsic)
            continue;
         nir_intrinsic_op op = nir_instr_as_intrinsic(instr)->intrinsic;
         if (op == nir_intrinsic_begin_invocation_interlock ||
             op == nir_intrinsic_end_invocation_interlock || op == nir_intrinsic_image_deref_load ||
             op == nir_intrinsic_image_deref_store) {
            assert(count < capacity);
            ops[count++] = op;
         }
      }
   }
   return count;
}

/* Whether shader computes anything from an undefined value. */
static bool
undefined(nir_shader *shader)
{
   nir_foreach_block(block, nir_shader_get_entrypoint(shader)) {
      nir_foreach_instr(instr, block) {
         if (instr->type == nir_instr_type_undef && !nir_def_is_unused(&nir_instr_as_undef(instr)->def))
            return true;
      }
   }
   return false;
}

/* A function reading its render target accesses it as storage in its
 * interlock section: it loads the texel, blends its output over it and
 * stores it, and writes no color output for the blend unit. */
static void
framebuffer_reads_access_storage_attachments(void)
{
   struct stela_air_error error;
   const struct stela_metal_target interlocked = {
      .nir = &nir_options, .max_instructions = 10000, .max_threadgroup_bytes = 32768,
      .max_private_bytes = 4096, .max_constant_bytes = 65536, .max_varying_slots = 16,
      .raster_order_groups = 1,
   };
   LLVMMemoryBufferRef input = bitcode(blended);
   const void *air = LLVMGetBufferStart(input);
   size_t size = LLVMGetBufferSize(input);
   /* A pipeline without the attachment reads zero, outside any interlock. */
   const struct stela_metal_specialization plain = {0};
   nir_shader *absent = stela_metal_compile(air, size, "blended", &interlocked, &plain, &error);
   assert(absent && !absent->info.fs.pixel_interlock_ordered);
   stela_metal_release(absent);
   /* Source-over blending of an sRGB attachment. */
   struct stela_metal_attachment attachments[8] = {{
      .format = VK_FORMAT_B8G8R8A8_SRGB, .bytes = 4, .blend = 1, .write_mask = 0xf,
      .source_rgb = STELA_AIR_BLEND_SOURCE_ALPHA,
      .destination_rgb = STELA_AIR_BLEND_ONE_MINUS_SOURCE_ALPHA,
      .source_alpha = STELA_AIR_BLEND_ONE,
      .destination_alpha = STELA_AIR_BLEND_ONE_MINUS_SOURCE_ALPHA,
   }};
   struct stela_metal_specialization storage = {
      .attachments = attachments, .attachment_count = 8, .samples = 1};
   /* A device without interlock cannot order them. */
   assert(!stela_metal_compile(air, size, "blended", &target, &storage, &error) &&
          error.kind == STELA_AIR_TARGET);
   nir_shader *shader = stela_metal_compile(air, size, "blended", &interlocked, &storage, &error);
   if (!shader)
      fprintf(stderr, "blended: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.fs.pixel_interlock_ordered && shader->info.fs.early_fragment_tests);
   nir_intrinsic_op ops[8];
   unsigned count = intrinsics(shader, ops, ARRAY_SIZE(ops));
   assert(count == 4 && ops[0] == nir_intrinsic_begin_invocation_interlock &&
          ops[1] == nir_intrinsic_image_deref_load && ops[2] == nir_intrinsic_image_deref_store &&
          ops[3] == nir_intrinsic_end_invocation_interlock);
   nir_foreach_variable_with_modes(var, shader, nir_var_shader_out)
      assert(var->data.location != FRAG_RESULT_DATA0);
   assert(!shader->info.fs.uses_sample_shading);
   stela_metal_release(shader);
   /* Multisampled, it runs per sample and orders each sample's fragments. */
   storage.samples = 4;
   shader = stela_metal_compile(air, size, "blended", &interlocked, &storage, &error);
   assert(shader && shader->info.fs.uses_sample_shading);
   assert(shader->info.fs.sample_interlock_ordered && !shader->info.fs.pixel_interlock_ordered);
   stela_metal_release(shader);
   storage.samples = 1;
   /* A masked-off attachment is only read. */
   attachments[0].write_mask = 0;
   shader = stela_metal_compile(air, size, "blended", &interlocked, &storage, &error);
   count = intrinsics(shader, ops, ARRAY_SIZE(ops));
   assert(shader && count == 3 && ops[1] == nir_intrinsic_image_deref_load);
   stela_metal_release(shader);
   attachments[0].write_mask = 0xf;
   /* The texel bytes must be the format's, and the format renderable. */
   attachments[0].bytes = 8;
   assert(!stela_metal_compile(air, size, "blended", &interlocked, &storage, &error) &&
          error.kind == STELA_AIR_INPUT);
   attachments[0] = (struct stela_metal_attachment){.format = VK_FORMAT_BC1_RGB_UNORM_BLOCK, .bytes = 8};
   assert(!stela_metal_compile(air, size, "blended", &interlocked, &storage, &error) &&
          error.kind == STELA_AIR_INPUT);
   LLVMDisposeMemoryBuffer(input);
   /* A read only a phi uses is loaded before the branch leading to it. */
   attachments[0] = (struct stela_metal_attachment){
      .format = VK_FORMAT_R16G16B16A16_SFLOAT, .bytes = 8, .write_mask = 0xf};
   input = bitcode(selected);
   shader = stela_metal_compile(LLVMGetBufferStart(input), LLVMGetBufferSize(input), "selected",
                                &interlocked, &storage, &error);
   if (!shader)
      fprintf(stderr, "selected: %u %s\n", error.kind, error.detail);
   count = intrinsics(shader, ops, ARRAY_SIZE(ops));
   assert(shader && count == 4 && ops[1] == nir_intrinsic_image_deref_load);
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(input);
   /* Every path to a use of a read carries its texel, including those the
    * structurizer joins. */
   input = bitcode(bypassed);
   shader = stela_metal_compile(LLVMGetBufferStart(input), LLVMGetBufferSize(input), "bypassed",
                                &interlocked, &storage, &error);
   if (!shader)
      fprintf(stderr, "bypassed: %u %s\n", error.kind, error.detail);
   assert(shader && !undefined(shader));
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(input);
}

/* Whether shader writes the fragment output at location. */
static bool
writes_output(nir_shader *shader, int location)
{
   nir_foreach_variable_with_modes(var, shader, nir_var_shader_out) {
      if (var->data.location == location)
         return true;
   }
   return false;
}

/* A function reading its render target that also writes depth, or whose
 * pipeline turns alpha into coverage, cannot be tested before it runs: it
 * runs the fragment tests itself, in the same interlock section as its
 * storage accesses, on the pass's depth and stencil copies, then writes the
 * attachment for the samples that pass, and its coverage is theirs. */
static void
framebuffer_reads_run_their_own_fragment_tests(void)
{
   struct stela_air_error error;
   const struct stela_metal_target interlocked = {
      .nir = &nir_options, .max_instructions = 10000, .max_threadgroup_bytes = 32768,
      .max_private_bytes = 4096, .max_constant_bytes = 65536, .max_varying_slots = 16,
      .raster_order_groups = 1, .fragment_tests = 1,
   };
   struct stela_metal_attachment attachments[8] = {{
      .format = VK_FORMAT_R8G8B8A8_UNORM, .bytes = 4, .write_mask = 0xf}};
   struct stela_metal_specialization storage = {
      .attachments = attachments, .attachment_count = 8, .samples = 1};
   LLVMMemoryBufferRef input = bitcode(deepened);
   const void *air = LLVMGetBufferStart(input);
   size_t size = LLVMGetBufferSize(input);
   /* Tested first, its depth output refuses. */
   assert(!stela_metal_compile(air, size, "deepened", &interlocked, &storage, &error) &&
          error.kind == STELA_AIR_ARGUMENT);
   storage.fragment_tests = STELA_METAL_SELF_TESTS;
   /* A device without fragment tests of its own cannot run them. */
   const struct stela_metal_target untested = {
      .nir = &nir_options, .max_instructions = 10000, .max_threadgroup_bytes = 32768,
      .max_private_bytes = 4096, .max_constant_bytes = 65536, .max_varying_slots = 16,
      .raster_order_groups = 1,
   };
   assert(!stela_metal_compile(air, size, "deepened", &untested, &storage, &error) &&
          error.kind == STELA_AIR_TARGET);
   nir_shader *shader = stela_metal_compile(air, size, "deepened", &interlocked, &storage, &error);
   if (!shader)
      fprintf(stderr, "deepened: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.fs.pixel_interlock_ordered && !shader->info.fs.early_fragment_tests);
   /* One section: the attachment's texel, then the depth and stencil
    * copies' tests and updates, then the attachment's write. */
   nir_intrinsic_op ops[16];
   unsigned count = intrinsics(shader, ops, ARRAY_SIZE(ops));
   assert(count >= 6 && ops[0] == nir_intrinsic_begin_invocation_interlock &&
          ops[1] == nir_intrinsic_image_deref_load && ops[count - 1] == nir_intrinsic_end_invocation_interlock &&
          ops[count - 2] == nir_intrinsic_image_deref_store);
   unsigned begins = 0;
   for (unsigned i = 0; i < count; i++)
      begins += ops[i] == nir_intrinsic_begin_invocation_interlock;
   assert(begins == 1);
   /* The pipeline writes no depth: the coverage of the samples that pass. */
   assert(!writes_output(shader, FRAG_RESULT_DEPTH) && writes_output(shader, FRAG_RESULT_SAMPLE_MASK));
   assert(!writes_output(shader, FRAG_RESULT_DATA0));
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(input);
   /* Alpha to coverage of a function reading its render target: its alpha
    * keeps samples before the tests. */
   input = bitcode(blended);
   storage.fragment_tests = STELA_METAL_SELF_TESTS | STELA_METAL_ALPHA_TO_COVERAGE | STELA_METAL_ALPHA_TO_ONE;
   storage.samples = 4;
   shader = stela_metal_compile(LLVMGetBufferStart(input), LLVMGetBufferSize(input), "blended",
                                &interlocked, &storage, &error);
   if (!shader)
      fprintf(stderr, "blended alpha to coverage: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.fs.sample_interlock_ordered && writes_output(shader, FRAG_RESULT_SAMPLE_MASK));
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(input);
   /* A storage attachment blending a second source takes it from the
    * function, which then writes neither source for the blend unit. */
   input = bitcode(dual);
   storage.fragment_tests = 0;
   storage.samples = 1;
   attachments[0].blend = 1;
   attachments[0].source_rgb = STELA_AIR_BLEND_SOURCE1_COLOR;
   attachments[0].destination_rgb = STELA_AIR_BLEND_ONE_MINUS_SOURCE1_ALPHA;
   attachments[0].source_alpha = STELA_AIR_BLEND_ONE;
   attachments[0].destination_alpha = STELA_AIR_BLEND_ZERO;
   shader = stela_metal_compile(LLVMGetBufferStart(input), LLVMGetBufferSize(input), "dual",
                                &interlocked, &storage, &error);
   if (!shader)
      fprintf(stderr, "dual: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.fs.early_fragment_tests && !writes_output(shader, FRAG_RESULT_DATA0));
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(input);
}

static void
patch_functions_compile_behind_their_tessellation_stages(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(patch, "patch", &reflection, &error));
   assert(reflection.stage == STELA_METAL_VERTEX && reflection.patch == STELA_AIR_PATCH_TRIANGLE &&
          reflection.patch_control_points == 3);
   /* The control points' members, of which the function reads the first. */
   assert(reflection.attribute_count == 2 && reflection.attributes[0].index == 1 &&
          reflection.attributes[0].components == 4 && reflection.attributes[0].patch_control_point &&
          reflection.attributes[0].active);
   assert(reflection.attributes[1].index == 2 && !reflection.attributes[1].active);
   assert(!strcmp(name(&reflection, reflection.attributes[1].name,
                       reflection.attributes[1].name_bytes), "uv"));

   const struct stela_metal_specialization plain = {0};
   assert(!compile(patch, "patch", &plain, &error) && error.kind == STELA_AIR_ARGUMENT);
   struct stela_metal_tessellation tessellation = {
      .partition = STELA_METAL_PARTITION_INTEGER, .max_factor = 16};
   struct stela_metal_stage_in stage_in = {0};
   stage_in.attributes[1] = (struct stela_metal_stage_in_attribute){31, 0, 4};
   stage_in.layouts[4] = (struct stela_metal_stage_in_layout){
      16, STELA_METAL_STEP_PER_PATCH_CONTROL_POINT, 1};
   const struct stela_metal_specialization specialization = {
      .tessellation = &tessellation, .stage_in = &stage_in};
   nir_shader *shader = compile(patch, "patch", &specialization, &error);
   if (!shader)
      fprintf(stderr, "patch: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.stage == MESA_SHADER_TESS_EVAL);
   assert(shader->info.tess._primitive_mode == TESS_PRIMITIVE_TRIANGLES);
   stela_metal_release(shader);

   /* The pipeline tessellates the function's patch, from a layout with its
    * attributes; no other function tessellates. */
   tessellation.quad = true;
   assert(!compile(patch, "patch", &specialization, &error) && error.kind == STELA_AIR_INPUT);
   tessellation.quad = false;
   stage_in.attributes[1].format = 0;
   assert(!compile(patch, "patch", &specialization, &error) && error.kind == STELA_AIR_ARGUMENT);
   assert(!compile(vertex, "vertex", &specialization, &error) && error.kind == STELA_AIR_INPUT);

   nir_shader *stages[2];
   stela_metal_tessellation_stages(&target, &tessellation, stages);
   assert(stages[0]->info.stage == MESA_SHADER_VERTEX);
   assert(stages[1]->info.stage == MESA_SHADER_TESS_CTRL);
   for (unsigned i = 0; i < 2; i++)
      stela_metal_release(stages[i]);
}

static void
kernels_compile_with_their_stage_in(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(staged, "staged", &reflection, &error));
   /* Both attributes, of which the kernel reads the first. */
   assert(reflection.stage == STELA_METAL_KERNEL && reflection.attribute_count == 2 &&
          reflection.attributes[0].index == 0 && reflection.attributes[0].components == 4 &&
          reflection.attributes[0].active);
   assert(reflection.attributes[1].index == 1 && !reflection.attributes[1].active);
   assert(!strcmp(name(&reflection, reflection.attributes[1].name,
                       reflection.attributes[1].name_bytes), "index"));

   struct stela_metal_stage_in stage_in = {0};
   stage_in.attributes[0] = (struct stela_metal_stage_in_attribute){31, 0, 2};
   stage_in.layouts[2] = (struct stela_metal_stage_in_layout){
      16, STELA_METAL_STEP_THREAD_POSITION_IN_GRID_X, 1};
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .stage_in = &stage_in};
   nir_shader *shader = compile(staged, "staged", &specialization, &error);
   if (!shader)
      fprintf(stderr, "staged: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.stage == MESA_SHADER_COMPUTE);
   stela_metal_release(shader);

   /* The layout provides what the kernel reads, and only kernels and patch
    * functions fetch theirs. */
   const struct stela_metal_specialization plain = {.workgroup_size = {64, 1, 1}};
   assert(!compile(staged, "staged", &plain, &error) && error.kind == STELA_AIR_ARGUMENT);
   const struct stela_metal_specialization fetched = {.stage_in = &stage_in};
   assert(!compile(vertex, "vertex", &fetched, &error) && error.kind == STELA_AIR_INPUT);
}

/* A vertex input in a format the device's vertex input lacks arrives as its
 * words, which the vertex function decodes; no other function, and no format
 * of no whole words, takes them. */
static void
vertex_inputs_decode_their_words(void)
{
   struct stela_air_error error = {0};
   const struct stela_metal_linked_varying linked[] = {{"user(uv)", 8, 0}};
   /* MTLVertexFormatFloatRGB9E5 at the function's location 3. */
   struct stela_metal_specialization specialization = {.varyings = linked, .varying_count = 1};
   specialization.vertex_formats[3] = 55;
   nir_shader *shader = compile(vertex, "vertex", &specialization, &error);
   if (!shader)
      fprintf(stderr, "vertex words: %u %s\n", error.kind, error.detail);
   assert(shader);
   nir_variable *words = varying(shader, nir_var_shader_in, VERT_ATTRIB_GENERIC0 + 3);
   assert(words && glsl_get_base_type(words->type) == GLSL_TYPE_UINT &&
          glsl_get_vector_elements(words->type) == 1);
   stela_metal_release(shader);

   assert(!compile(fragment, "fragment", &specialization, &error) && error.kind == STELA_AIR_INPUT);
   /* MTLVertexFormatUChar3 is three bytes. */
   specialization.vertex_formats[3] = 2;
   assert(!compile(vertex, "vertex", &specialization, &error) && error.kind == STELA_AIR_INPUT);
}

static void
mesh_stages_compile_with_their_launch(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(object, "object", &reflection, &error));
   assert(reflection.stage == STELA_METAL_OBJECT && !reflection.buffer_count);
   assert(reflect(mesh, "mesh", &reflection, &error));
   assert(reflection.stage == STELA_METAL_MESH && reflection.varying_count == 1);

   const struct stela_metal_specialization launch = {
      .workgroup_size = {32, 1, 1}, .launched_by_grid = 1,
   };
   nir_shader *shader = compile(object, "object", &launch, &error);
   if (!shader)
      fprintf(stderr, "object: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.stage == MESA_SHADER_TASK);
   stela_metal_release(shader);

   const struct stela_metal_linked_varying linked[] = {{"user(uv)", 8, 0}};
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {32, 1, 1}, .varyings = linked, .varying_count = 1,
   };
   shader = compile(mesh, "mesh", &specialization, &error);
   if (!shader)
      fprintf(stderr, "mesh: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.stage == MESA_SHADER_MESH);
   assert(varying(shader, nir_var_shader_out, VARYING_SLOT_VAR0));
   stela_metal_release(shader);
}

static void
function_constants_reflect_and_specialize(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(constants, "constants", &reflection, &error));
   assert(reflection.function_constant_count == 2);
   const struct stela_metal_function_constant *scale = &reflection.function_constants[0];
   const struct stela_metal_function_constant *enabled = &reflection.function_constants[1];
   assert(!strcmp(name(&reflection, scale->name, scale->name_bytes), "scale"));
   assert(!strcmp(name(&reflection, scale->type, scale->type_bytes), "uint"));
   assert(scale->index == 0 && !scale->required);
   assert(!strcmp(name(&reflection, enabled->type, enabled->type_bytes), "bool"));
   assert(enabled->index == 3 && enabled->required);

   const uint32_t seven = 7;
   const uint8_t on = 1;
   const struct stela_air_constant_value values[] = {{0, &seven, 4}, {3, &on, 1}, {9, &on, 1}};
   struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .constants = values, .constant_count = 3,
   };
   nir_shader *shader = compile(constants, "constants", &specialization, &error);
   if (!shader)
      fprintf(stderr, "constants: %u %s\n", error.kind, error.detail);
   /* Every read folds to its value; none is left in constant data. */
   assert(shader && shader->constant_data_size == 0);
   assert(reflect_specialized(constants, "constants", values, 3, &reflection, &error));
   LLVMMemoryBufferRef input = bitcode(constants);
   nir_shader *prepared = stela_metal_compile_reflected(
      LLVMGetBufferStart(input), LLVMGetBufferSize(input), "constants",
      &target, &specialization, &reflection, &error);
   LLVMDisposeMemoryBuffer(input);
   assert(prepared);
   struct blob normal, reused;
   blob_init(&normal);
   blob_init(&reused);
   nir_serialize(&normal, shader, true);
   nir_serialize(&reused, prepared, true);
   assert(!normal.out_of_memory && !reused.out_of_memory);
   assert(normal.size == reused.size && !memcmp(normal.data, reused.data, normal.size));
   blob_finish(&normal);
   blob_finish(&reused);
   stela_metal_release(prepared);
   stela_metal_release(shader);

   /* scale is optional, and its definedness test folds to false. */
   specialization.constants = &values[1];
   specialization.constant_count = 1;
   shader = compile(constants, "constants", &specialization, &error);
   assert(shader);
   stela_metal_release(shader);

   /* enabled is required. */
   specialization.constants = values;
   specialization.constant_count = 1;
   assert(!compile(constants, "constants", &specialization, &error) &&
          error.kind == STELA_AIR_ARGUMENT);
   /* Values take their constant's size, once each. */
   const struct stela_air_constant_value wide[] = {{3, &seven, 4}};
   specialization.constants = wide;
   assert(!compile(constants, "constants", &specialization, &error) && error.kind == STELA_AIR_INPUT);
   const struct stela_air_constant_value twice[] = {{3, &on, 1}, {3, &on, 1}};
   specialization.constants = twice;
   specialization.constant_count = 2;
   assert(!compile(constants, "constants", &specialization, &error) && error.kind == STELA_AIR_INPUT);
}

/* Whether the shader stores the constant value. */
static bool
stores_constant(nir_shader *shader, uint32_t value)
{
   nir_opt_constant_folding(shader);
   nir_foreach_function_impl(impl, shader) {
      nir_foreach_block(block, impl) {
         nir_foreach_instr(instr, block) {
            if (instr->type != nir_instr_type_intrinsic)
               continue;
            nir_intrinsic_instr *store = nir_instr_as_intrinsic(instr);
            if (store->intrinsic == nir_intrinsic_store_global &&
                nir_src_is_const(store->src[0]) && nir_src_as_uint(store->src[0]) == value)
               return true;
         }
      }
   }
   return false;
}

static void
derived_constants_are_constructed_from_their_function_constants(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(derived, "derived", &reflection, &error));
   assert(reflection.function_constant_count == 1);

   const int32_t five = 5;
   const struct stela_air_constant_value values[] = {{0, &five, 4}};
   struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .constants = values, .constant_count = 1,
   };
   nir_shader *shader = compile(derived, "derived", &specialization, &error);
   if (!shader)
      fprintf(stderr, "derived: %u %s\n", error.kind, error.detail);
   /* Its value folds into the store; none is left in constant data. */
   assert(shader && stores_constant(shader, 10) && !shader->constant_data_size);
   stela_metal_release(shader);

   shader = compile(converted, "derived", &specialization, &error);
   assert(shader && stores_constant(shader, 10));
   stela_metal_release(shader);

   assert(reflect_specialized(derived_with_helper, "derived", values, 1, &reflection, &error));
   shader = compile(derived_with_helper, "derived", &specialization, &error);
   assert(shader && stores_constant(shader, 10));
   stela_metal_release(shader);

   /* Intrinsics fold before evaluation: min(5, 4). */
   shader = compile(bounded, "derived", &specialization, &error);
   assert(shader && stores_constant(shader, 4));
   stela_metal_release(shader);

   /* An i1 global is a byte: 5 < 8 holds. */
   shader = compile(narrowed, "narrowed", &specialization, &error);
   if (!shader)
      fprintf(stderr, "narrowed: %u %s\n", error.kind, error.detail);
   assert(shader && stores_constant(shader, 7));
   stela_metal_release(shader);

   /* Without width, its definedness test selects eight. */
   specialization.constant_count = 0;
   shader = compile(derived, "derived", &specialization, &error);
   assert(shader && stores_constant(shader, 8));
   stela_metal_release(shader);

   /* Constructors are evaluated, never run. */
   assert(reflect(called, "derived", &reflection, &error));
   assert(!compile(called, "derived", &specialization, &error) && error.kind == STELA_AIR_INSTRUCTION &&
          strstr(error.detail, "program-scope constants"));

   /* A module may construct more globals than a function has arguments:
    * VideoToolbox's transfer kernel constructs 525. */
   enum { SPARE = 300 };
   const char *ctors = strstr(derived, "@llvm.global_ctors"), *end = strstr(derived, "ret void\n}\n!air");
   size_t bytes = strlen(derived) + SPARE * 128;
   char *many = malloc(bytes), *at = many;
   assert(many && ctors && end);
   at += sprintf(at, "%.*s", (int)(ctors - derived), derived);
   for (unsigned i = 0; i < SPARE; i++)
      at += sprintf(at, "@spare%u = internal unnamed_addr addrspace(2) global i32 0, align 4\n", i);
   at += sprintf(at, "%.*s", (int)(end - ctors), ctors);
   for (unsigned i = 0; i < SPARE; i++)
      at += sprintf(at, "store i32 %%v, ptr addrspace(2) @spare%u, align 4\n", i);
   sprintf(at, "%s", end);
   specialization.constant_count = 1;
   shader = compile(many, "derived", &specialization, &error);
   assert(shader && stores_constant(shader, 10));
   stela_metal_release(shader);
   free(many);
}

/* Apple's libraries construct hundreds of program-scope constants; their
 * count is bounded by the module, not by a fixed table. */
static void
constructed_globals_are_not_capped(void)
{
   char *fixture = ralloc_strdup(NULL, derived);
   for (unsigned i = 0; i < 300; i++)
      assert(ralloc_asprintf_append(&fixture, "@extra%u = internal addrspace(2) global i32 %u\n", i, i));
   LLVMMemoryBufferRef input = bitcode(fixture);
   ralloc_free(fixture);
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   const struct stela_air_constant_value unused = {0};
   assert(stela_metal_reflect(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                              "derived", &unused, 0, &reflection, &error));
   LLVMDisposeMemoryBuffer(input);
}

static void
constructor_priorities_are_integer_literals(void)
{
   const char *priority = strstr(derived, "65535");
   assert(priority);
   const char *invalid[] = {"undef", "poison"};
   for (unsigned i = 0; i < ARRAY_SIZE(invalid); i++) {
      char *fixture = ralloc_asprintf(NULL, "%.*s%s%s", (int)(priority - derived), derived,
                                      invalid[i], priority + strlen("65535"));
      LLVMMemoryBufferRef input = bitcode(fixture);
      ralloc_free(fixture);
      struct stela_metal_reflection reflection;
      struct stela_air_error error;
      assert(!stela_metal_reflect(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                  "derived", NULL, 0, &reflection, &error));
      assert(error.kind == STELA_AIR_METADATA && strstr(error.detail, "invalid AIR constructor list"));
      LLVMDisposeMemoryBuffer(input);
   }
}

static unsigned
count_intrinsic(nir_shader *shader, nir_intrinsic_op op)
{
   unsigned count = 0;
   nir_foreach_function_impl(impl, shader) {
      nir_foreach_block(block, impl) {
         nir_foreach_instr(instr, block) {
            count += instr->type == nir_instr_type_intrinsic &&
                     nir_instr_as_intrinsic(instr)->intrinsic == op;
         }
      }
   }
   return count;
}

/* Arguments a false function constant makes absent: reflection leaves
 * them out, folding removes the paths that read them, and another argument
 * takes their index. */
static void
function_constants_select_arguments(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   /* Unspecialized, it declares only what no constant can leave out. */
   assert(reflect(conditional, "conditional", &reflection, &error));
   assert(reflection.buffer_count == 1);
   for (uint8_t on = 0; on < 2; on++) {
      const struct stela_air_constant_value value = {0, &on, sizeof(on)};
      assert(reflect_specialized(conditional, "conditional", &value, 1, &reflection, &error));
      uint32_t access = on ? STELA_METAL_ACCESS_READ :
                             STELA_METAL_ACCESS_READ | STELA_METAL_ACCESS_WRITE;
      assert(reflection.buffer_count == 2 && reflection.buffers[1].index == 1 &&
             reflection.buffers[1].access == access);
      assert(reflect_specialized(conditional_with_helper, "conditional", &value, 1,
                                 &reflection, &error));
      assert(reflection.buffer_count == 2 && reflection.buffers[1].index == 1 &&
             reflection.buffers[1].access == access);
      const struct stela_metal_specialization specialization = {
         .workgroup_size = {64, 1, 1}, .constants = &value, .constant_count = 1,
      };
      nir_shader *shader = compile(conditional, "conditional", &specialization, &error);
      if (!shader)
         fprintf(stderr, "conditional: %u %s\n", error.kind, error.detail);
      assert(shader);
      stela_metal_release(shader);

      shader = compile(hoisted, "conditional", &specialization, &error);
      assert(shader);
      stela_metal_release(shader);

      /* A function that reads an absent argument reads nothing. */
      shader = compile(unguarded, "conditional", &specialization, &error);
      assert(shader);
      stela_metal_release(shader);
   }
   assert(!reflect(mistyped, "conditional", &reflection, &error) &&
          error.kind == STELA_AIR_METADATA && strstr(error.detail, "predicate"));
   /* A predicate of an integer constant holds when it is not zero. */
   assert(reflect(counted, "counted", &reflection, &error));
   assert(reflection.buffer_count == 1);
   for (uint16_t splits = 0; splits < 3; splits++) {
      const struct stela_air_constant_value value = {0, &splits, sizeof(splits)};
      if (!reflect_specialized(counted, "counted", &value, 1, &reflection, &error))
         fprintf(stderr, "counted: %u %s\n", error.kind, error.detail);
      assert(reflection.buffer_count == (splits ? 2u : 1u));
      assert(!splits || (reflection.buffers[1].index == 1 &&
                         reflection.buffers[1].access == STELA_METAL_ACCESS_READ));
      const struct stela_metal_specialization specialization = {
         .workgroup_size = {64, 1, 1}, .constants = &value, .constant_count = 1,
      };
      nir_shader *shader = compile(counted, "counted", &specialization, &error);
      assert(shader);
      stela_metal_release(shader);
   }
   /* A render stage drops the path that must reach an absent buffer. */
   for (uint8_t on = 0; on < 2; on++) {
      const struct stela_air_constant_value value = {0, &on, sizeof(on)};
      const struct stela_metal_specialization specialization = {
         .constants = &value, .constant_count = 1,
      };
      nir_shader *shader = compile(absent_path, "absent_path", &specialization, &error);
      if (!shader)
         fprintf(stderr, "absent_path: %u %s\n", error.kind, error.detail);
      assert(shader);
      assert(!count_intrinsic(shader, nir_intrinsic_load_global) == !on);
      stela_metal_release(shader);
   }
}

/* The false constant makes the helper unreachable, but admission of the
 * original call graph still precedes early specialization. */
static void
specialized_fragment_calls_preserve_graph_admission(void)
{
   const char *body =
      "@enabled = external addrspace(2) constant i8\n"
      "define <4 x float> @entry() {\n"
      "%value = load i8, ptr addrspace(2) @enabled\n"
      "%on = icmp ne i8 %value, 0\n"
      "br i1 %on, label %active, label %inactive\n"
      "active:\n%result = call <4 x float> @helper()\nret <4 x float> %result\n"
      "inactive:\nret <4 x float> zeroinitializer\n}\n"
      "!air.fragment = !{!2}\n!2 = !{ptr @entry, !3, !4}\n!3 = !{!30}\n!4 = !{}\n"
      "!30 = !{!\"air.render_target\", i32 0, i32 0, !\"air.arg_type_name\", !\"float4\","
      " !\"air.arg_name\", !\"color\"}\n"
      "!air.function_constants = !{!50}\n!50 = !{ptr addrspace(2) @enabled, !\"bool\", !\"enabled\", i32 0}\n";
   const char *unsupported = "declare <4 x float> @unsupported()\n"
      "define internal <4 x float> @helper() {\n"
      "%result = call <4 x float> @unsupported()\nret <4 x float> %result\n}\n";
   char *fixture = ralloc_asprintf(NULL, "%s%s", body, unsupported);
   struct stela_air_error error;
   const struct stela_metal_specialization defaults = {0};
   uint8_t enabled = 0;
   const struct stela_air_constant_value value = {0, &enabled, sizeof(enabled)};
   const struct stela_metal_specialization explicit = {.constants = &value, .constant_count = 1};
   nir_shader *a = compile(fixture, "entry", &defaults, &error);
   nir_shader *b = compile(fixture, "entry", &explicit, &error);
   assert(a && b);
   struct blob original, folded;
   blob_init(&original);
   blob_init(&folded);
   nir_serialize(&original, a, true);
   nir_serialize(&folded, b, true);
   assert(!original.out_of_memory && !folded.out_of_memory && original.size == folded.size &&
          !memcmp(original.data, folded.data, original.size));
   blob_finish(&original);
   blob_finish(&folded);
   stela_metal_release(a);
   stela_metal_release(b);

   enabled = 1;
   assert(!compile(fixture, "entry", &explicit, &error));
   assert(error.kind == STELA_AIR_INSTRUCTION && strstr(error.detail, "unresolved external function"));
   const struct stela_air_error failure = error;
   assert(ralloc_asprintf_append(&fixture,
      "declare i32 @air.get_size_visible_function_table(ptr addrspace(1))\n"));
   assert(!compile(fixture, "entry", &explicit, &error));
   assert(error.kind == failure.kind && error.instruction == failure.instruction &&
          !strcmp(error.detail, failure.detail));
   ralloc_free(fixture);

   enabled = 0;
   fixture = ralloc_asprintf(NULL, "%s%s", body, "define internal <4 x float> @helper() {\n"
      "%result = call <4 x float> @helper()\nret <4 x float> %result\n}\n");
   assert(!compile(fixture, "entry", &explicit, &error));
   assert(error.kind == STELA_AIR_INSTRUCTION && strstr(error.detail, "recursive calls outside"));
   ralloc_free(fixture);

   fixture = ralloc_asprintf(NULL, "%s%s", body, "define internal <4 x float> @helper() {\n"
      "%result = call <4 x float> @chain0()\nret <4 x float> %result\n}\n");
   for (unsigned i = 0; i < 256; i++) {
      assert(ralloc_asprintf_append(&fixture, "define internal <4 x float> @chain%u() {\n"
         "%%result = call <4 x float> @chain%u()\nret <4 x float> %%result\n}\n", i, i + 1));
   }
   assert(ralloc_asprintf_append(&fixture,
      "define internal <4 x float> @chain256() {\nret <4 x float> zeroinitializer\n}\n"));
   LLVMMemoryBufferRef input = bitcode(fixture);
   ralloc_free(fixture);
   const struct stela_air_stage_options stage = {
      .stage = STELA_AIR_FRAGMENT, .resources = {.max_instructions = 10000, .constants = &value,
                                                 .constant_count = 1},
   };
   assert(!stela_air_metal_stage_to_nir(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                       "entry", &nir_options, &stage, &error));
   assert(error.kind == STELA_AIR_LIMIT && strstr(error.detail, "call depth exceeded"));
   LLVMDisposeMemoryBuffer(input);
}

/* A location expression places a present texture; one its constant leaves
 * absent may name no slot, until a read of it survives folding. */
static void
absent_textures_take_no_location(void)
{
   struct stela_air_error error;
   for (int32_t index = -1; index <= 3; index += 4) {
      const struct stela_air_constant_value value = {0, &index, sizeof(index)};
      const struct stela_metal_specialization specialization = {
         .workgroup_size = {64, 1, 1}, .constants = &value, .constant_count = 1,
      };
      nir_shader *shader = compile(located, "located", &specialization, &error);
      assert(shader);
      stela_metal_release(shader);
      shader = compile(unguarded_texture, "located", &specialization, &error);
      assert(index < 0 ? !shader && error.kind == STELA_AIR_ARGUMENT : shader != NULL);
      if (shader)
         stela_metal_release(shader);
   }
}

/* An output a false function constant makes absent is not one. */
static void
function_constants_select_outputs(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   for (uint8_t on = 0; on < 2; on++) {
      const struct stela_air_constant_value value = {0, &on, sizeof(on)};
      assert(reflect_specialized(optional_output, "optional", &value, 1, &reflection, &error));
      const struct stela_metal_linked_varying varying = {"generated(f)", 12, 0};
      const struct stela_metal_specialization specialization = {
         .constants = &value, .constant_count = 1, .varyings = &varying, .varying_count = 1,
      };
      nir_shader *shader = compile(optional_output, "optional", &specialization, &error);
      if (!shader)
         fprintf(stderr, "optional: %u %s\n", error.kind, error.detail);
      assert(shader);
      unsigned outputs = 0;
      nir_foreach_shader_out_variable(variable, shader)
         outputs++;
      assert(outputs == 1u + on);
      stela_metal_release(shader);
   }
}

static void
older_kernels_compile(void)
{
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   struct stela_air_error error;
   nir_shader *shader = compile(older_kernel, "older", &specialization, &error);
   if (!shader)
      fprintf(stderr, "older: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);
}

static void
unoptimized_kernels_compile(void)
{
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   struct stela_air_error error;
   nir_shader *shader = compile(unoptimized_kernel, "unoptimized", &specialization, &error);
   if (!shader)
      fprintf(stderr, "unoptimized: %u %s\n", error.kind, error.detail);
   assert(shader && !shader->scratch_size);
   stela_metal_release(shader);
}

static void
undefined_lanes_read_as_anything(void)
{
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   struct stela_air_error error;
   nir_shader *shader = compile(undefined_lanes, "lanes", &specialization, &error);
   if (!shader)
      fprintf(stderr, "lanes: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);
}

static void
arrays_reflect_each_slot(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(arrays, "arrays", &reflection, &error));
   assert(reflection.texture_count == 3 && reflection.sampler_count == 2);
   for (unsigned i = 0; i < 3; i++) {
      assert(reflection.textures[i].index == 2 + i);
      assert(reflection.textures[i].primitive == STELA_AIR_HALF);
   }
   assert(reflection.samplers[0] == 1 && reflection.samplers[1] == 2);

   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(arrays, "arrays", &specialization, &error);
   if (!shader)
      fprintf(stderr, "arrays: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);
}

static const char *
binding_name(const struct stela_metal_bindings *bindings, uint32_t offset, uint32_t bytes)
{
   static char text[STELA_METAL_BINDING_NAME_BYTES + 1];
   memcpy(text, bindings->names + offset, bytes);
   text[bytes] = 0;
   return text;
}

#define NAMED(row, text) (!strcmp(binding_name(&bindings, (row)->name, (row)->name_bytes), text))
#define TYPED(row, text) (!strcmp(binding_name(&bindings, (row)->type, (row)->type_bytes), text))

static void
bindings_reflect_their_structures(void)
{
   static struct stela_metal_bindings bindings;
   struct stela_air_error error;
   LLVMMemoryBufferRef input = bitcode(bound);
   bool reflected = stela_metal_reflect_bindings(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                                 "bound", NULL, 0, &bindings, &error);
   if (!reflected)
      fprintf(stderr, "bound: %u %s\n", error.kind, error.detail);
   assert(reflected && bindings.binding_count == 6);
   const struct stela_metal_binding *u = &bindings.bindings[0];
   assert(u->kind == STELA_METAL_BINDING_BUFFER && u->index == 0 && u->count == 1 && u->used);
   assert(u->access == STELA_METAL_ACCESS_READ && !u->argument_buffer);
   assert(u->size == 128 && u->alignment == 16 && NAMED(u, "u") && TYPED(u, "Uniforms"));
   assert(u->members == 0 && u->member_count == 3);
   const struct stela_metal_member *inner = &bindings.members[1];
   assert(NAMED(&bindings.members[0], "mvp") && TYPED(&bindings.members[0], "float4x4"));
   assert(NAMED(inner, "inner") && inner->offset == 64 && inner->size == 16 &&
          inner->elements == 2 && !inner->argument);
   assert(bindings.members[2].offset == 96 && bindings.members[2].elements == 3 &&
          bindings.members[2].member_count == 0);

   const struct stela_metal_binding *args = &bindings.bindings[1];
   assert(args->argument_buffer && !args->used && args->members == 3 && args->member_count == 3);
   const struct stela_metal_member *ts = &bindings.members[3];
   assert(ts->argument && ts->kind == STELA_AIR_FIELD_TEXTURE && ts->id == 0 && ts->count == 2);
   assert(ts->texture_type == STELA_AIR_TEXTURE_2D && ts->primitive == STELA_AIR_FLOAT &&
          ts->access == STELA_METAL_ACCESS_SAMPLE);
   const struct stela_metal_member *d = &bindings.members[4];
   assert(d->kind == STELA_AIR_FIELD_BUFFER && d->id == 2 && d->size_pointed == 4 &&
          d->alignment_pointed == 4 &&
          d->access == (STELA_METAL_ACCESS_READ | STELA_METAL_ACCESS_WRITE));
   assert(bindings.members[5].kind == STELA_AIR_FIELD_DATA && bindings.members[5].id == 3);
   /* A nested structure's members follow its parents'. */
   assert(inner->members == 6 && inner->member_count == 2 && bindings.member_count == 8);
   assert(NAMED(&bindings.members[7], "h") && bindings.members[7].offset == 8);

   const struct stela_metal_binding *rw = &bindings.bindings[2];
   assert(rw->kind == STELA_METAL_BINDING_TEXTURE && rw->index == 1 && rw->count == 1 &&
          rw->texture_type == STELA_AIR_TEXTURE_2D && rw->primitive == STELA_AIR_FLOAT &&
          !rw->depth && rw->access == (STELA_METAL_ACCESS_READ | STELA_METAL_ACCESS_WRITE));
   assert(bindings.bindings[3].kind == STELA_METAL_BINDING_SAMPLER &&
          NAMED(&bindings.bindings[3], "s"));
   const struct stela_metal_binding *tg = &bindings.bindings[4];
   assert(tg->kind == STELA_METAL_BINDING_THREADGROUP && tg->size == 16 && tg->alignment == 16);
   assert(bindings.bindings[5].index == 2 && bindings.bindings[5].used);

   assert(!stela_metal_reflect_bindings(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                        "missing", NULL, 0, &bindings, &error) &&
          error.kind == STELA_AIR_ARGUMENT);
   LLVMDisposeMemoryBuffer(input);
}

/* An unspecialized function's reflection lists the arguments its constants
 * may leave out; a specialized one, those they keep. */
static void
bindings_list_conditional_arguments_until_specialized(void)
{
   static struct stela_metal_bindings bindings;
   struct stela_air_error error;
   LLVMMemoryBufferRef input = bitcode(conditional);
   assert(stela_metal_reflect_bindings(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                       "conditional", NULL, 0, &bindings, &error));
   assert(bindings.binding_count == 3);
   assert(NAMED(&bindings.bindings[1], "tint") && bindings.bindings[1].index == 1);
   assert(NAMED(&bindings.bindings[2], "plain") && bindings.bindings[2].index == 1);
   const uint8_t on = 1;
   const struct stela_air_constant_value values[] = {{0, &on, 1}};
   assert(stela_metal_reflect_bindings(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                       "conditional", values, 1, &bindings, &error));
   assert(bindings.binding_count == 2 && NAMED(&bindings.bindings[1], "tint"));
   LLVMDisposeMemoryBuffer(input);
}

/* A kernel's imageblock is a binding at index 0, used where the function
 * calls an imageblock function. */
static void
imageblocks_are_bindings(void)
{
   static struct stela_metal_bindings bindings;
   struct stela_air_error error;
   LLVMMemoryBufferRef input = bitcode(tiled);
   bool reflected = stela_metal_reflect_bindings(LLVMGetBufferStart(input), LLVMGetBufferSize(input),
                                                 "tiled", NULL, 0, &bindings, &error);
   if (!reflected)
      fprintf(stderr, "tiled: %u %s\n", error.kind, error.detail);
   assert(reflected && bindings.binding_count == 2);
   const struct stela_metal_binding *block = &bindings.bindings[0];
   assert(block->kind == STELA_METAL_BINDING_IMAGEBLOCK && block->index == 0 && block->used &&
          block->access == STELA_METAL_ACCESS_READ && NAMED(block, "block") &&
          TYPED(block, "imageblock<Color, layout_implicit>"));
   assert(bindings.bindings[1].kind == STELA_METAL_BINDING_BUFFER &&
          bindings.bindings[1].index == 3);
   LLVMDisposeMemoryBuffer(input);
}

#undef NAMED
#undef TYPED

/* Calls to the module's functions are inlined; recursion is refused. */
static void
defined_functions_inline_into_their_callers(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(calls, "calls", &reflection, &error) && reflection.buffer_count == 1);
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(calls, "calls", &specialization, &error);
   if (!shader)
      fprintf(stderr, "calls: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);

   assert(reflect(calls, "recurses", &reflection, &error));
   assert(!compile(calls, "recurses", &specialization, &error) &&
          error.kind == STELA_AIR_INSTRUCTION && strstr(error.detail, "recursive"));
}

/* A module with no direct call edges still checks unsupported calls during lowering. */
static void
unresolved_calls_are_rejected_without_a_call_graph(void)
{
   const char *callees[] = {"@external", "null"};
   const char *errors[] = {"unresolved external function external", "only from visible function tables"};
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   for (unsigned i = 0; i < ARRAY_SIZE(callees); i++) {
      char *fixture = ralloc_asprintf(NULL,
         "declare void @external()\ndefine void @unresolved() {\ncall void %s()\nret void\n}\n"
         "!air.kernel = !{!2}\n!2 = !{ptr @unresolved, !3, !3}\n!3 = !{}\n", callees[i]);
      struct stela_air_error error;
      assert(!compile(fixture, "unresolved", &specialization, &error));
      assert(error.kind == STELA_AIR_INSTRUCTION && strstr(error.detail, errors[i]));
      ralloc_free(fixture);
   }
}

/* os_log.log_error("hi %d %f %ld", ...) through the method Apple's compiler
 * defines for it, in a kernel, with a float its call promotes and a long. */
static const char *logging =
   "@sub = private addrspace(2) constant [4 x i8] c\"sub\\00\"\n"
   "@cat = private addrspace(2) constant [4 x i8] c\"cat\\00\"\n"
   "@ptr = private addrspace(2) constant [3 x i8] c\"%p\\00\"\n"
   "@fmt = private addrspace(2) constant [13 x i8] c\"hi %d %f %ld\\00\"\n"
   "%log = type { ptr addrspace(2), ptr addrspace(2) }\n"
   "@log = internal addrspace(2) global %log { ptr addrspace(2) @sub, ptr addrspace(2) @cat }\n"
   "@default = external addrspace(2) global %log\n"
   "@llvm.global_ctors = appending global [0 x { i32, ptr, ptr }] zeroinitializer\n"
   "declare void @llvm.va_start.p0(ptr)\ndeclare void @llvm.va_end.p0(ptr)\n"
   "declare void @air.os_log(ptr addrspace(2), ptr addrspace(2), i32, ptr addrspace(2), ptr, i64)\n"
   "define linkonce_odr void @log_error(ptr addrspace(2) %o, ptr addrspace(2) %f, i64 %n, ...) {\n"
   "%list = alloca ptr, align 8\ncall void @llvm.va_start.p0(ptr %list)\n"
   "%s = load ptr addrspace(2), ptr addrspace(2) %o, align 8\n"
   "%g = getelementptr inbounds %log, ptr addrspace(2) %o, i64 0, i32 1\n"
   "%c = load ptr addrspace(2), ptr addrspace(2) %g, align 8\n"
   "%l = load ptr, ptr %list, align 8\n"
   "call void @air.os_log(ptr addrspace(2) %s, ptr addrspace(2) %c, i32 16, ptr addrspace(2) %f,"
   " ptr %l, i64 %n)\n"
   "call void @llvm.va_end.p0(ptr %list)\nret void\n}\n"
   "define void @logs(ptr addrspace(1) %data, i32 %tid) {\n"
   "%t = uitofp i32 %tid to float\n%d = fpext float %t to double\n%w = zext i32 %tid to i64\n"
   "call void (ptr addrspace(2), ptr addrspace(2), i64, ...) @log_error(ptr addrspace(2) @log,"
   " ptr addrspace(2) @fmt, i64 20, i32 %tid, double %d, i64 %w)\n"
   "call void (ptr addrspace(2), ptr addrspace(2), i64, ...) @log_error(ptr addrspace(2) @default,"
   " ptr addrspace(2) @fmt, i64 4, i32 %tid)\n"
   "call void (ptr addrspace(2), ptr addrspace(2), i64, ...) @log_error(ptr addrspace(2) @default,"
   " ptr addrspace(2) @ptr, i64 8, ptr addrspace(1) %data)\n"
   "ret void\n}\n"
   "define void @promotes(ptr addrspace(1) %data, i32 %tid) {\n"
   "%t = uitofp i32 %tid to float\n%d = fpext float %t to double\n"
   "%f = fptrunc double %d to float\n"
   "store float %f, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "!air.kernel = !{!2, !5}\n"
   "!2 = !{ptr @logs, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n!5 = !{ptr @promotes, !3, !4}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

static void
os_log_appends_self_contained_records(void)
{
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(logging, "logs", &reflection, &error));
   nir_shader *shader = compile(logging, "logs", &specialization, &error);
   assert(shader);
   /* Each call reads whether the buffer is past its capacity, reserves its
    * record and writes at least its six words. */
   assert(count_intrinsic(shader, nir_intrinsic_load_global) >= 6);
   assert(count_intrinsic(shader, nir_intrinsic_global_atomic) >= 3);
   assert(count_intrinsic(shader, nir_intrinsic_store_global) >= 18);
   stela_metal_release(shader);
   /* A double any other instruction reads is no promotion. */
   assert(!compile(logging, "promotes", &specialization, &error));
   /* A library defines os_log_default with a null subsystem and an all-ones
    * category: of neither, as its declaration is. */
   static const char declared[] = "@default = external addrspace(2) global %log\n";
   static const char defined[] = "@default = internal addrspace(2) global %log { ptr addrspace(2) null,"
                                 " ptr addrspace(2) inttoptr (i64 -1 to ptr addrspace(2)) }\n";
   const char *at = strstr(logging, declared);
   assert(at);
   size_t before = (size_t)(at - logging), bytes = strlen(logging) + sizeof(defined);
   char *library = malloc(bytes);
   assert(library);
   snprintf(library, bytes, "%.*s%s%s", (int)before, logging, defined, at + strlen(declared));
   shader = compile(library, "logs", &specialization, &error);
   if (!shader)
      fprintf(stderr, "defined default: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);
   free(library);
}

static const char *stitching_kernel =
   "declare i32 @quadrupled.MTL_VISIBLE_FN_REF(i32) section \"air.externally_defined\"\n"
   "define void @stitches(ptr addrspace(1) %data, i32 %tid) {\n"
   "%v = call i32 @quadrupled.MTL_VISIBLE_FN_REF(i32 %tid)\n"
   "store i32 %v, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n!air.visible_function_references = !{!5}\n"
   "!2 = !{ptr @stitches, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!5 = !{!\"air.visible_function_reference\", ptr @quadrupled.MTL_VISIBLE_FN_REF,"
   " !\"quadrupled\"}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* A kernel that keeps a generic pointer to its buffer in a thread struct and
 * passes the struct to a visible function another module defines, as MPS's
 * NDArray kernels pass their stitched functions a _MPSKernelInOut. */
static const char *inout_kernel =
   "%inout = type { ptr addrspace(8), i64 }\n"
   "declare float @read_first.MTL_VISIBLE_FN_REF(ptr) section \"air.externally_defined\"\n"
   "define void @passes_inout(ptr addrspace(1) %data, ptr addrspace(1) %out, i32 %tid) {\n"
   "%s = alloca %inout, align 8\n"
   "%g = addrspacecast ptr addrspace(1) %data to ptr addrspace(8)\n"
   "%p = getelementptr inbounds %inout, ptr %s, i64 0, i32 0\n"
   "store ptr addrspace(8) %g, ptr %p, align 8\n"
   "%n = getelementptr inbounds %inout, ptr %s, i64 0, i32 1\n"
   "store i64 4, ptr %n, align 8\n"
   "%v = call float @read_first.MTL_VISIBLE_FN_REF(ptr %s)\n"
   "%i = zext i32 %tid to i64\n"
   "%o = getelementptr inbounds float, ptr addrspace(1) %out, i64 %i\n"
   "store float %v, ptr addrspace(1) %o, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n!air.visible_function_references = !{!5}\n"
   "!2 = !{ptr @passes_inout, !3, !4}\n!3 = !{}\n!4 = !{!40, !41, !42}\n"
   "!5 = !{!\"air.visible_function_reference\", ptr @read_first.MTL_VISIBLE_FN_REF,"
   " !\"read_first\"}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"float\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.buffer\", !\"air.location_index\", i32 1, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"float\", !\"air.arg_name\", !\"out\"}\n"
   "!42 = !{i32 2, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* The visible function it calls: the first element through the struct's
 * pointer. */
static const char *inout_reader =
   "%inout = type { ptr addrspace(8), i64 }\n"
   "define float @read_first(ptr %s) {\n"
   "%p = getelementptr inbounds %inout, ptr %s, i64 0, i32 0\n"
   "%g = load ptr addrspace(8), ptr %p, align 8\n"
   "%d = addrspacecast ptr addrspace(8) %g to ptr addrspace(1)\n"
   "%v = load float, ptr addrspace(1) %d, align 4\nret float %v\n}\n"
   "!air.visible = !{}\n";

/* A pointer a thread struct holds reaches the linked function that reads
 * it; without that function, the refusal names it. */
static void
thread_structs_carry_pointers_to_linked_functions(void)
{
   LLVMMemoryBufferRef reader = bitcode(inout_reader);
   const struct stela_air_module modules[] = {
      {LLVMGetBufferStart(reader), LLVMGetBufferSize(reader)},
   };
   const struct stela_metal_specialization linked = {
      .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 1,
   };
   struct stela_air_error error = {0};
   nir_shader *shader = compile(inout_kernel, "passes_inout", &linked, &error);
   if (!shader)
      fprintf(stderr, "passes_inout: %u %s\n", error.kind, error.detail);
   assert(shader);
   /* The struct is gone: two binding addresses, then the read goes
    * straight to the buffer. */
   assert(count_intrinsic(shader, nir_intrinsic_load_global) == 3);
   assert(count_intrinsic(shader, nir_intrinsic_store_global) == 1);
   assert(!count_intrinsic(shader, nir_intrinsic_load_deref) &&
          !count_intrinsic(shader, nir_intrinsic_store_deref));
   stela_metal_release(shader);

   const struct stela_metal_specialization unlinked = {.workgroup_size = {64, 1, 1}};
   assert(!compile(inout_kernel, "passes_inout", &unlinked, &error));
   assert(error.kind == STELA_AIR_ARGUMENT &&
          !strcmp(error.detail, "visible function read_first is not linked"));
   LLVMDisposeMemoryBuffer(reader);
}

/* A kernel whose path past a check LLVM proved impossible is unreachable. */
static const char *unreachable_kernel =
   "define void @trusts(ptr addrspace(1) %data, i32 %tid) {\n"
   "entry:\n%big = icmp ugt i32 %tid, 1000\nbr i1 %big, label %never, label %store\n"
   "never:\nunreachable\n"
   "store:\nstore i32 %tid, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n"
   "!2 = !{ptr @trusts, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* Reaching unreachable is undefined: the invocation ends there. */
static void
unreachable_paths_end_the_invocation(void)
{
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   struct stela_air_error error = {0};
   nir_shader *shader = compile(unreachable_kernel, "trusts", &specialization, &error);
   if (!shader)
      fprintf(stderr, "trusts: %u %s\n", error.kind, error.detail);
   assert(shader && count_intrinsic(shader, nir_intrinsic_store_global) == 1);
   stela_metal_release(shader);
}

/* A file's bytes, or NULL. */
static void *
read_file(const char *path, size_t *size)
{
   FILE *file = fopen(path, "rb");
   if (!file)
      return NULL;
   fseek(file, 0, SEEK_END);
   *size = (size_t)ftell(file);
   fseek(file, 0, SEEK_SET);
   void *bytes = malloc(*size ? *size : 1);
   bool read = bytes && fread(bytes, 1, *size, file) == *size;
   fclose(file);
   if (!read) {
      free(bytes);
      return NULL;
   }
   return bytes;
}

/* Function constant values as INDEX=HEX,...; "-" for none. */
static unsigned
parse_constants(const char *text, struct stela_air_constant_value *values, uint8_t (*bytes)[16])
{
   unsigned count = 0;
   while (text && *text && *text != '-') {
      unsigned index = strtoul(text, (char **)&text, 10);
      assert(*text == '=');
      text++;
      unsigned size = 0;
      while (isxdigit((unsigned char)text[0]) && isxdigit((unsigned char)text[1]) && size < 16) {
         char pair[3] = {text[0], text[1], 0};
         bytes[count][size++] = (uint8_t)strtoul(pair, NULL, 16);
         text += 2;
      }
      values[count] = (struct stela_air_constant_value){index, bytes[count], size};
      count++;
      if (*text == ',')
         text++;
   }
   return count;
}

/* The modules MPS links an ndarray kernel with, from STELA_TEST_MPS_FUSED's
 * `directory`: the stitched module `stitched`, then the sources its
 * manifest.txt's `source PATH NAME VALUES` lines name, each found by its
 * file name in the directory. */
static struct stela_air_module mps_modules[64];
static struct stela_air_constant_value mps_values[64][32];
static uint8_t mps_bytes[64][32][16];
static char mps_names[64][256];

static unsigned
mps_load_modules(const char *directory, const char *stitched)
{
   char path[4096];
   size_t size;
   unsigned count = 0;
   snprintf(path, sizeof(path), "%s/%s", directory, stitched);
   mps_modules[count] = (struct stela_air_module){.bitcode = read_file(path, &size), .size = size};
   assert(mps_modules[count].bitcode);
   count++;
   snprintf(path, sizeof(path), "%s/manifest.txt", directory);
   FILE *manifest = fopen(path, "r");
   assert(manifest);
   char line[4096];
   while (fgets(line, sizeof(line), manifest)) {
      char source[512], function[256], constants[2048];
      if (sscanf(line, "source %511s %255s %2047s", source, function, constants) != 3)
         continue;
      assert(count < ARRAY_SIZE(mps_modules));
      const char *file = strrchr(source, '/') ? strrchr(source, '/') + 1 : source;
      unsigned copy = 0;
      for (unsigned i = 1; i < count; i++)
         copy += !strcmp(mps_names[i], file);
      snprintf(mps_names[count], sizeof(mps_names[count]), "%s", file);
      snprintf(path, sizeof(path), "%s/%s", directory, file);
      mps_modules[count] = (struct stela_air_module){
         .bitcode = read_file(path, &size), .size = size, .constants = mps_values[count],
         .constant_count = parse_constants(constants, mps_values[count], mps_bytes[count]),
         .copy = copy,
      };
      assert(mps_modules[count].bitcode);
      count++;
   }
   fclose(manifest);
   return count;
}

static void
mps_release_modules(unsigned count)
{
   for (unsigned i = 0; i < count; i++)
      free((void *)mps_modules[i].bitcode);
}

/* Oracle: MPS's ndArrayIdentity, which passes its stitched functions a
 * thread _MPSKernelInOut holding the generic pointer to its buffer array,
 * with MetalFX's values. STELA_TEST_MPS_FUSED names a directory of Apple
 * output, never committed: ndArrayIdentity.bc, stitched.bc and a
 * manifest.txt whose `source PATH NAME VALUES` lines name the modules the
 * stitched one calls, each found by its file name in the directory. */
static void
mps_identity_links_its_stitched_functions(void)
{
   const char *directory = getenv("STELA_TEST_MPS_FUSED");
   if (!directory)
      return;
   char path[4096];
   size_t kernel_size;
   snprintf(path, sizeof(path), "%s/ndArrayIdentity.bc", directory);
   void *kernel = read_file(path, &kernel_size);
   assert(kernel);
   unsigned count = mps_load_modules(directory, "stitched.bc");
   const struct stela_air_module *modules = mps_modules;
   /* MetalFX's: two buffers, an identity map. */
   struct stela_air_constant_value own[2];
   uint8_t own_bytes[2][16];
   unsigned own_count = parse_constants("122=02000000,124=00000000", own, own_bytes);
   struct stela_metal_target driver = target;
   driver.max_instructions = 1u << 20;
   struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .constants = own, .constant_count = own_count,
      .modules = modules, .module_count = count,
   };
   struct stela_air_error error = {0};
   nir_shader *shader = stela_metal_compile(kernel, kernel_size, "ndArrayIdentity", &driver,
                                            &specialization, &error);
   if (!shader)
      fprintf(stderr, "ndArrayIdentity: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);
   /* Unlinked, the refusal names a stitched function, not the struct. */
   specialization.module_count = 0;
   assert(!stela_metal_compile(kernel, kernel_size, "ndArrayIdentity", &driver, &specialization,
                               &error));
   assert(error.kind == STELA_AIR_ARGUMENT && strstr(error.detail, "Primary_f is not linked"));
   mps_release_modules(count);
   free(kernel);
}

/* Oracle: the MPS ndarray kernels a MetalFX network runs, linked with the
 * stitched functions they call. STELA_TEST_MPS_FUSED's directory (see
 * above) holds a kernels.txt of lines `FILE ENTRY VALUES X,Y,Z LENGTHS
 * STITCHED`: the kernel's bitcode, its function constant values, its
 * threadgroup size, its threadgroup memory lengths as INDEX:BYTES,... ("-"
 * for none, as the service sizes each slot at dispatch) and the stitched
 * module. Each compiles. The depthwise gradient kernels pass each of a
 * block's calls its own thread struct of buffer pointers. */
static void
mps_kernels_compile(void)
{
   const char *directory = getenv("STELA_TEST_MPS_FUSED");
   char path[4096];
   snprintf(path, sizeof(path), "%s/kernels.txt", directory ? directory : "");
   FILE *list = directory ? fopen(path, "r") : NULL;
   if (!list)
      return;
   char line[4096];
   unsigned failed = 0;
   while (fgets(line, sizeof(line), list)) {
      char file[256], entry[256], constants[1024], lengths[256], stitched[256];
      unsigned x, y, z;
      if (line[0] == '#' ||
          sscanf(line, "%255s %255s %1023s %u,%u,%u %255s %255s", file, entry, constants, &x, &y, &z,
                 lengths, stitched) != 8)
         continue;
      size_t kernel_size;
      snprintf(path, sizeof(path), "%s/%s", directory, file);
      void *kernel = read_file(path, &kernel_size);
      assert(kernel);
      unsigned count = mps_load_modules(directory, stitched);
      struct stela_air_constant_value own[32];
      uint8_t own_bytes[32][16];
      unsigned own_count = parse_constants(constants, own, own_bytes);
      struct stela_metal_threadgroup_length threadgroup[8];
      unsigned threadgroup_count = 0;
      for (const char *at = lengths; *at && *at != '-' && threadgroup_count < ARRAY_SIZE(threadgroup);) {
         char *end;
         threadgroup[threadgroup_count].index = strtoul(at, &end, 10);
         assert(*end == ':');
         threadgroup[threadgroup_count++].bytes = strtoul(end + 1, &end, 10);
         at = *end == ',' ? end + 1 : end;
      }
      struct stela_metal_target driver = target;
      driver.max_instructions = 1u << 20;
      driver.max_private_bytes = 512u << 10;
      const struct stela_metal_specialization specialization = {
         .workgroup_size = {x, y, z}, .constants = own, .constant_count = own_count,
         .threadgroup_lengths = threadgroup, .threadgroup_length_count = threadgroup_count,
         .modules = mps_modules, .module_count = count,
      };
      struct stela_air_error error = {0};
      nir_shader *shader = stela_metal_compile(kernel, kernel_size, entry, &driver, &specialization,
                                               &error);
      if (!shader) {
         fprintf(stderr, "%s %s: %u at %u %s\n", entry, constants, error.kind, error.instruction,
                 error.detail);
         failed++;
      }
      stela_metal_release(shader);
      mps_release_modules(count);
      free(kernel);
   }
   fclose(list);
   assert(!failed);
}

/* Keeps the bitcode of LLVM's wrapper for a Darwin target, as `metal/library.rs`
 * reads it from a library. */
static void
keep_stitched(void *data, const void *bytes, size_t size)
{
   uint32_t wrapper[5];
   assert(size >= sizeof(wrapper));
   memcpy(wrapper, bytes, sizeof(wrapper));
   assert(wrapper[0] == 0x0b17c0de && wrapper[2] + wrapper[3] <= size);
   *(LLVMMemoryBufferRef *)data = LLVMCreateMemoryBufferWithMemoryRangeCopy(
      (const char *)bytes + wrapper[2], wrapper[3], "stitched");
}

static bool
visible_quadrupled(void *data, const char *name, size_t name_bytes,
                   enum stela_metal_function_stage stage)
{
   assert(name_bytes == 10 && !memcmp(name, "quadrupled", 10) && stage == STELA_METAL_VISIBLE);
   (*(unsigned *)data)++;
   return true;
}

/* quadrupled(x) = doubled(doubled(x)) is a visible function of its own module,
 * which a pipeline links with the module it calls. */
static void
stitched_functions_call_their_nodes_in_order(void)
{
   LLVMMemoryBufferRef source = bitcode(calls);
   struct stela_air_stitch_source sources[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "doubled"},
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "calls"},
   };
   const uint32_t first = 0, second = 1;
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_INPUT},
      {.function = 0, .arguments = &first, .argument_count = 1},
      {.function = 0, .arguments = &second, .argument_count = 1},
   };
   const struct stela_air_stitch_graph graph = {"quadrupled", nodes, 3, 2};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   assert(stela_metal_stitch(sources, 1, &graph, 1, keep_stitched, &stitched, &error));

   unsigned listed = 0;
   assert(stela_metal_functions(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched),
                                visible_quadrupled, &listed, &error));
   assert(listed == 1);

   const struct stela_air_module modules[] = {
      {LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched)},
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source)},
   };
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 2,
   };
   nir_shader *shader = compile(stitching_kernel, "stitches", &specialization, &error);
   assert(shader);
   stela_metal_release(shader);

   /* Without the module it calls, the function is undefined. */
   const struct stela_metal_specialization unlinked = {
      .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 1,
   };
   assert(!compile(stitching_kernel, "stitches", &unlinked, &error));

   /* Only visible functions stitch, with as many arguments as they take. */
   assert(!stela_metal_stitch(&sources[1], 1, &graph, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "not visible"));
   const struct stela_air_stitch_node calling_nothing[] = {
      {.function = 0, .argument_count = 0},
   };
   const struct stela_air_stitch_graph short_call = {"short", calling_nothing, 1,
                                                     STELA_AIR_STITCH_NO_OUTPUT};
   assert(!stela_metal_stitch(sources, 1, &short_call, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "number of arguments"));
   LLVMDisposeMemoryBuffer(stitched);
   LLVMDisposeMemoryBuffer(source);
}

/* Core Image's shape: made() returns a struct, and summed() takes it as one
 * stitching argument coerced to its two fields, whose first is a struct of
 * the same layout under another name, as another module names it. stored()
 * returns nothing, which its stitching info's null return type says, as
 * Core Image's pixel writes do. */
static const char *coerced =
   "%\"struct.a.box\" = type { i32 }\n%\"struct.b.box\" = type { i32 }\n"
   "%\"struct.pair\" = type { %\"struct.a.box\", i32 }\n"
   "define %\"struct.pair\" @made(i32 %x, i32 %y) {\n"
   "%b = insertvalue %\"struct.a.box\" poison, i32 %x, 0\n"
   "%p = insertvalue %\"struct.pair\" poison, %\"struct.a.box\" %b, 0\n"
   "%q = insertvalue %\"struct.pair\" %p, i32 %y, 1\nret %\"struct.pair\" %q\n}\n"
   "define i32 @summed(%\"struct.b.box\" %f, i32 %g) {\n"
   "%v = extractvalue %\"struct.b.box\" %f, 0\n%s = add i32 %v, %g\nret i32 %s\n}\n"
   "define void @stored(i32 %v, i32 %w) {\nret void\n}\n"
   "!air.visible = !{!501, !502, !503}\n"
   "!501 = !{ptr @made, !510, !511}\n!502 = !{ptr @summed, !512, !513, !520}\n"
   "!510 = !{!514}\n!514 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"pair\"}\n"
   "!511 = !{!515, !516}\n"
   "!515 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"x\"}\n"
   "!516 = !{i32 1, !\"air.visible_input\", !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"y\"}\n"
   "!512 = !{!517}\n!517 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"int\"}\n"
   "!513 = !{!515, !516}\n"
   "!520 = !{!\"air.stitching_info\", !521, !522}\n"
   "!521 = !{!\"air.stitching_type\", !530}\n!530 = !{!\"air.int_type\", i32 4, i32 4, null}\n"
   "!522 = !{!\"air.stitching_argument\", !523, !\"p\"}\n!523 = !{!\"air.stitching_type\", !524}\n"
   "!524 = !{!\"air.struct_type\", i32 8, i32 4, null, !\"pair\", !525}\n!525 = !{!526, !527}\n"
   "!526 = !{!\"air.record_field\", i32 0, i32 4, !530, !\"b\", null}\n"
   "!527 = !{!\"air.record_field\", i32 4, i32 4, !530, !\"g\", null}\n"
   "!503 = !{ptr @stored, !540, !513, !541}\n!540 = !{}\n"
   "!541 = !{!\"air.stitching_info\", null, !542, !543}\n"
   "!542 = !{!\"air.stitching_argument\", !544, !\"v\"}\n!544 = !{!\"air.stitching_type\", !530}\n"
   "!543 = !{!\"air.stitching_argument\", !544, !\"w\"}\n";

/* A call passes a struct value's fields to the parameters its coerced
 * argument covers; an input cannot feed one. */
static void
stitched_struct_arguments_pass_their_fields(void)
{
   LLVMMemoryBufferRef source = bitcode(coerced);
   const struct stela_air_stitch_source sources[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "made"},
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "summed"},
   };
   const uint32_t pair[] = {0, 1}, made = 2;
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_INPUT, .input = 0},
      {.function = STELA_AIR_STITCH_INPUT, .input = 1},
      {.function = 0, .arguments = pair, .argument_count = 2},
      {.function = 1, .arguments = &made, .argument_count = 1},
   };
   const struct stela_air_stitch_graph graph = {"total", nodes, 4, 3};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   assert(stela_metal_stitch(sources, 2, &graph, 1, keep_stitched, &stitched, &error));

   LLVMContextRef context = LLVMContextCreate();
   LLVMModuleRef module;
   assert(!LLVMParseBitcodeInContext2(context, stitched, &module));
   LLVMDisposeMemoryBuffer(stitched);
   LLVMValueRef total = LLVMGetNamedFunction(module, "total");
   assert(total && LLVMCountParams(total) == 2);
   unsigned calls = 0;
   for (LLVMBasicBlockRef block = LLVMGetFirstBasicBlock(total); block;
        block = LLVMGetNextBasicBlock(block)) {
      for (LLVMValueRef instr = LLVMGetFirstInstruction(block); instr;
           instr = LLVMGetNextInstruction(instr)) {
         if (LLVMGetInstructionOpcode(instr) != LLVMCall)
            continue;
         size_t length;
         const char *name = LLVMGetValueName2(LLVMGetCalledValue(instr), &length);
         if (length == 6 && !memcmp(name, "summed", 6)) {
            assert(LLVMGetNumArgOperands(instr) == 2);
            calls++;
         }
      }
   }
   assert(calls == 1);
   LLVMDisposeModule(module);
   LLVMContextDispose(context);

   const uint32_t input = 0;
   const struct stela_air_stitch_node passing[] = {
      {.function = STELA_AIR_STITCH_INPUT, .input = 0},
      {.function = 1, .arguments = &input, .argument_count = 1},
   };
   const struct stela_air_stitch_graph passed = {"passed", passing, 2, 1};
   assert(!stela_metal_stitch(sources, 2, &passed, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "no call's value"));

   const struct stela_air_stitch_source void_source = {
      LLVMGetBufferStart(source), LLVMGetBufferSize(source), "stored"};
   const struct stela_air_stitch_node storing[] = {
      {.function = STELA_AIR_STITCH_INPUT, .input = 0},
      {.function = STELA_AIR_STITCH_INPUT, .input = 1},
      {.function = 0, .arguments = pair, .argument_count = 2},
   };
   const struct stela_air_stitch_graph store = {"store", storing, 3, STELA_AIR_STITCH_NO_OUTPUT};
   stitched = NULL;
   assert(stela_metal_stitch(&void_source, 1, &store, 1, keep_stitched, &stitched, &error));
   LLVMDisposeMemoryBuffer(stitched);
   LLVMDisposeMemoryBuffer(source);
}

/* A kernel that leaves its loop two ways, by returning from each. */
static const char *returning_kernel =
   "define void @returns(ptr addrspace(1) %data, i32 %tid) {\n"
   "entry:\nbr label %loop\n"
   "loop:\n%i = phi i32 [0, %entry], [%n, %next]\n%p = getelementptr i32, ptr addrspace(1) %data, i32 %i\n"
   "%v = load i32, ptr addrspace(1) %p, align 4\n%hit = icmp eq i32 %v, %tid\n"
   "br i1 %hit, label %found, label %next\n"
   "found:\nstore i32 %i, ptr addrspace(1) %data, align 4\nret void\n"
   "next:\n%n = add i32 %i, 1\n%more = icmp ult i32 %n, %tid\nbr i1 %more, label %loop, label %done\n"
   "done:\nret void\n}\n"
   "!air.kernel = !{!2}\n!2 = !{ptr @returns, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

static bool
jumps(nir_shader *shader, nir_jump_type type)
{
   nir_foreach_function_impl(impl, shader) {
      nir_foreach_block(block, impl) {
         nir_foreach_instr(instr, block) {
            if (instr->type == nir_instr_type_jump && nir_instr_as_jump(instr)->type == type)
               return true;
         }
      }
   }
   return false;
}

/* Returns from a loop leave no return for the backends, whose divergence
 * analysis takes none. */
static void
loops_return_without_return_jumps(void)
{
   struct stela_air_error error;
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(returning_kernel, "returns", &specialization, &error);
   assert(shader && !jumps(shader, nir_jump_return));
   nir_divergence_analysis(shader);
   stela_metal_release(shader);
}

/* kept(data, v) and its float and float4 forms give a stitched kernel's
 * values somewhere to go. */
static const char *keeps =
   "define void @kept(ptr addrspace(1) %data, i32 %v) {\n"
   "store i32 %v, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "define void @kept_float(ptr addrspace(1) %data, float %v) {\n"
   "store float %v, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "define void @kept_float4(ptr addrspace(1) %data, <4 x float> %v) {\n"
   "store <4 x float> %v, ptr addrspace(1) %data, align 16\nret void\n}\n"
   "!air.visible = !{!1, !4, !5}\n"
   "!1 = !{ptr @kept, !2, !3}\n!2 = !{}\n!3 = !{!30, !31}\n"
   "!4 = !{ptr @kept_float, !2, !3}\n!5 = !{ptr @kept_float4, !2, !3}\n"
   "!30 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"int*\","
   " !\"air.arg_name\", !\"data\"}\n"
   "!31 = !{i32 1, !\"air.visible_input\", !\"air.arg_type_name\", !\"int\","
   " !\"air.arg_name\", !\"v\"}\n";

/* Whether compile_stitched's process has made bicubic samplers. */
static uint32_t stitched_bicubic;

/* Stitched kernel `entry` of stitched, linked with the module of source and
 * keeps', which its calls name. */
static nir_shader *
compile_stitched(LLVMMemoryBufferRef stitched, const char *entry, LLVMMemoryBufferRef source,
                 LLVMMemoryBufferRef kept, const struct stela_metal_threadgroup_length *lengths,
                 unsigned length_count, struct stela_air_error *error)
{
   const struct stela_air_module modules[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source)},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept)},
   };
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 2,
      .threadgroup_length_count = length_count, .threadgroup_lengths = lengths,
      .bicubic = stitched_bicubic,
   };
   nir_shader *shader = stela_metal_compile(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched),
                                            entry, &target, &specialization, error);
   if (!shader)
      fprintf(stderr, "%s: %u %s\n", entry, error->kind, error->detail);
   return shader;
}

static bool
kernel_positioned(void *data, const char *name, size_t name_bytes,
                  enum stela_metal_function_stage stage)
{
   assert(name_bytes == 10 && !memcmp(name, "positioned", 10) && stage == STELA_METAL_KERNEL);
   (*(unsigned *)data)++;
   return true;
}

/* kernel positioned() { kept(buffer 0, doubled(thread_position_in_grid)); }
 * reads its own builtins, as the type of the parameter each feeds; only a
 * kernel's graph reads builtins, and it takes and returns nothing. */
static void
stitched_kernels_read_their_builtins(void)
{
   LLVMMemoryBufferRef source = bitcode(calls), kept = bitcode(keeps);
   const struct stela_air_stitch_source sources[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "doubled"},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept), "kept"},
   };
   const uint32_t position = 0, stored[] = {2, 1};
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_BUILTIN, .input = STELA_AIR_STITCH_THREAD_POSITION_IN_GRID},
      {.function = 0, .arguments = &position, .argument_count = 1},
      {.function = STELA_AIR_STITCH_BUFFER_ADDRESS, .input = 0},
      {.function = 1, .arguments = stored, .argument_count = 2},
   };
   const struct stela_air_stitch_graph graph = {"positioned", nodes, 4, STELA_AIR_STITCH_NO_OUTPUT,
                                                true};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   assert(stela_metal_stitch(sources, 2, &graph, 1, keep_stitched, &stitched, &error));
   unsigned listed = 0;
   assert(stela_metal_functions(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched),
                                kernel_positioned, &listed, &error));
   assert(listed == 1);
   nir_shader *shader = compile_stitched(stitched, "positioned", source, kept, NULL, 0, &error);
   assert(shader && count_intrinsic(shader, nir_intrinsic_load_workgroup_id) >= 1);
   stela_metal_release(shader);
   /* A module links again as another specialization's copy, whose
    * definitions take its suffix; linked twice alike, it defines its
    * functions twice. */
   struct stela_air_module copies[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source)},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept)},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept), .copy = 1},
   };
   const struct stela_air_linked_function copied[] = {{"kept.stela.copy1", 16}, {"kept", 4}};
   struct stela_metal_specialization twice = {
      .workgroup_size = {64, 1, 1}, .modules = copies, .module_count = 3,
      .linked_functions = copied, .linked_function_count = 2,
   };
   shader = stela_metal_compile(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched), "positioned",
                                &target, &twice, &error);
   if (!shader)
      fprintf(stderr, "copies: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);
   copies[2].copy = 0;
   assert(!stela_metal_compile(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched), "positioned",
                               &target, &twice, &error));
   LLVMDisposeMemoryBuffer(stitched);

   /* The position in the threadgroup is the local one. */
   const struct stela_air_stitch_node local[] = {
      {.function = STELA_AIR_STITCH_BUILTIN, .input = STELA_AIR_STITCH_THREAD_POSITION_IN_THREADGROUP},
      nodes[1], nodes[2], nodes[3],
   };
   const struct stela_air_stitch_graph locally = {"positioned", local, 4,
                                                  STELA_AIR_STITCH_NO_OUTPUT, true};
   assert(stela_metal_stitch(sources, 2, &locally, 1, keep_stitched, &stitched, &error));
   shader = compile_stitched(stitched, "positioned", source, kept, NULL, 0, &error);
   /* The local position is the invocation index's in the running shape. */
   assert(shader && count_intrinsic(shader, nir_intrinsic_load_local_invocation_index) >= 1);
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(stitched);

   /* As Metal's: a visible function's graph reads no builtin, and a
    * kernel's takes no input and returns nothing. */
   const struct stela_air_stitch_graph visible = {"visible", nodes, 4, STELA_AIR_STITCH_NO_OUTPUT};
   assert(!stela_metal_stitch(sources, 2, &visible, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "reads a builtin or binding"));
   const struct stela_air_stitch_graph returning = {"returning", nodes, 2, 1, true};
   assert(!stela_metal_stitch(sources, 2, &returning, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "returns a value"));
   const struct stela_air_stitch_node input[] = {
      {.function = STELA_AIR_STITCH_INPUT}, nodes[1],
   };
   const struct stela_air_stitch_graph taking = {"taking", input, 2, STELA_AIR_STITCH_NO_OUTPUT,
                                                 true};
   assert(!stela_metal_stitch(sources, 2, &taking, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "takes an input"));
   /* Only known builtins stitch. */
   const struct stela_air_stitch_node unknown[] = {
      {.function = STELA_AIR_STITCH_BUILTIN, .input = 0}, nodes[1], nodes[2], nodes[3],
   };
   const struct stela_air_stitch_graph unknowing = {"unknowing", unknown, 4,
                                                    STELA_AIR_STITCH_NO_OUTPUT, true};
   assert(!stela_metal_stitch(sources, 2, &unknowing, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "does not know"));
   LLVMDisposeMemoryBuffer(kept);
   LLVMDisposeMemoryBuffer(source);
}

/* kernel positioned() { kept(buffer 0, doubled(the int 8 bytes into buffer
 * 1)); } reads the buffers it binds there, which its reflection lists. */
static void
stitched_buffer_addresses_read_their_kernels_buffers(void)
{
   LLVMMemoryBufferRef source = bitcode(calls), kept = bitcode(keeps);
   const struct stela_air_stitch_source sources[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "doubled"},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept), "kept"},
   };
   const uint32_t address = 0, stored[] = {2, 1};
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_BUFFER_ADDRESS, .input = 1, .offset = 8, .dereference = true},
      {.function = 0, .arguments = &address, .argument_count = 1},
      {.function = STELA_AIR_STITCH_BUFFER_ADDRESS, .input = 0},
      {.function = 1, .arguments = stored, .argument_count = 2},
   };
   const struct stela_air_stitch_graph graph = {"positioned", nodes, 4, STELA_AIR_STITCH_NO_OUTPUT,
                                                true};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   assert(stela_metal_stitch(sources, 2, &graph, 1, keep_stitched, &stitched, &error));
   struct stela_metal_reflection reflection;
   assert(stela_metal_reflect(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched),
                              "positioned", NULL, 0, &reflection, &error));
   assert(reflection.stage == STELA_METAL_KERNEL && reflection.buffer_count == 2);
   assert(reflection.buffers[0].index == 1 && reflection.buffers[0].access == STELA_METAL_ACCESS_READ);
   assert(reflection.buffers[1].index == 0 &&
          reflection.buffers[1].access == (STELA_METAL_ACCESS_READ | STELA_METAL_ACCESS_WRITE));
   nir_shader *shader = compile_stitched(stitched, "positioned", source, kept, NULL, 0, &error);
   /* Both buffers' addresses from the table, and the int. */
   assert(shader && count_intrinsic(shader, nir_intrinsic_load_global) == 3);
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(stitched);

   /* An int parameter takes no address. */
   const struct stela_air_stitch_node addressing[] = {
      {.function = STELA_AIR_STITCH_BUFFER_ADDRESS, .input = 1}, nodes[1], nodes[2], nodes[3],
   };
   const struct stela_air_stitch_graph pointing = {"pointing", addressing, 4,
                                                   STELA_AIR_STITCH_NO_OUTPUT, true};
   assert(!stela_metal_stitch(sources, 2, &pointing, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "no buffer pointer"));
   LLVMDisposeMemoryBuffer(kept);
   LLVMDisposeMemoryBuffer(source);
}

static const char *early_outs =
   "define i1 @outside(i32 %x) {\n%o = icmp ugt i32 %x, 7\nret i1 %o\n}\n"
   "define void @mark(ptr addrspace(1) %data) {\n"
   "store i32 1, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "!air.visible = !{!1, !4}\n"
   "!1 = !{ptr @outside, !2, !3}\n!2 = !{}\n!3 = !{!30}\n"
   "!30 = !{i32 0, !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"x\"}\n"
   "!4 = !{ptr @mark, !2, !5}\n!5 = !{!50}\n"
   "!50 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n";

static const char *guarding_kernel =
   "declare void @guarded.MTL_VISIBLE_FN_REF(i32, ptr addrspace(1)) section \"air.externally_defined\"\n"
   "define void @guards(ptr addrspace(1) %data, i32 %tid) {\n"
   "call void @guarded.MTL_VISIBLE_FN_REF(i32 %tid, ptr addrspace(1) %data)\nret void\n}\n"
   "!air.kernel = !{!2}\n!air.visible_function_references = !{!5}\n"
   "!2 = !{ptr @guards, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!5 = !{!\"air.visible_function_reference\", ptr @guarded.MTL_VISIBLE_FN_REF, !\"guarded\"}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* guarded(x, data) { if (outside(x)) return; mark(data); }: an early return
 * branches past the calls after it, in the kernel it inlines into too. */
static void
stitched_early_returns_skip_what_follows(void)
{
   LLVMMemoryBufferRef source = bitcode(early_outs);
   const struct stela_air_stitch_source sources[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "outside"},
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "mark"},
   };
   const uint32_t x = 0, data = 3;
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_INPUT, .input = 0},
      {.function = 0, .arguments = &x, .argument_count = 1},
      {.function = STELA_AIR_STITCH_EARLY_RETURN, .input = 1},
      {.function = STELA_AIR_STITCH_INPUT, .input = 1},
      {.function = 1, .arguments = &data, .argument_count = 1},
   };
   const struct stela_air_stitch_graph graph = {"guarded", nodes, 5, STELA_AIR_STITCH_NO_OUTPUT};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   assert(stela_metal_stitch(sources, 2, &graph, 1, keep_stitched, &stitched, &error));
   LLVMContextRef context = LLVMContextCreate();
   LLVMModuleRef module;
   assert(!LLVMParseBitcodeInContext2(context, stitched, &module));
   LLVMValueRef guarded = LLVMGetNamedFunction(module, "guarded");
   assert(guarded && LLVMCountBasicBlocks(guarded) == 3);
   LLVMDisposeModule(module);
   LLVMContextDispose(context);
   const struct stela_air_module modules[] = {
      {LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched)},
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source)},
   };
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 2,
   };
   nir_shader *shader = compile(guarding_kernel, "guards", &specialization, &error);
   assert(shader && count_intrinsic(shader, nir_intrinsic_store_global) == 1);
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(stitched);

   /* Its condition is an earlier call's result, and it is no argument. */
   const struct stela_air_stitch_node unconditioned[] = {
      {.function = STELA_AIR_STITCH_INPUT, .input = 0},
      {.function = STELA_AIR_STITCH_EARLY_RETURN, .input = 0},
      {.function = 0, .arguments = &x, .argument_count = 1},
   };
   const struct stela_air_stitch_graph inputs = {"inputs", unconditioned, 3, STELA_AIR_STITCH_NO_OUTPUT};
   assert(!stela_metal_stitch(sources, 2, &inputs, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "no earlier call"));
   const uint32_t early = 2;
   const struct stela_air_stitch_node passing[] = {
      nodes[0], nodes[1], nodes[2], {.function = 1, .arguments = &early, .argument_count = 1},
   };
   const struct stela_air_stitch_graph passed = {"passed", passing, 4, STELA_AIR_STITCH_NO_OUTPUT};
   assert(!stela_metal_stitch(sources, 2, &passed, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "is a call's argument"));
   const struct stela_air_stitch_graph outputting = {"outputting", nodes, 3, 1};
   assert(!stela_metal_stitch(sources, 2, &outputting, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "other than the output's values"));

   /* A function with an output returns a call's value of the output's type:
    * valued(x) { if (outside(x)) return outside(x); return outside(x); }. */
   const uint32_t value = 1;
   const struct stela_air_stitch_node valuing[] = {
      nodes[0], nodes[1],
      {.function = STELA_AIR_STITCH_EARLY_RETURN, .input = 1, .arguments = &value, .argument_count = 1},
      nodes[1],
   };
   const struct stela_air_stitch_graph valued = {"valued", valuing, 4, 3};
   assert(stela_metal_stitch(sources, 2, &valued, 1, keep_stitched, &stitched, &error));
   LLVMDisposeMemoryBuffer(stitched);
   const struct stela_air_stitch_graph untyped = {"untyped", valuing, 3, 0};
   assert(!stela_metal_stitch(sources, 2, &untyped, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "no earlier call of the output's type"));

   /* A two-dimensional kernel reads a thread position after an early
    * return, as Core Image's graphs do: the position, whose running shape
    * needs the dispatch's grid, is read in the prelude.
    * kernel late() { if (outside(grid.x)) return; kept(buffer 0, local.x); } */
   LLVMMemoryBufferRef kept = bitcode(keeps);
   const struct stela_air_stitch_source late_sources[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "outside"},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept), "kept"},
   };
   const uint32_t grid = 0, condition = 1, stored[] = {4, 3};
   const struct stela_air_stitch_node late[] = {
      {.function = STELA_AIR_STITCH_BUILTIN, .input = STELA_AIR_STITCH_THREAD_POSITION_IN_GRID},
      {.function = 0, .arguments = &grid, .argument_count = 1},
      {.function = STELA_AIR_STITCH_EARLY_RETURN, .input = condition},
      {.function = STELA_AIR_STITCH_BUILTIN, .input = STELA_AIR_STITCH_THREAD_POSITION_IN_THREADGROUP},
      {.function = STELA_AIR_STITCH_BUFFER_ADDRESS, .input = 0},
      {.function = 1, .arguments = stored, .argument_count = 2},
   };
   const struct stela_air_stitch_graph lately = {"late", late, 6, STELA_AIR_STITCH_NO_OUTPUT, true};
   assert(stela_metal_stitch(late_sources, 2, &lately, 1, keep_stitched, &stitched, &error));
   const struct stela_air_module late_modules[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source)},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept)},
   };
   const struct stela_metal_specialization square = {
      .workgroup_size = {8, 8, 1}, .modules = late_modules, .module_count = 2,
   };
   shader = stela_metal_compile(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched), "late",
                                &target, &square, &error);
   if (!shader)
      fprintf(stderr, "late: %u %s\n", error.kind, error.detail);
   assert(shader && count_intrinsic(shader, nir_intrinsic_store_global) == 1);
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(stitched);
   LLVMDisposeMemoryBuffer(kept);
   LLVMDisposeMemoryBuffer(source);
}

static const char *samples =
   "declare { <4 x float>, i8 } @air.sample_texture_2d.v4f32(ptr addrspace(1), ptr addrspace(2),"
   " <2 x float>, i1, <2 x i32>, i1, float, float, i32)\n"
   "define <4 x float> @sampled(ptr addrspace(1) %t, ptr addrspace(2) %s) {\n"
   "%r = call { <4 x float>, i8 } @air.sample_texture_2d.v4f32(ptr addrspace(1) %t,"
   " ptr addrspace(2) %s, <2 x float> <float 0.5, float 0.5>, i1 true, <2 x i32> zeroinitializer,"
   " i1 false, float 0.0, float 0.0, i32 0)\n"
   "%v = extractvalue { <4 x float>, i8 } %r, 0\nret <4 x float> %v\n}\n"
   "!air.visible = !{!71}\n!71 = !{ptr @sampled, !72, !73}\n"
   "!72 = !{!74}\n!74 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"float4\"}\n"
   "!73 = !{!75, !76}\n"
   "!75 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"__metal_texture_2d_t\","
   " !\"air.arg_name\", !\"t\"}\n"
   "!76 = !{i32 1, !\"air.visible_input\", !\"air.arg_type_name\", !\"sampler\","
   " !\"air.arg_name\", !\"s\"}\n";

/* kernel bound() { kept_float4(buffer 0, sampled(texture 3, sampler 1)); }
 * samples what it binds there, the texture's type as the sample call names
 * it, which its reflection lists as Metal's does. */
/* The texture samples a shader makes: each sample call's, and the one tap
 * of the loop its bicubic branch runs for samplers with Metal's bicubic
 * filter, once for the nearest texel or sixteen times for weighed ones,
 * and the two of the branch two-plane textures take, one per plane. */
#define BICUBIC_TAPS 1
#define PLANE_SAMPLES 2
static unsigned
sample_count(nir_shader *shader)
{
   unsigned count = 0;
   nir_foreach_function_impl(impl, shader) {
      nir_foreach_block(block, impl) {
         nir_foreach_instr(instr, block) {
            if (instr->type != nir_instr_type_tex)
               continue;
            nir_texop op = nir_instr_as_tex(instr)->op;
            count += op == nir_texop_tex || op == nir_texop_txb || op == nir_texop_txl ||
                     op == nir_texop_txd;
         }
      }
   }
   return count;
}

static void
stitched_textures_and_samplers_read_their_kernels_bindings(void)
{
   LLVMMemoryBufferRef source = bitcode(samples), kept = bitcode(keeps);
   const struct stela_air_stitch_source sources[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "sampled"},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept), "kept_float4"},
   };
   const uint32_t resources[] = {0, 1}, stored[] = {3, 2};
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_TEXTURE, .input = 3},
      {.function = STELA_AIR_STITCH_SAMPLER, .input = 1},
      {.function = 0, .arguments = resources, .argument_count = 2},
      {.function = STELA_AIR_STITCH_BUFFER_ADDRESS, .input = 0},
      {.function = 1, .arguments = stored, .argument_count = 2},
   };
   const struct stela_air_stitch_graph graph = {"bound", nodes, 5, STELA_AIR_STITCH_NO_OUTPUT,
                                                true};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   assert(stela_metal_stitch(sources, 2, &graph, 1, keep_stitched, &stitched, &error));
   struct stela_metal_reflection reflection;
   assert(stela_metal_reflect(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched), "bound",
                              NULL, 0, &reflection, &error));
   assert(reflection.texture_count == 1 && reflection.textures[0].index == 3 &&
          reflection.textures[0].type == STELA_AIR_TEXTURE_2D &&
          reflection.textures[0].access == STELA_METAL_ACCESS_SAMPLE &&
          reflection.textures[0].primitive == STELA_AIR_FLOAT);
   assert(reflection.sampler_count == 1 && reflection.samplers[0] == 1);
   assert(reflection.buffer_count == 1 && reflection.buffers[0].index == 0);
   /* Its sampler is bound at index 1, whose samples take the bicubic path
    * only in a variant for draws or dispatches that bind a bicubic sampler
    * there: not for a process that has made bicubic samplers, nor for one
    * bound at another index. */
   const uint32_t plain[] = {0, STELA_METAL_BICUBIC_IDS, 1u << 0 | 1u << 2};
   for (unsigned i = 0; i < ARRAY_SIZE(plain); i++) {
      stitched_bicubic = plain[i];
      nir_shader *shader = compile_stitched(stitched, "bound", source, kept, NULL, 0, &error);
      assert(shader);
      assert(sample_count(shader) == 1 + PLANE_SAMPLES);
      stela_metal_release(shader);
   }
   stitched_bicubic = 1u << 1;
   nir_shader *shader = compile_stitched(stitched, "bound", source, kept, NULL, 0, &error);
   stitched_bicubic = 0;
   assert(shader);
   assert(sample_count(shader) == 1 + BICUBIC_TAPS + PLANE_SAMPLES);
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(stitched);

   /* A texture feeds a texture parameter, a sampler a sampler one. */
   const uint32_t swapped[] = {1, 0};
   const struct stela_air_stitch_node crossed[] = {
      nodes[0], nodes[1], {.function = 0, .arguments = swapped, .argument_count = 2},
      nodes[3], nodes[4],
   };
   const struct stela_air_stitch_graph crossing = {"crossing", crossed, 5,
                                                   STELA_AIR_STITCH_NO_OUTPUT, true};
   assert(!stela_metal_stitch(sources, 2, &crossing, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "no texture"));
   LLVMDisposeMemoryBuffer(kept);
   LLVMDisposeMemoryBuffer(source);
}

/* Core Image's stitchable filters: _ci_sampler_builder packs the texture
 * and sampler into its Sampler struct of Metal's one-pointer wrappers, and
 * the filter kernel takes the struct's fields by value and passes them to a
 * helper that extracts the pointers (ci_filters_stitchable's _lanczosDownH
 * and coreimage::sample). */
static const char *wrapped_samples =
   "%T = type { ptr addrspace(1) }\n%S = type { ptr addrspace(2) }\n%B = type { %T, %S }\n"
   "declare { <4 x float>, i8 } @air.sample_texture_2d.v4f32(ptr addrspace(1), ptr addrspace(2),"
   " <2 x float>, i1, <2 x i32>, i1, float, float, i32)\n"
   "define %B @builder(ptr addrspace(1) %t, ptr addrspace(2) %s) {\n"
   "%tw = insertvalue %T undef, ptr addrspace(1) %t, 0\n"
   "%sw = insertvalue %S undef, ptr addrspace(2) %s, 0\n"
   "%b0 = insertvalue %B undef, %T %tw, 0\n%b1 = insertvalue %B %b0, %S %sw, 1\nret %B %b1\n}\n"
   "define internal <4 x float> @helper(%T %t, %S %s) {\n"
   "%tp = extractvalue %T %t, 0\n%sp = extractvalue %S %s, 0\n"
   "%r = call { <4 x float>, i8 } @air.sample_texture_2d.v4f32(ptr addrspace(1) %tp,"
   " ptr addrspace(2) %sp, <2 x float> <float 0.5, float 0.5>, i1 true, <2 x i32> zeroinitializer,"
   " i1 false, float 0.0, float 0.0, i32 0)\n"
   "%v = extractvalue { <4 x float>, i8 } %r, 0\nret <4 x float> %v\n}\n"
   "define <4 x float> @sampled(%T %t, %S %s) {\n"
   "%v = call <4 x float> @helper(%T %t, %S %s)\nret <4 x float> %v\n}\n"
   "!air.visible = !{!71, !81}\n!71 = !{ptr @sampled, !72, !73, !90}\n"
   "!72 = !{!74}\n!74 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"float4\"}\n"
   "!73 = !{!75, !76}\n"
   "!75 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"texture2d<float, sample>\","
   " !\"air.arg_name\", !\"t\"}\n"
   "!76 = !{i32 1, !\"air.visible_input\", !\"air.arg_type_name\", !\"sampler\","
   " !\"air.arg_name\", !\"s\"}\n"
   "!81 = !{ptr @builder, !82, !83}\n"
   "!82 = !{!84}\n!84 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"Sampler\"}\n"
   "!83 = !{!85, !86}\n"
   "!85 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"texture2d<float, sample>\","
   " !\"air.arg_name\", !\"t\"}\n"
   "!86 = !{i32 1, !\"air.visible_input\", !\"air.arg_type_name\", !\"sampler\","
   " !\"air.arg_name\", !\"s\"}\n"
   "!90 = !{!\"air.stitching_info\", !91, !92}\n"
   "!91 = !{!\"air.stitching_type\", !93}\n"
   "!93 = !{!\"air.vector_type\", i32 16, i32 16, null, !94, i32 4}\n"
   "!94 = !{!\"air.float_type\", i32 4, i32 4, null}\n"
   "!92 = !{!\"air.stitching_argument\", !95, !\"src\"}\n!95 = !{!\"air.stitching_type\", !96}\n"
   "!96 = !{!\"air.struct_type\", i32 16, i32 8, null, !\"Sampler\", !97}\n!97 = !{!98, !99}\n"
   "!98 = !{!\"air.record_field\", i32 0, i32 8, !100, !\"t\", null}\n"
   "!100 = !{!\"air.texture_2d_type\", i32 8, i32 8, null, !94, !\"sample\"}\n"
   "!99 = !{!\"air.record_field\", i32 8, i32 8, !101, !\"s\", null}\n"
   "!101 = !{!\"air.sampler_type\", i32 8, i32 8, null}\n";

/* The kernel of stitched_textures_and_samplers_read_their_kernels_bindings
 * over wrapped_samples samples its bindings as well. */
static void
stitched_wrapped_textures_and_samplers_read_their_kernels_bindings(void)
{
   LLVMMemoryBufferRef source = bitcode(wrapped_samples), kept = bitcode(keeps);
   const struct stela_air_stitch_source sources[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "builder"},
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "sampled"},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept), "kept_float4"},
   };
   const uint32_t resources[] = {0, 1}, built[] = {2}, stored[] = {4, 3};
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_TEXTURE, .input = 3},
      {.function = STELA_AIR_STITCH_SAMPLER, .input = 1},
      {.function = 0, .arguments = resources, .argument_count = 2},
      {.function = 1, .arguments = built, .argument_count = 1},
      {.function = STELA_AIR_STITCH_BUFFER_ADDRESS, .input = 0},
      {.function = 2, .arguments = stored, .argument_count = 2},
   };
   const struct stela_air_stitch_graph graph = {"bound", nodes, 6, STELA_AIR_STITCH_NO_OUTPUT,
                                                true};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   if (!stela_metal_stitch(sources, 3, &graph, 1, keep_stitched, &stitched, &error))
      fprintf(stderr, "wrapped stitch: %u %s\n", error.kind, error.detail);
   assert(stitched);
   nir_shader *shader = compile_stitched(stitched, "bound", source, kept, NULL, 0, &error);
   if (!shader)
      fprintf(stderr, "wrapped: %u %s\n", error.kind, error.detail);
   assert(shader);
   assert(sample_count(shader) == 1 + PLANE_SAMPLES);
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(stitched);
   LLVMDisposeMemoryBuffer(kept);
   LLVMDisposeMemoryBuffer(source);
}

static const char *shares =
   "define float @shared(ptr addrspace(3) %m) {\n"
   "store float 1.0, ptr addrspace(3) %m, align 4\n"
   "%v = load float, ptr addrspace(3) %m, align 4\nret float %v\n}\n"
   "!air.visible = !{!71}\n!71 = !{ptr @shared, !72, !73}\n"
   "!72 = !{!74}\n!74 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"float\"}\n"
   "!73 = !{!75}\n"
   "!75 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"float\","
   " !\"air.arg_name\", !\"m\"}\n";

/* wrapped_samples split between two libraries, as Core Image's
 * _ci_sampler_builder (ci_stdlib_stitchable) and its filters
 * (ci_filters_stitchable) are: each module names its own copy of Metal's
 * wrapper structs, so the stitched kernel calls the filter with a type
 * linking keeps distinct from the filter's own, and the call still inlines
 * into texture calls of the kernel's binding. */
static const char *split_builder =
   "%\"T.3\" = type { ptr addrspace(1) }\n%\"S.3\" = type { ptr addrspace(2) }\n"
   "%\"B.3\" = type { %\"T.3\", %\"S.3\" }\n"
   "define internal %\"B.3\" @pack(ptr addrspace(1) %t, ptr addrspace(2) %s) {\n"
   "%tw = insertvalue %\"T.3\" undef, ptr addrspace(1) %t, 0\n"
   "%sw = insertvalue %\"S.3\" undef, ptr addrspace(2) %s, 0\n"
   "%b0 = insertvalue %\"B.3\" undef, %\"T.3\" %tw, 0\n"
   "%b1 = insertvalue %\"B.3\" %b0, %\"S.3\" %sw, 1\nret %\"B.3\" %b1\n}\n"
   "define %\"B.3\" @builder(ptr addrspace(1) %t, ptr addrspace(2) %s) {\n"
   "%b = call %\"B.3\" @pack(ptr addrspace(1) %t, ptr addrspace(2) %s)\nret %\"B.3\" %b\n}\n"
   "!air.visible = !{!81}\n"
   "!81 = !{ptr @builder, !82, !83}\n"
   "!82 = !{!84}\n!84 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"Sampler\"}\n"
   "!83 = !{!85, !86}\n"
   "!85 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"__metal_texture_2d_t\","
   " !\"air.arg_name\", !\"t\"}\n"
   "!86 = !{i32 1, !\"air.visible_input\", !\"air.arg_type_name\", !\"__metal_sampler_t\","
   " !\"air.arg_name\", !\"s\"}\n";

static const char *split_filter =
   "%\"T.127\" = type { ptr addrspace(1) }\n%\"S.127\" = type { ptr addrspace(2) }\n"
   "declare { <4 x float>, i8 } @air.sample_texture_2d.v4f32(ptr addrspace(1), ptr addrspace(2),"
   " <2 x float>, i1, <2 x i32>, i1, float, float, i32)\n"
   "define internal <4 x float> @helper(%\"T.127\" %t, %\"S.127\" %s) {\n"
   "%tp = extractvalue %\"T.127\" %t, 0\n%sp = extractvalue %\"S.127\" %s, 0\n"
   "%r = call { <4 x float>, i8 } @air.sample_texture_2d.v4f32(ptr addrspace(1) %tp,"
   " ptr addrspace(2) %sp, <2 x float> <float 0.5, float 0.5>, i1 true, <2 x i32> zeroinitializer,"
   " i1 false, float 0.0, float 0.0, i32 0)\n"
   "%v = extractvalue { <4 x float>, i8 } %r, 0\nret <4 x float> %v\n}\n"
   "define <4 x float> @sampled(%\"T.127\" %t, %\"S.127\" %s) {\n"
   "%v = call <4 x float> @helper(%\"T.127\" %t, %\"S.127\" %s)\nret <4 x float> %v\n}\n"
   "!air.visible = !{!71}\n!71 = !{ptr @sampled, !72, !73, !90}\n"
   "!72 = !{!74}\n!74 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"float4\"}\n"
   "!73 = !{!75, !76}\n"
   "!75 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"texture2d<float, sample>\","
   " !\"air.arg_name\", !\"t\"}\n"
   "!76 = !{i32 1, !\"air.visible_input\", !\"air.arg_type_name\", !\"sampler\","
   " !\"air.arg_name\", !\"s\"}\n"
   "!90 = !{!\"air.stitching_info\", !91, !92}\n"
   "!91 = !{!\"air.stitching_type\", !93}\n"
   "!93 = !{!\"air.vector_type\", i32 16, i32 16, null, !94, i32 4}\n"
   "!94 = !{!\"air.float_type\", i32 4, i32 4, null}\n"
   "!92 = !{!\"air.stitching_argument\", !95, !\"src\"}\n!95 = !{!\"air.stitching_type\", !96}\n"
   "!96 = !{!\"air.struct_type\", i32 16, i32 8, null, !\"Sampler\", !97}\n!97 = !{!98, !99}\n"
   "!98 = !{!\"air.record_field\", i32 0, i32 8, !100, !\"t\", null}\n"
   "!100 = !{!\"air.texture_2d_type\", i32 8, i32 8, null, !94, !\"sample\"}\n"
   "!99 = !{!\"air.record_field\", i32 8, i32 8, !101, !\"s\", null}\n"
   "!101 = !{!\"air.sampler_type\", i32 8, i32 8, null}\n";

static void
stitched_libraries_pass_their_own_struct_copies(void)
{
   LLVMMemoryBufferRef builder = bitcode(split_builder), filter = bitcode(split_filter),
                       kept = bitcode(keeps);
   const struct stela_air_stitch_source sources[] = {
      {LLVMGetBufferStart(builder), LLVMGetBufferSize(builder), "builder"},
      {LLVMGetBufferStart(filter), LLVMGetBufferSize(filter), "sampled"},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept), "kept_float4"},
   };
   const uint32_t resources[] = {0, 1}, built[] = {2}, stored[] = {4, 3};
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_TEXTURE, .input = 1},
      {.function = STELA_AIR_STITCH_SAMPLER, .input = 0},
      {.function = 0, .arguments = resources, .argument_count = 2},
      {.function = 1, .arguments = built, .argument_count = 1},
      {.function = STELA_AIR_STITCH_BUFFER_ADDRESS, .input = 0},
      {.function = 2, .arguments = stored, .argument_count = 2},
   };
   const struct stela_air_stitch_graph graph = {"split", nodes, 6, STELA_AIR_STITCH_NO_OUTPUT,
                                                true};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   if (!stela_metal_stitch(sources, 3, &graph, 1, keep_stitched, &stitched, &error))
      fprintf(stderr, "split stitch: %u %s\n", error.kind, error.detail);
   assert(stitched);
   const struct stela_air_module modules[] = {
      {LLVMGetBufferStart(builder), LLVMGetBufferSize(builder)},
      {LLVMGetBufferStart(filter), LLVMGetBufferSize(filter)},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept)},
   };
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 3,
   };
   nir_shader *shader = stela_metal_compile(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched),
                                            "split", &target, &specialization, &error);
   if (!shader)
      fprintf(stderr, "split: %u %s\n", error.kind, error.detail);
   assert(shader);
   assert(sample_count(shader) == 1 + PLANE_SAMPLES);
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(stitched);
   LLVMDisposeMemoryBuffer(kept);
   LLVMDisposeMemoryBuffer(filter);
   LLVMDisposeMemoryBuffer(builder);
}

/* kernel bound() { kept_float(buffer 0, shared(threadgroup 2)); } reads
 * and writes the threadgroup memory its dispatch sizes at that index, which
 * its reflection lists. */
static void
stitched_threadgroup_memory_is_its_kernels(void)
{
   LLVMMemoryBufferRef source = bitcode(shares), kept = bitcode(keeps);
   const struct stela_air_stitch_source sources[] = {
      {LLVMGetBufferStart(source), LLVMGetBufferSize(source), "shared"},
      {LLVMGetBufferStart(kept), LLVMGetBufferSize(kept), "kept_float"},
   };
   const uint32_t memory[] = {0}, stored[] = {2, 1};
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_THREADGROUP, .input = 2},
      {.function = 0, .arguments = memory, .argument_count = 1},
      {.function = STELA_AIR_STITCH_BUFFER_ADDRESS, .input = 0},
      {.function = 1, .arguments = stored, .argument_count = 2},
   };
   const struct stela_air_stitch_graph graph = {"bound", nodes, 4, STELA_AIR_STITCH_NO_OUTPUT,
                                                true};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   assert(stela_metal_stitch(sources, 2, &graph, 1, keep_stitched, &stitched, &error));
   struct stela_metal_reflection reflection;
   assert(stela_metal_reflect(LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched), "bound",
                              NULL, 0, &reflection, &error));
   assert(reflection.threadgroup_count == 1 && reflection.threadgroup[0].index == 2);
   assert(reflection.buffer_count == 1 && reflection.buffers[0].index == 0);
   const struct stela_metal_threadgroup_length lengths[] = {{2, 128}};
   nir_shader *shader = compile_stitched(stitched, "bound", source, kept, lengths, 1, &error);
   assert(shader && shader->info.shared_size >= 128);
   stela_metal_release(shader);

   /* Its dispatch sizes it, as a kernel's threadgroup buffer. */
   assert(!compile_stitched(stitched, "bound", source, kept, NULL, 0, &error) &&
          strstr(error.detail, "threadgroup buffer 2"));
   LLVMDisposeMemoryBuffer(stitched);

   /* Threadgroup memory feeds only a threadgroup pointer. */
   const struct stela_air_stitch_node texture[] = {
      {.function = STELA_AIR_STITCH_TEXTURE, .input = 2}, nodes[1], nodes[2], nodes[3],
   };
   const struct stela_air_stitch_graph crossing = {"crossing", texture, 4,
                                                   STELA_AIR_STITCH_NO_OUTPUT, true};
   assert(!stela_metal_stitch(sources, 2, &crossing, 1, keep_stitched, &stitched, &error) &&
          strstr(error.detail, "no texture"));
   LLVMDisposeMemoryBuffer(kept);
   LLVMDisposeMemoryBuffer(source);
}

/* compile() with the modules and linked functions of a pipeline. */
static nir_shader *
compile_linked(const char *fixture, const char *entry, const char *const *modules,
               unsigned module_count, const char *const *functions, unsigned function_count,
               struct stela_air_error *error)
{
   LLVMMemoryBufferRef buffers[2];
   struct stela_air_module linked_modules[2];
   struct stela_air_linked_function linked_functions[4];
   assert(module_count <= 2 && function_count <= 4);
   for (unsigned i = 0; i < module_count; i++) {
      buffers[i] = bitcode(modules[i]);
      linked_modules[i] = (struct stela_air_module){LLVMGetBufferStart(buffers[i]),
                                                    LLVMGetBufferSize(buffers[i])};
   }
   for (unsigned i = 0; i < function_count; i++)
      linked_functions[i] = (struct stela_air_linked_function){functions[i], strlen(functions[i])};
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .modules = linked_modules, .module_count = module_count,
      .linked_functions = linked_functions, .linked_function_count = function_count,
   };
   nir_shader *shader = compile(fixture, entry, &specialization, error);
   for (unsigned i = 0; i < module_count; i++)
      LLVMDisposeMemoryBuffer(buffers[i]);
   return shader;
}

/* Calls through visible function tables dispatch over the linked functions,
 * and external functions resolve to the linked modules'. */
static void
linked_functions_resolve_tables_and_externals(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(tables, "tables", &reflection, &error));
   assert(reflection.function_tables == 1 && reflection.buffer_count == 2);
   /* A visible function reflects as no stage, and compiles only linked. */
   assert(reflect(tables, "doubled", &reflection, &error));
   assert(reflection.stage == STELA_METAL_VISIBLE && !reflection.buffer_count);
   const struct stela_metal_specialization none = {0};
   assert(!compile(tables, "doubled", &none, &error) && error.kind == STELA_AIR_ARGUMENT);

   const char *functions[] = {"doubled", "negated", "wide"};
   nir_shader *shader = compile_linked(tables, "tables", &linked, 1, functions, 3, &error);
   if (!shader)
      fprintf(stderr, "tables: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);
   shader = compile_linked(recast, "tables", &linked, 1, functions, 3, &error);
   assert(shader);
   stela_metal_release(shader);
   shader = compile_linked(sized, "tables", &linked, 1, functions, 3, &error);
   assert(shader);
   stela_metal_release(shader);
   /* A library deployed to another version of the same target links. */
   static const char *redeployed_linked =
      "target triple = \"air64_v29-apple-macosx27.0.0\"\n"
      "define i32 @external(i32 %x) {\n%y = add i32 %x, 7\nret i32 %y\n}\n"
      "define i32 @negated(i32 %x) {\n%y = sub i32 0, %x\nret i32 %y\n}\n"
      "define i64 @wide(i64 %x) {\nret i64 %x\n}\n"
      "!air.visible = !{}\n";
   shader = compile_linked(tables, "tables", &redeployed_linked, 1, functions, 3, &error);
   if (!shader)
      fprintf(stderr, "redeployed: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);
   /* A library another build of the compiler made links too: an
    * offline-built dynamic library under a kernel compiled at run time. */
   {
      static const char *other_build =
         "!air.version = !{!20}\n!air.language_version = !{!21}\n!llvm.ident = !{!22}\n"
         "!air.compile_options = !{!23, !24, !25}\n"
         "!20 = !{i32 2, i32 9, i32 0}\n!21 = !{!\"Metal\", i32 4, i32 1, i32 0}\n"
         "!22 = !{!\"Apple metal version 32023.830 (metalfe-32023.830.2)\"}\n"
         "!23 = !{!\"air.compile.denorms_disable\"}\n!24 = !{!\"air.compile.fast_math_enable\"}\n"
         "!25 = !{!\"air.compile.framebuffer_fetch_enable\"}\n";
      LLVMMemoryBufferRef built = bitcode_built(linked, other_build);
      const struct stela_air_module modules[] = {{LLVMGetBufferStart(built), LLVMGetBufferSize(built)}};
      struct stela_air_linked_function named[3];
      for (unsigned i = 0; i < 3; i++)
         named[i] = (struct stela_air_linked_function){functions[i], strlen(functions[i])};
      const struct stela_metal_specialization specialization = {
         .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 1,
         .linked_functions = named, .linked_function_count = 3,
      };
      shader = compile(tables, "tables", &specialization, &error);
      if (!shader)
         fprintf(stderr, "other build: %u %s\n", error.kind, error.detail);
      assert(shader);
      stela_metal_release(shader);
      LLVMDisposeMemoryBuffer(built);
   }
   /* One an older toolchain built for an older AIR and language links
    * under one the system compiles, and the reverse; one for another system
    * does not. */
   {
      static const char *older =
         "!air.version = !{!20}\n!air.language_version = !{!21}\n"
         "!air.compile_options = !{!23, !24, !25}\n"
         "!20 = !{i32 2, i32 8, i32 0}\n!21 = !{!\"Metal\", i32 3, i32 2, i32 0}\n"
         "!23 = !{!\"air.compile.denorms_disable\"}\n!24 = !{!\"air.compile.fast_math_enable\"}\n"
         "!25 = !{!\"air.compile.framebuffer_fetch_enable\"}\n";
      const char *triples[] = {"air64_v28-apple-macosx27.0.0", "air64_v29-apple-ios27.0.0"};
      for (unsigned t = 0; t < 2; t++) {
         size_t bytes = strlen(linked) + 128;
         char *source = malloc(bytes);
         snprintf(source, bytes, "target triple = \"%s\"\n%s", triples[t], linked);
         LLVMMemoryBufferRef built = bitcode_built(source, older);
         const struct stela_air_module modules[] = {{LLVMGetBufferStart(built), LLVMGetBufferSize(built)}};
         struct stela_air_linked_function named[3];
         for (unsigned i = 0; i < 3; i++)
            named[i] = (struct stela_air_linked_function){functions[i], strlen(functions[i])};
         const struct stela_metal_specialization specialization = {
            .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 1,
            .linked_functions = named, .linked_function_count = 3,
         };
         shader = compile(tables, "tables", &specialization, &error);
         if (t == 0) {
            if (!shader)
               fprintf(stderr, "older: %u %s\n", error.kind, error.detail);
            assert(shader);
            stela_metal_release(shader);
         } else {
            assert(!shader && error.kind == STELA_AIR_INPUT && strstr(error.detail, "compiled for"));
         }
         LLVMDisposeMemoryBuffer(built);
         free(source);
      }
      /* An older kernel links the system's newer library, and keeps its own
       * target, which its metadata is read by. */
      size_t bytes = strlen(tables) + 128;
      char *older_tables = malloc(bytes);
      snprintf(older_tables, bytes, "target triple = \"%s\"\n%s", triples[0], tables);
      LLVMMemoryBufferRef kernel = bitcode_built(older_tables, older), newer = bitcode(linked);
      const struct stela_air_module modules[] = {{LLVMGetBufferStart(newer), LLVMGetBufferSize(newer)}};
      struct stela_air_linked_function named[3];
      for (unsigned i = 0; i < 3; i++)
         named[i] = (struct stela_air_linked_function){functions[i], strlen(functions[i])};
      const struct stela_metal_specialization specialization = {
         .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 1,
         .linked_functions = named, .linked_function_count = 3,
      };
      shader = stela_metal_compile(LLVMGetBufferStart(kernel), LLVMGetBufferSize(kernel), "tables",
                                   &target, &specialization, &error);
      if (!shader)
         fprintf(stderr, "older kernel: %u %s\n", error.kind, error.detail);
      assert(shader);
      stela_metal_release(shader);
      LLVMDisposeMemoryBuffer(kernel);
      LLVMDisposeMemoryBuffer(newer);
      free(older_tables);
   }
   shader = compile_linked(tested, "tables", &linked, 1, functions, 3, &error);
   if (!shader)
      fprintf(stderr, "tested: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);

   /* Without the library, the external function is unresolved. */
   assert(!compile_linked(tables, "tables", NULL, 0, functions, 1, &error));
   assert(error.kind == STELA_AIR_INSTRUCTION && strstr(error.detail, "unresolved external function external"));
   /* A linked function no module defines. */
   const char *missing[] = {"doubled", "missing"};
   assert(!compile_linked(tables, "tables", &linked, 1, missing, 2, &error));
   assert(error.kind == STELA_AIR_ARGUMENT && strstr(error.detail, "not defined"));
   /* A table's pointer that escapes its calls. */
   assert(!compile_linked(escaping, "tables", &linked, 1, functions, 3, &error));
   assert(error.kind == STELA_AIR_INSTRUCTION && strstr(error.detail, "only called"));
   /* A library's kernel of the kernel's name is its own. */
   const char *sharing[] = {linked, "define void @tables() {\nret void\n}\n!air.kernel = !{!0}\n"
                                    "!0 = !{ptr @tables, !1, !1}\n!1 = !{}\n"};
   shader = compile_linked(tables, "tables", sharing, 2, functions, 3, &error);
   assert(shader);
   stela_metal_release(shader);
   /* A library that defines the kernel's own function again. */
   const char *twice[] = {linked, "define i32 @doubled(i32 %x) {\nret i32 %x\n}\n!air.visible = !{}\n"};
   assert(!compile_linked(tables, "tables", twice, 2, functions, 3, &error));
}

/* compile() of biased, linking offsets with values, or NULL for none. */
static nir_shader *
compile_biased(const struct stela_air_constant_value *offset, unsigned offset_count,
               struct stela_air_error *error)
{
   const uint32_t five = 5;
   const struct stela_air_constant_value bias = {0, &five, sizeof(five)};
   LLVMMemoryBufferRef buffer = bitcode(offsets);
   const struct stela_air_module module = {LLVMGetBufferStart(buffer), LLVMGetBufferSize(buffer),
                                           offset, offset_count};
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .constant_count = 1, .constants = &bias,
      .modules = &module, .module_count = 1,
   };
   nir_shader *shader = compile(biased, "biased", &specialization, error);
   LLVMDisposeMemoryBuffer(buffer);
   return shader;
}

/* A linked module takes its own function constant values, apart from the
 * function's. */
static void
linked_modules_specialize_their_function_constants(void)
{
   struct stela_air_error error;
   const uint32_t seven = 7;
   const struct stela_air_constant_value offset = {0, &seven, sizeof(seven)};
   nir_shader *shader = compile_biased(&offset, 1, &error);
   if (!shader)
      fprintf(stderr, "biased: %u %s\n", error.kind, error.detail);
   assert(shader && stores_constant(shader, 5 + 7));
   stela_metal_release(shader);

   assert(!compile_biased(NULL, 0, &error) && error.kind == STELA_AIR_INPUT &&
          strstr(error.detail, "have no values"));
   assert(!compile_biased(&offset, 0, &error) && error.kind == STELA_AIR_ARGUMENT &&
          strstr(error.detail, "required function constant"));
}

/* Kernels that encode indirect commands reflect so, and whether they may
 * set a command's barrier. */
static void
command_encoding_reflects_barriers(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(kernel, "kernel", &reflection, &error));
   assert(!reflection.commands && !reflection.command_barriers);
   assert(reflect(resets, "commands", &reflection, &error));
   assert(reflection.commands && !reflection.command_barriers);
   assert(reflect(barriers, "commands", &reflection, &error));
   assert(reflection.commands && reflection.command_barriers);
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(barriers, "commands", &specialization, &error);
   if (!shader)
      fprintf(stderr, "commands: %u %s\n", error.kind, error.detail);
   assert(shader);
   stela_metal_release(shader);
}

static void
put32(uint8_t *bytes, uint32_t value)
{
   memcpy(bytes, &value, sizeof(value));
}

static void
put64(uint8_t *bytes, uint64_t value)
{
   memcpy(bytes, &value, sizeof(value));
}

#define BASE 0x10000ull
#define TABLE 0x0u
#define DISPATCH 0x700u
#define OUT 0x800u

/* One invocation of the shaped kernel, as the local_index thread of
 * workgroup 1 of a dispatch whose arguments shape its threadgroups 4 by 2
 * and place its threadgroup buffer at 48 bytes: its effects. */
static unsigned
run_shaped(nir_shader *shader, uint32_t local_index, struct air_effect_store *trace,
           unsigned capacity)
{
   static uint8_t memory[0x900];
   uint8_t push[STELA_METAL_PUSH_BYTES] = {0};
   memset(memory, 0, sizeof(memory));
   put64(push + STELA_METAL_PUSH_TABLES, BASE + TABLE);
   put64(push + STELA_METAL_PUSH_ARGUMENTS, BASE + DISPATCH);
   put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES, BASE + OUT);
   const uint32_t shape[] = {4, 2, 1};
   for (unsigned i = 0; i < 3; i++)
      put32(memory + DISPATCH + STELA_METAL_DISPATCH_SHAPE + 4 * i, shape[i]);
   put32(memory + DISPATCH + STELA_METAL_DISPATCH_OFFSETS + 4, 48);
   const struct air_effect_environment environment = {
      .base = BASE, .bytes = memory, .size = sizeof(memory), .push = push,
      .push_size = sizeof(push), .local_index = local_index, .workgroup = {1, 0, 0},
   };
   struct air_effect_run run = air_effect_execute_in(shader, 100000, trace, capacity, &environment);
   if (run.status != AIR_EFFECT_COMPLETE)
      fprintf(stderr, "shaped: %s\n", run.detail);
   assert(run.status == AIR_EFFECT_COMPLETE);
   return run.stores;
}

static void
expect_store(const struct air_effect_store *store, bool shared, uint64_t address,
             const uint32_t *values, unsigned components)
{
   assert(store->shared == shared && store->address == address);
   assert(store->bits == 32 && store->components == components);
   for (unsigned i = 0; i < components; i++)
      assert(store->values[i].u32 == values[i]);
}

/* What sums stores, run as one invocation with its buffer at OUT. */
static uint32_t
run_summed(nir_shader *shader)
{
   static uint8_t memory[0x900];
   static uint8_t scratch[512];
   uint8_t push[STELA_METAL_PUSH_BYTES] = {0};
   memset(memory, 0, sizeof(memory));
   put64(push + STELA_METAL_PUSH_TABLES, BASE + TABLE);
   put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES, BASE + OUT);
   const struct air_effect_environment environment = {
      .base = BASE, .bytes = memory, .size = sizeof(memory), .push = push,
      .push_size = sizeof(push), .scratch = scratch, .scratch_size = sizeof(scratch),
   };
   struct air_effect_store store;
   struct air_effect_run run = air_effect_execute_in(shader, 100000, &store, 1, &environment);
   if (run.status != AIR_EFFECT_COMPLETE)
      fprintf(stderr, "sums: %s\n", run.detail);
   assert(run.status == AIR_EFFECT_COMPLETE && run.stores == 1 && store.address == BASE + OUT);
   return store.values[0].u32;
}

/* A thread array of N words: a fixed loop stores 3i + tid to word i, then
 * the word the buffer's first value names, masked into the array, replaces
 * that value. */
#define THREAD_ARRAY_KERNEL(N, MASK)                                                          \
   "define void @thread_array(ptr addrspace(1) %data, i32 %tid) {\n"                          \
   "entry:\n"                                                                                  \
   "%arr = alloca [" #N " x i32], align 4\n"                                                   \
   "br label %fill\n"                                                                          \
   "fill:\n"                                                                                   \
   "%i = phi i32 [0, %entry], [%next, %fill]\n"                                                \
   "%wide = zext i32 %i to i64\n"                                                              \
   "%slot = getelementptr inbounds [" #N " x i32], ptr %arr, i64 0, i64 %wide\n"               \
   "%triple = mul i32 %i, 3\n"                                                                 \
   "%value = add i32 %triple, %tid\n"                                                          \
   "store i32 %value, ptr %slot, align 4\n"                                                    \
   "%next = add i32 %i, 1\n"                                                                   \
   "%more = icmp ult i32 %next, " #N "\n"                                                      \
   "br i1 %more, label %fill, label %read\n"                                                   \
   "read:\n"                                                                                   \
   "%k = load i32, ptr addrspace(1) %data, align 4\n"                                          \
   "%m = and i32 %k, " #MASK "\n"                                                              \
   "%mw = zext i32 %m to i64\n"                                                                \
   "%at = getelementptr inbounds [" #N " x i32], ptr %arr, i64 0, i64 %mw\n"                   \
   "%x = load i32, ptr %at, align 4\n"                                                         \
   "store i32 %x, ptr addrspace(1) %data, align 4\n"                                           \
   "ret void\n}\n"                                                                             \
   "!air.kernel = !{!2}\n"                                                                     \
   "!2 = !{ptr @thread_array, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"                           \
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\"," \
   " !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"                      \
   " !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"data\"}\n"                       \
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","        \
   " !\"air.arg_name\", !\"tid\"}\n"

/* The value thread_array stores, run as one invocation whose buffer at OUT
 * starts with first. */
static uint32_t
run_thread_array(nir_shader *shader, uint32_t first)
{
   static uint8_t memory[0x900];
   static uint8_t scratch[512];
   uint8_t push[STELA_METAL_PUSH_BYTES] = {0};
   memset(memory, 0, sizeof(memory));
   memset(scratch, 0, sizeof(scratch));
   put64(push + STELA_METAL_PUSH_TABLES, BASE + TABLE);
   put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES, BASE + OUT);
   put32(memory + OUT, first);
   const struct air_effect_environment environment = {
      .base = BASE, .bytes = memory, .size = sizeof(memory), .push = push,
      .push_size = sizeof(push), .scratch = scratch, .scratch_size = sizeof(scratch),
   };
   struct air_effect_store store;
   struct air_effect_run run = air_effect_execute_in(shader, 100000, &store, 1, &environment);
   if (run.status != AIR_EFFECT_COMPLETE)
      fprintf(stderr, "thread array: %s\n", run.detail);
   assert(run.status == AIR_EFFECT_COMPLETE && run.stores == 1 && store.address == BASE + OUT);
   return store.values[0].u32;
}

/* A thread array of up to 16 words lives in registers, its dynamic index a
 * select over its words; a larger one stays in scratch. Both read the same
 * values. */
static void
small_thread_arrays_live_in_registers(void)
{
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   struct stela_air_error error;
   nir_shader *shader = compile(THREAD_ARRAY_KERNEL(16, 15), "thread_array", &specialization, &error);
   if (!shader)
      fprintf(stderr, "thread array: %u %s\n", error.kind, error.detail);
   assert(shader && !shader->scratch_size);
   assert(!count_intrinsic(shader, nir_intrinsic_load_scratch) &&
          !count_intrinsic(shader, nir_intrinsic_store_scratch));
   assert(!count_intrinsic(shader, nir_intrinsic_load_deref) &&
          !count_intrinsic(shader, nir_intrinsic_store_deref));
   for (uint32_t k = 0; k < 16; k++)
      assert(run_thread_array(shader, k) == 3 * k);
   stela_metal_release(shader);

   shader = compile(THREAD_ARRAY_KERNEL(17, 15), "thread_array", &specialization, &error);
   if (!shader)
      fprintf(stderr, "thread array: %u %s\n", error.kind, error.detail);
   assert(shader && shader->scratch_size > 16 * 4);
   assert(count_intrinsic(shader, nir_intrinsic_load_scratch));
   assert(run_thread_array(shader, 13) == 39);
   stela_metal_release(shader);
}

/* MPS's shapes: setup() returns a struct its callers take coerced to its
 * fields, constant() returns its function constant k in a struct through
 * sret, and added() and extracted() take such structs through pointers to
 * copies. llvm.compiler.used keeps a helper, as Apple's compiler keeps a
 * struct's stitching helpers, and each stitching type names its four
 * helpers (load, store, destroy and copy from a buffer) as Apple's do. */
#define TRAITS ", ptr @helper, ptr @helper, ptr @helper, null"
#define STITCH_STRUCTS                                                                        \
   "!300 = !{!\"air.int_type\", i32 4, i32 4, null}\n"                                         \
   "!301 = !{!\"air.vector_type\", i32 8, i32 8, null, !300, i32 2}\n"                         \
   "!302 = !{!\"air.struct_type\", i32 16, i32 8, null, !\"pair\", !303}\n!303 = !{!304, !305}\n" \
   "!304 = !{!\"air.record_field\", i32 0, i32 8, !301, !\"a\", null}\n"                        \
   "!305 = !{!\"air.record_field\", i32 8, i32 8, !301, !\"b\", null}\n"                        \
   "!306 = !{!\"air.struct_type\", i32 32, i32 16, null, !\"value\", !307}\n!307 = !{!308, !309}\n" \
   "!308 = !{!\"air.record_field\", i32 0, i32 16, !300, !\"v\", null}\n"                       \
   "!309 = !{!\"air.record_field\", i32 16, i32 16, !302, !\"p\", null}\n"                      \
   "!310 = !{!\"air.stitching_type\", !300" TRAITS "}\n"                                       \
   "!311 = !{!\"air.stitching_type\", !302" TRAITS "}\n"                                       \
   "!312 = !{!\"air.stitching_type\", !306" TRAITS "}\n"                                       \
   "!313 = !{!\"air.stitching_argument\", !310, !\"x\"}\n"                                     \
   "!314 = !{!\"air.stitching_argument\", !311, !\"p\"}\n"                                     \
   "!315 = !{!\"air.stitching_argument\", !312, !\"v\"}\n"
static const char *struct_values =
   "%struct.pair = type { <2 x i32>, <2 x i32> }\n%struct.value = type { i32, %struct.pair }\n"
   "@k = external addrspace(2) constant i32\n"
   "@llvm.compiler.used = appending global [1 x ptr] [ptr @helper], section \"llvm.metadata\"\n"
   "define internal void @helper() {\nret void\n}\n"
   "define %struct.pair @setup(i32 %x) {\n"
   "%v = insertelement <2 x i32> poison, i32 %x, i32 0\n"
   "%w = insertelement <2 x i32> %v, i32 %x, i32 1\n"
   "%p = insertvalue %struct.pair poison, <2 x i32> %w, 0\n"
   "%q = insertvalue %struct.pair %p, <2 x i32> %w, 1\nret %struct.pair %q\n}\n"
   "define void @constant(ptr sret(%struct.value) align 16 %r, <2 x i32> %a, <2 x i32> %b) {\n"
   "%k = load i32, ptr addrspace(2) @k, align 4\nstore i32 %k, ptr %r, align 16\nret void\n}\n"
   "define void @added(ptr sret(%struct.value) align 16 %r, ptr %p, ptr %q) {\n"
   "%x = load i32, ptr %p, align 16\n%y = load i32, ptr %q, align 16\n%s = add i32 %x, %y\n"
   "store i32 %s, ptr %r, align 16\nret void\n}\n"
   "define i32 @extracted(ptr %p) {\n%x = load i32, ptr %p, align 16\nret i32 %x\n}\n"
   "!air.visible = !{!320, !321, !322, !323}\n"
   "!air.function_constants = !{!330}\n"
   "!330 = !{ptr addrspace(2) @k, !\"int\", !\"k\", i32 0, i1 true}\n"
   "!320 = !{ptr @setup, !340, !341, !350}\n!321 = !{ptr @constant, !342, !343, !351}\n"
   "!322 = !{ptr @added, !342, !344, !352}\n!323 = !{ptr @extracted, !340, !345, !353}\n"
   "!340 = !{!346}\n!346 = !{!\"air.visible_output\", !\"air.arg_type_name\", !\"int\"}\n"
   "!342 = !{}\n"
   "!360 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"x\"}\n"
   "!361 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"value\"}\n"
   "!362 = !{i32 1, !\"air.visible_input\", !\"air.arg_type_name\", !\"int2\", !\"air.arg_name\", !\"a\"}\n"
   "!363 = !{i32 2, !\"air.visible_input\", !\"air.arg_type_name\", !\"int2\", !\"air.arg_name\", !\"b\"}\n"
   "!364 = !{i32 1, !\"air.visible_input\", !\"air.arg_type_name\", !\"value\", !\"air.arg_name\", !\"p\"}\n"
   "!365 = !{i32 2, !\"air.visible_input\", !\"air.arg_type_name\", !\"value\", !\"air.arg_name\", !\"q\"}\n"
   "!366 = !{i32 0, !\"air.visible_input\", !\"air.arg_type_name\", !\"value\", !\"air.arg_name\", !\"p\"}\n"
   "!341 = !{!360}\n!343 = !{!361, !362, !363}\n!344 = !{!361, !364, !365}\n!345 = !{!366}\n"
   "!350 = !{!\"air.stitching_info\", !311, !313}\n"
   "!351 = !{!\"air.stitching_info\", !312, !314}\n"
   "!352 = !{!\"air.stitching_info\", !312, !315, !315}\n"
   "!353 = !{!\"air.stitching_info\", !310, !315}\n" STITCH_STRUCTS;

static const char *summing_kernel =
   "declare i32 @sum.MTL_VISIBLE_FN_REF(i32) section \"air.externally_defined\"\n"
   "define void @sums(ptr addrspace(1) %data, i32 %tid) {\n"
   "%v = call i32 @sum.MTL_VISIBLE_FN_REF(i32 %tid)\n"
   "store i32 %v, ptr addrspace(1) %data, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n!air.visible_function_references = !{!5}\n"
   "!2 = !{ptr @sums, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!5 = !{!\"air.visible_function_reference\", ptr @sum.MTL_VISIBLE_FN_REF, !\"sum\"}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"int\", !\"air.arg_name\", !\"data\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* sum(x) = extracted(added(constant(setup(x)), constant(setup(x)))) with k
 * 1 and 2: two specializations of constant(), which Metal names apart, so
 * the sum is 3. kept() returns added()'s struct through sret as added()
 * does. */
static void
stitched_specializations_pass_structs_by_their_abi(void)
{
   LLVMMemoryBufferRef source = bitcode(struct_values);
   const uint32_t one = 1, two = 2;
   const struct stela_air_constant_value first = {0, &one, 4}, second = {0, &two, 4};
   const void *bytes = LLVMGetBufferStart(source);
   size_t size = LLVMGetBufferSize(source);
   const struct stela_air_stitch_source sources[] = {
      {bytes, size, "setup", &first, 1},    {bytes, size, "constant", &first, 1},
      {bytes, size, "constant", &second, 1}, {bytes, size, "added", &first, 1},
      {bytes, size, "extracted", &first, 1},
   };
   const uint32_t input = 0, setup = 1, added[] = {2, 3}, sum = 4;
   const struct stela_air_stitch_node nodes[] = {
      {.function = STELA_AIR_STITCH_INPUT},
      {.function = 0, .arguments = &input, .argument_count = 1},
      {.function = 1, .arguments = &setup, .argument_count = 1},
      {.function = 2, .arguments = &setup, .argument_count = 1},
      {.function = 3, .arguments = added, .argument_count = 2},
      {.function = 4, .arguments = &sum, .argument_count = 1},
   };
   const struct stela_air_stitch_graph graphs[] = {{"sum", nodes, 6, 5}, {"kept", nodes, 5, 4}};
   struct stela_air_error error;
   LLVMMemoryBufferRef stitched = NULL;
   if (!stela_metal_stitch(sources, 5, graphs, 2, keep_stitched, &stitched, &error))
      fprintf(stderr, "struct values: %u %s\n", error.kind, error.detail);
   assert(stitched);

   /* kept() returns through its first parameter, an sret of added()'s. */
   LLVMContextRef context = LLVMContextCreate();
   LLVMModuleRef module;
   LLVMMemoryBufferRef copy = LLVMCreateMemoryBufferWithMemoryRangeCopy(
      LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched), "stitched");
   assert(!LLVMParseBitcodeInContext2(context, copy, &module));
   LLVMDisposeMemoryBuffer(copy);
   LLVMValueRef kept = LLVMGetNamedFunction(module, "kept");
   unsigned sret = LLVMGetEnumAttributeKindForName("sret", 4);
   assert(kept && LLVMCountParams(kept) == 2 && LLVMGetEnumAttributeAtIndex(kept, 1, sret));
   LLVMDisposeModule(module);
   LLVMContextDispose(context);

   /* Each specialization links from its own copy of the module. */
   const struct stela_air_module modules[] = {
      {LLVMGetBufferStart(stitched), LLVMGetBufferSize(stitched)},
      {bytes, size, &first, 1, 0},
      {bytes, size, &second, 1, 1},
   };
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .modules = modules, .module_count = 3,
   };
   nir_shader *shader = compile(summing_kernel, "sums", &specialization, &error);
   if (!shader)
      fprintf(stderr, "sums: %u %s\n", error.kind, error.detail);
   assert(shader);
   assert(run_summed(shader) == 3);
   stela_metal_release(shader);
   LLVMDisposeMemoryBuffer(stitched);

   LLVMDisposeMemoryBuffer(source);
}

/* gathers() stores its thread's position to the third buffer of an
 * array_ref of three bound from index 1: a thread pointer to their
 * addresses. */
static const char *gathers =
   "declare void @llvm.experimental.noalias.scope.decl(metadata)\n"
   "define void @gathers(ptr %buffers, i32 %tid) {\n"
   "%g = addrspacecast ptr %buffers to ptr addrspace(8)\n"
   "%e = getelementptr inbounds ptr addrspace(1), ptr addrspace(8) %g, i64 2\n"
   "%b = load ptr addrspace(1), ptr addrspace(8) %e, align 8\n"
   "call void @llvm.experimental.noalias.scope.decl(metadata !60)\n"
   "store i32 %tid, ptr addrspace(1) %b, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n!2 = !{ptr @gathers, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 1, i32 3, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_name\", !\"array_ref<void>\","
   " !\"air.arg_name\", !\"buffers\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n"
   "!60 = !{!61}\n!61 = distinct !{!61, !62}\n!62 = distinct !{!62}\n";

/* A buffer array binds its buffers at consecutive indices, each of which
 * reflection lists, and its loads read their bound addresses. */
static void
buffer_arrays_bind_consecutive_buffers(void)
{
   struct stela_metal_reflection reflection;
   struct stela_air_error error;
   assert(reflect(gathers, "gathers", &reflection, &error));
   assert(reflection.buffer_count == 3);
   for (unsigned i = 0; i < 3; i++)
      assert(reflection.buffers[i].index == 1 + i);

   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(gathers, "gathers", &specialization, &error);
   if (!shader)
      fprintf(stderr, "gathers: %u %s\n", error.kind, error.detail);
   assert(shader);
   static uint8_t memory[0x900];
   static uint8_t scratch[64];
   uint8_t push[STELA_METAL_PUSH_BYTES] = {0};
   put64(push + STELA_METAL_PUSH_TABLES, BASE + TABLE);
   for (unsigned i = 0; i < 3; i++)
      put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES + 8 * (1 + i), BASE + OUT + 16 * i);
   const struct air_effect_environment environment = {
      .base = BASE, .bytes = memory, .size = sizeof(memory), .push = push,
      .push_size = sizeof(push), .local_index = 5, .scratch = scratch,
      .scratch_size = sizeof(scratch),
   };
   struct air_effect_store store;
   struct air_effect_run run = air_effect_execute_in(shader, 100000, &store, 1, &environment);
   if (run.status != AIR_EFFECT_COMPLETE)
      fprintf(stderr, "gathers: %s\n", run.detail);
   assert(run.status == AIR_EFFECT_COMPLETE && run.stores == 1);
   const uint32_t stored = 5;
   expect_store(&store, false, BASE + OUT + 32, &stored, 1);
   stela_metal_release(shader);
}

/* Copies src[tid % n] where n > tid, as a guard on a function constant
 * divisor reads; past it, threads below 2 go through a branch on false
 * and one on true. With n zero, folding leaves the copy behind `br i1
 * false` with a poison address, and the ISA call stays behind `br i1 true`
 * unless ISA_BRANCH makes its path live. */
#define DIVIDED(isa_branch)                                                                    \
   "@n = external addrspace(2) constant i32\n"                                                 \
   "declare i32 @llvm.agx3.example(i32)\n"                                                     \
   "define void @divided(ptr addrspace(1) %src, ptr addrspace(1) %dst, i32 %tid) {\n"           \
   "entry:\n%n = load i32, ptr addrspace(2) @n, align 4\n"                                     \
   "%some = icmp ugt i32 %n, %tid\nbr i1 %some, label %copy, label %join\n"                    \
   "copy:\n%r = urem i32 %tid, %n\n%i = zext i32 %r to i64\n"                                  \
   "%s = getelementptr inbounds i32, ptr addrspace(1) %src, i64 %i\n"                          \
   "%v = load i32, ptr addrspace(1) %s, align 4\nbr label %join\n"                             \
   "join:\n%w = phi i32 [ %v, %copy ], [ 7, %entry ]\n"                                        \
   "%small = icmp ult i32 %tid, 2\nbr i1 %small, label %a, label %x\n"                         \
   "a:\nbr i1 false, label %x, label %y\n"                                                     \
   "y:\nbr i1 " isa_branch ", label %x, label %isa\n"                                          \
   "isa:\n%e = call i32 @llvm.agx3.example(i32 %w)\nbr label %x\n"                             \
   "x:\n%z = phi i32 [ %w, %join ], [ 100, %a ], [ 50, %y ], [ %e, %isa ]\n"                   \
   "%j = zext i32 %tid to i64\n"                                                               \
   "%d = getelementptr inbounds i32, ptr addrspace(1) %dst, i64 %j\n"                          \
   "store i32 %z, ptr addrspace(1) %d, align 4\nret void\n}\n"                                 \
   "!air.kernel = !{!2}\n"                                                                     \
   "!2 = !{ptr @divided, !3, !4}\n!3 = !{}\n!4 = !{!40, !41, !42}\n"                           \
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read\","      \
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\"," \
   " i32 4, !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"src\"}\n"                 \
   "!41 = !{i32 1, !\"air.buffer\", !\"air.location_index\", i32 1, i32 1, !\"air.read_write\","\
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\"," \
   " i32 4, !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"dst\"}\n"                 \
   "!42 = !{i32 2, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","       \
   " !\"air.arg_name\", !\"tid\"}\n"                                                           \
   "!air.function_constants = !{!50}\n"                                                        \
   "!50 = !{ptr addrspace(2) @n, !\"uint\", !\"n\", i32 0, i1 true}\n"

/* Runs thread tid of workgroup 0 with src at SOURCE (16 bytes of `fill`
 * from 10 up, or `fill` itself when bytes) and the second buffer at
 * DESTINATION: its stores into trace, and their count. */
static unsigned
run_two_buffers(nir_shader *shader, uint32_t tid, bool bytes, uint8_t fill,
                struct air_effect_store *trace, unsigned capacity)
{
   enum { SOURCE = 0x100, DESTINATION = 0x200 };
   static uint8_t memory[0x900];
   uint8_t push[STELA_METAL_PUSH_BYTES] = {0};
   memset(memory, 0, sizeof(memory));
   put64(push + STELA_METAL_PUSH_TABLES, BASE + TABLE);
   put64(push + STELA_METAL_PUSH_ARGUMENTS, BASE + DISPATCH);
   put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES, BASE + SOURCE);
   put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES + 8, BASE + DESTINATION);
   for (unsigned i = 0; i < 3; i++)
      put32(memory + DISPATCH + STELA_METAL_DISPATCH_SHAPE + 4 * i, 1);
   if (bytes)
      memset(memory + SOURCE, fill, 0x100);
   else
      for (unsigned i = 0; i < 4; i++)
         put32(memory + SOURCE + 4 * i, fill + i);
   const struct air_effect_environment environment = {
      .base = BASE, .bytes = memory, .size = sizeof(memory), .push = push,
      .push_size = sizeof(push), .local_index = tid,
   };
   const nir_lower_compute_system_values_options linear = {.lower_cs_local_id_to_index = true};
   nir_lower_compute_system_values(shader, &linear);
   nir_lower_vars_to_ssa(shader);
   nir_lower_undef_to_zero(shader, NULL);
   nir_opt_dce(shader);
   struct air_effect_run run =
      air_effect_execute_in(shader, 1u << 20, trace, capacity, &environment);
   if (run.status != AIR_EFFECT_COMPLETE)
      fprintf(stderr, "two buffers: %s\n", run.detail);
   assert(run.status == AIR_EFFECT_COMPLETE);
   for (unsigned i = 0; i < run.stores; i++)
      trace[i].address -= BASE + DESTINATION;
   return run.stores;
}

/* What divided stores for thread tid with constant n, or ~0u if refused. */
static uint32_t
divided_result(const char *fixture, uint32_t n, uint32_t tid, struct stela_air_error *error)
{
   const struct stela_air_constant_value value = {0, &n, 4};
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .constants = &value, .constant_count = 1,
   };
   nir_shader *shader = compile(fixture, "divided", &specialization, error);
   if (!shader)
      return ~0u;
   struct air_effect_store trace[1];
   unsigned stores = run_two_buffers(shader, tid, false, 10, trace, 1);
   stela_metal_release(shader);
   assert(stores == 1 && trace[0].address == 4 * tid && trace[0].bits == 32);
   return trace[0].values[0].u32;
}

/* A branch on a constant takes one edge: what only the other reaches, such
 * as a folded zero divisor's poison address or an ISA intrinsic, is not
 * translated, and a PHI takes no value along it. */
static void
constant_branches_take_one_edge(void)
{
   struct stela_air_error error = {0};
   const char *dead = DIVIDED("true"), *live = DIVIDED("%some");
   uint32_t result = divided_result(dead, 0, 0, &error);
   if (result == ~0u)
      fprintf(stderr, "divided: %u %s\n", error.kind, error.detail);
   assert(result == 50);
   assert(divided_result(dead, 0, 3, &error) == 7);
   assert(divided_result(dead, 4, 0, &error) == 50);
   assert(divided_result(dead, 4, 3, &error) == 13);
   assert(divided_result(dead, 4, 5, &error) == 7);
   /* Reachable, the ISA intrinsic is named. */
   assert(divided_result(live, 4, 0, &error) == ~0u);
   assert(error.kind == STELA_AIR_INSTRUCTION &&
          !strcmp(error.detail, "unsupported Apple GPU ISA intrinsic llvm.agx3.example"));
}

/* Compiles a module of STELA_TEST_METALFX_AIR with one constant list. */
static nir_shader *
metalfx_kernel(const char *directory, const char *name,
               const struct stela_air_constant_value *values, unsigned count,
               struct stela_air_error *error)
{
   char path[4096];
   size_t size;
   snprintf(path, sizeof(path), "%s/%s.air", directory, name);
   void *air = read_file(path, &size);
   assert(air);
   struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .constants = values, .constant_count = count,
   };
   /* Each constexpr sampler state a sampler of its own, as the device
    * gives them. */
   static uint64_t states[STELA_METAL_SAMPLER_STATES];
   static struct stela_air_constexpr_sampler samplers[STELA_METAL_SAMPLER_STATES];
   *error = (struct stela_air_error){0};
   assert(stela_metal_sampler_states(air, size, states, &specialization.sampler_count, error));
   for (unsigned i = 0; i < specialization.sampler_count; i++)
      samplers[i] = (struct stela_air_constexpr_sampler){states[i], (uint64_t)i << 32 | i};
   specialization.samplers = samplers;
   nir_shader *shader = stela_metal_compile(air, size, name, &target, &specialization, error);
   if (!shader)
      fprintf(stderr, "%s: %u %s\n", name, error->kind, error->detail);
   free(air);
   return shader;
}

/* Oracle: MetalFX's kernels that use the constructs above.
 * STELA_TEST_METALFX_AIR names a directory of its default.metallib's
 * modules, one NAME.air each, never committed. */
static void
metalfx_kernels_compile(void)
{
   const char *directory = getenv("STELA_TEST_METALFX_AIR");
   if (!directory)
      return;
   struct stela_air_error error;
   /* Zero channels leave no thread a copy: the guard folds to false over
    * a zero divisor. One channel copies a byte. */
   uint16_t zero = 0, one = 1;
   const struct stela_air_constant_value none[] = {{0, &zero, 2}, {1, &zero, 2}};
   const struct stela_air_constant_value single[] = {{0, &one, 2}, {1, &one, 2}};
   struct air_effect_store trace[2];
   nir_shader *shader = metalfx_kernel(directory, "DepthToSpaceWeights", none, 2, &error);
   assert(shader && run_two_buffers(shader, 0, true, 0x5a, trace, 2) == 0);
   stela_metal_release(shader);
   shader = metalfx_kernel(directory, "DepthToSpaceWeights", single, 2, &error);
   assert(shader && run_two_buffers(shader, 0, true, 0x5a, trace, 2) == 1);
   assert(trace[0].bits == 8 && trace[0].values[0].u8 == 0x5a);
   stela_metal_release(shader);
   /* Motion vector splats: unsigned maxima at ushort positions. */
   const char *splats[] = {"splatAppMotionVectors", "splatOpticalMotionVectors"};
   const unsigned calls[] = {10, 4};
   for (unsigned i = 0; i < ARRAY_SIZE(splats); i++) {
      shader = metalfx_kernel(directory, splats[i], NULL, 0, &error);
      assert(shader);
      unsigned atomics = 0;
      nir_foreach_function_impl(impl, shader) {
         nir_foreach_block(block, impl) {
            nir_foreach_instr(instr, block) {
               if (instr->type != nir_instr_type_intrinsic)
                  continue;
               nir_intrinsic_instr *intrinsic = nir_instr_as_intrinsic(instr);
               if (!nir_intrinsic_has_atomic_op(intrinsic) || !nir_intrinsic_has_format(intrinsic))
                  continue;
               assert(nir_intrinsic_atomic_op(intrinsic) == nir_atomic_op_umax &&
                      nir_intrinsic_format(intrinsic) == PIPE_FORMAT_R32_UINT);
               atomics++;
            }
         }
      }
      assert(atomics == calls[i]);
      stela_metal_release(shader);
   }
   /* The convolutions of Apple's matrix unit name its intrinsics, with
    * every constant's bytes 1 so their loops run. */
   const char *isa[] = {"Conv1x1FusedBilinear", "Conv3x3Stride2", "Conv3x3Stride2U8",
                        "ConvKxKStride2_K3x3_F16", "Conv3x3SigmoidD2S", "ConvKxKStride2_K6x6"};
   static const struct { const char *type; unsigned bytes; } sizes[] = {
      {"bool", 1}, {"ushort", 2}, {"ushort4", 8}, {"uint", 4}, {"float", 4}, {"float4", 16},
   };
   uint8_t filled[16];
   memset(filled, 1, sizeof(filled));
   for (unsigned i = 0; i < ARRAY_SIZE(isa); i++) {
      char path[4096];
      size_t size;
      snprintf(path, sizeof(path), "%s/%s.air", directory, isa[i]);
      void *air = read_file(path, &size);
      assert(air);
      static struct stela_metal_reflection reflection;
      assert(stela_metal_reflect(air, size, isa[i], NULL, 0, &reflection, &error));
      struct stela_air_constant_value values[STELA_METAL_FUNCTION_CONSTANTS];
      unsigned count = 0;
      for (unsigned c = 0; c < reflection.function_constant_count; c++) {
         const struct stela_metal_function_constant *constant = &reflection.function_constants[c];
         unsigned bytes = 0;
         for (unsigned k = 0; k < ARRAY_SIZE(sizes); k++) {
            if (strlen(sizes[k].type) == constant->type_bytes &&
                !memcmp(sizes[k].type, reflection.names + constant->type, constant->type_bytes))
               bytes = sizes[k].bytes;
         }
         assert(bytes);
         values[count++] = (struct stela_air_constant_value){constant->index, filled, bytes};
      }
      free(air);
      assert(!metalfx_kernel(directory, isa[i], values, count, &error));
      assert(error.kind == STELA_AIR_INSTRUCTION &&
             strstr(error.detail, "unsupported Apple GPU ISA intrinsic llvm.agx3."));
   }
}

/* What recursive stores over 3 with function linked at handle 1 and calls
 * through tables nested at most depth deep, or ~0u if it fails to compile. */
static uint32_t
recursion_result(const char *function, uint32_t depth, struct stela_air_error *error)
{
   const struct stela_air_linked_function linked = {function, strlen(function)};
   const struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .linked_functions = &linked, .linked_function_count = 1,
      .max_call_stack_depth = depth,
   };
   nir_shader *shader = compile(recursive, "recursive", &specialization, error);
   if (!shader)
      return ~0u;
   enum { FUNCTIONS = 0x100, DATA = 0x200 };
   static uint8_t memory[0x300];
   uint8_t push[STELA_METAL_PUSH_BYTES] = {0};
   memset(memory, 0, sizeof(memory));
   put64(push + STELA_METAL_PUSH_TABLES, BASE + TABLE);
   put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES, BASE + FUNCTIONS);
   put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES + 8, BASE + DATA);
   put64(memory + FUNCTIONS, 2);
   put64(memory + FUNCTIONS + 8, 1);
   put32(memory + DATA, 3);
   const struct air_effect_environment environment = {
      .base = BASE, .bytes = memory, .size = sizeof(memory), .push = push,
      .push_size = sizeof(push),
   };
   /* The oracle reads the invocation's linear index and SSA values, and
    * refuses undefined ones, which structurizing leaves in its path
    * variables and for the paths a phi does not take. */
   const nir_lower_compute_system_values_options linear = {.lower_cs_local_id_to_index = true};
   nir_lower_compute_system_values(shader, &linear);
   nir_lower_vars_to_ssa(shader);
   nir_lower_undef_to_zero(shader, NULL);
   nir_opt_dce(shader);
   struct air_effect_store trace[1];
   struct air_effect_run run = air_effect_execute_in(shader, 100000, trace, 1, &environment);
   stela_metal_release(shader);
   if (run.status != AIR_EFFECT_COMPLETE)
      fprintf(stderr, "recursive: %s\n", run.detail);
   assert(run.status == AIR_EFFECT_COMPLETE && run.stores == 1);
   assert(trace[0].address == BASE + DATA && trace[0].bits == 32 && trace[0].components == 1);
   return trace[0].values[0].u32;
}

/* Recursion through function tables runs to the pipeline's call stack
 * depth, and a call nested deeper calls none. */
static void
recursion_unrolls_to_the_call_stack_depth(void)
{
   struct stela_air_error error;
   assert(recursion_result("down", 4, &error) == 6);
   assert(recursion_result("down", 9, &error) == 6);
   assert(recursion_result("down", 2, &error) == 4);
   assert(recursion_result("down", 1, &error) == 2);
   assert(recursion_result("down", 0, &error) == 0);
   /* Two calls per level: 1 + 2 * (1 + 2 * 1). */
   assert(recursion_result("split", 3, &error) == 7);
   /* Unrolling past the compilation budget, or the depth Stela unrolls. */
   assert(recursion_result("split", 40, &error) == ~0u && error.kind == STELA_AIR_LIMIT &&
          strstr(error.detail, "instruction budget"));
   assert(recursion_result("down", 1000, &error) == ~0u && error.kind == STELA_AIR_LIMIT &&
          strstr(error.detail, "call stack depth"));
}

/* A kernel of indirect commands takes each dispatch's threadgroup shape
 * and threadgroup buffer placement, and its threads past the shape skip
 * it. */
static void
dynamic_kernels_take_their_dispatch_shape(void)
{
   struct stela_air_error error;
   struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .dynamic_threadgroups = 1,
   };
   nir_shader *shader = compile(shaped, "shaped", &specialization, &error);
   if (!shader)
      fprintf(stderr, "shaped: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.workgroup_size[0] == 64);
   /* The threadgroup buffers share all of the target's threadgroup memory. */
   assert(shader->info.shared_size == target.max_threadgroup_bytes);
   /* The driver's own layout of what remains places nothing again. */
   nir_lower_vars_to_explicit_types(shader, nir_var_mem_shared, glsl_get_natural_size_align_bytes);
   assert(shader->info.shared_size == target.max_threadgroup_bytes);

   struct air_effect_store trace[8];
   assert(run_shaped(shader, 5, trace, ARRAY_SIZE(trace)) == 4);
   expect_store(&trace[0], false, BASE + OUT, (const uint32_t[]){1, 1, 0}, 3);
   expect_store(&trace[1], false, BASE + OUT + 16, (const uint32_t[]){4, 2, 1}, 3);
   expect_store(&trace[2], false, BASE + OUT + 32, (const uint32_t[]){5, 1, 0}, 3);
   expect_store(&trace[3], true, 48, (const uint32_t[]){1}, 1);
   assert(run_shaped(shader, 8, trace, ARRAY_SIZE(trace)) == 0);
   stela_metal_release(shader);

   /* Dynamic threadgroups are a kernel's, which takes no lengths. */
   const struct stela_metal_threadgroup_length lengths[] = {{1, 16}};
   specialization.threadgroup_length_count = 1;
   specialization.threadgroup_lengths = lengths;
   assert(!compile(shaped, "shaped", &specialization, &error) && error.kind == STELA_AIR_INPUT);
   const struct stela_metal_specialization dynamic = {.dynamic_threadgroups = 1};
   assert(!compile(fragment, "fragment", &dynamic, &error) &&
          error.kind == STELA_AIR_INPUT);
}

/* `shaped` with its threadgroup buffer's metadata typed by `typing`. */
static char *
shaped_typed(const char *typing)
{
   static const char typed[] =
      "!\"air.address_space\", i32 3, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\", i32 4,"
      " !\"air.arg_type_name\", !\"uint\",";
   const char *at = strstr(shaped, typed);
   assert(at && !strstr(at + 1, typed));
   return ralloc_asprintf(NULL, "%.*s!\"air.address_space\", i32 3%s%s", (int)(at - shaped), shaped,
                          typing, at + strlen(typed));
}

/* A `threadgroup void *` argument, which declares neither size nor
 * alignment, as MPS's reductions take theirs: it reflects no bytes of its
 * own, takes its dispatch's length and placement, and its stores reach the
 * memory a typed buffer's do. Half a declaration is refused. */
static void
untyped_threadgroup_buffers_take_their_dispatch_lengths(void)
{
   struct stela_air_error error;
   char *untyped = shaped_typed(", !\"air.arg_type_name\", !\"void\",");
   struct stela_metal_reflection reflection;
   assert(reflect(untyped, "shaped", &reflection, &error));
   assert(reflection.threadgroup_count == 1 && reflection.threadgroup[0].index == 1 &&
          reflection.threadgroup[0].bytes == 0);

   struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .dynamic_threadgroups = 1,
   };
   nir_shader *shader = compile(untyped, "shaped", &specialization, &error);
   if (!shader)
      fprintf(stderr, "untyped dynamic: %u %s\n", error.kind, error.detail);
   assert(shader);
   struct air_effect_store trace[8];
   assert(run_shaped(shader, 5, trace, ARRAY_SIZE(trace)) == 4);
   expect_store(&trace[0], false, BASE + OUT, (const uint32_t[]){1, 1, 0}, 3);
   expect_store(&trace[1], false, BASE + OUT + 16, (const uint32_t[]){4, 2, 1}, 3);
   expect_store(&trace[2], false, BASE + OUT + 32, (const uint32_t[]){5, 1, 0}, 3);
   expect_store(&trace[3], true, 48, (const uint32_t[]){1}, 1);
   stela_metal_release(shader);

   const struct stela_metal_threadgroup_length lengths[] = {{1, 256}};
   specialization = (struct stela_metal_specialization){
      .workgroup_size = {64, 1, 1}, .threadgroup_length_count = 1, .threadgroup_lengths = lengths,
   };
   shader = compile(untyped, "shaped", &specialization, &error);
   if (!shader)
      fprintf(stderr, "untyped: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.shared_size >= 256);
   stela_metal_release(shader);
   /* Its only bytes are its dispatch's length. */
   specialization.threadgroup_length_count = 0;
   assert(!compile(untyped, "shaped", &specialization, &error) && error.kind == STELA_AIR_ARGUMENT);
   ralloc_free(untyped);

   const char *halves[] = {
      ", !\"air.arg_type_size\", i32 4, !\"air.arg_type_name\", !\"uint\",",
      ", !\"air.arg_type_align_size\", i32 4, !\"air.arg_type_name\", !\"uint\",",
      ", !\"air.arg_type_size\", i32 0, !\"air.arg_type_align_size\", i32 4,",
   };
   for (unsigned i = 0; i < ARRAY_SIZE(halves); i++) {
      char *half = shaped_typed(halves[i]);
      assert(!reflect(half, "shaped", &reflection, &error) && error.kind == STELA_AIR_ARGUMENT);
      ralloc_free(half);
   }
}

/* A C++ template parameter object, an internal constant of the thread
 * address space, which the kernel copies and reads fields of, as MPS's
 * tensor kernels copy their matmul descriptors. */
static const char *descriptor_copy =
   "%struct.d = type { i32, i32, i8, i32 }\n"
   "@descriptor = internal unnamed_addr constant %struct.d { i32 16, i32 32, i8 1, i32 5 },"
   " align 16\n"
   "declare void @llvm.memcpy.p0.p0.i64(ptr, ptr, i64, i1)\n"
   "define void @copied(ptr addrspace(1) %out) {\n"
   "%copy = alloca %struct.d, align 4\n"
   "call void @llvm.memcpy.p0.p0.i64(ptr align 4 %copy, ptr align 16 @descriptor, i64 16, i1 false)\n"
   "%b = getelementptr i8, ptr %copy, i64 4\n%v = load i32, ptr %b, align 4\n"
   "%c = getelementptr i8, ptr @descriptor, i64 12\n%w = load i32, ptr %c, align 4\n"
   "%s = add i32 %v, %w\nstore i32 %s, ptr addrspace(1) %out, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n!2 = !{ptr @copied, !3, !4}\n!3 = !{}\n!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"out\"}\n";

/* A threadgroup array of bfloat, as MPS's quantized matrix kernels stage
 * their tiles: 2.5 and 1.5 stored at its first and last elements. */
static const char *bfloat_tile =
   "@tile = internal addrspace(3) global [4 x bfloat] undef, align 2\n"
   "define void @tiled(ptr addrspace(1) %out) {\n"
   "store bfloat 0xR4020, ptr addrspace(3) @tile, align 2\n"
   "%p = getelementptr [4 x bfloat], ptr addrspace(3) @tile, i64 0, i64 3\n"
   "store bfloat 0xR3FC0, ptr addrspace(3) %p, align 2\n"
   "store i32 1, ptr addrspace(1) %out, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n!2 = !{ptr @tiled, !3, !4}\n!3 = !{}\n!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"out\"}\n";

/* Constant data of the thread address space holds its initializer's
 * values, copied or read in place. */
static void
thread_constants_are_constant_data(void)
{
   struct stela_air_error error;
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(descriptor_copy, "copied", &specialization, &error);
   if (!shader)
      fprintf(stderr, "copied: %u %s\n", error.kind, error.detail);
   assert(shader);
   assert(run_summed(shader) == 32 + 5);
   stela_metal_release(shader);
}

/* A threadgroup bfloat array holds the bfloat bits stored to it. */
static void
threadgroup_globals_hold_bfloat(void)
{
   struct stela_air_error error;
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(bfloat_tile, "tiled", &specialization, &error);
   if (!shader)
      fprintf(stderr, "tiled: %u %s\n", error.kind, error.detail);
   assert(shader && shader->info.shared_size >= 8);
   static uint8_t memory[0x900];
   uint8_t push[STELA_METAL_PUSH_BYTES] = {0};
   memset(memory, 0, sizeof(memory));
   put64(push + STELA_METAL_PUSH_TABLES, BASE + TABLE);
   put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES, BASE + OUT);
   const struct air_effect_environment environment = {
      .base = BASE, .bytes = memory, .size = sizeof(memory), .push = push,
      .push_size = sizeof(push),
   };
   struct air_effect_store trace[4];
   struct air_effect_run run = air_effect_execute_in(shader, 100000, trace, ARRAY_SIZE(trace),
                                                     &environment);
   if (run.status != AIR_EFFECT_COMPLETE)
      fprintf(stderr, "tiled: %s\n", run.detail);
   assert(run.status == AIR_EFFECT_COMPLETE && run.stores == 3);
   const uint16_t bits[] = {0x4020, 0x3fc0};
   for (unsigned i = 0; i < 2; i++) {
      assert(trace[i].shared && trace[i].bits == 16 && trace[i].components == 1);
      assert(trace[i].values[0].u16 == bits[i]);
   }
   assert(trace[1].address == trace[0].address + 6);
   expect_store(&trace[2], false, BASE + OUT, (const uint32_t[]){1}, 1);
   stela_metal_release(shader);
}

/* Counts a loop to 4, and by tens to 40, beside a pointer PHI stepping off
 * null, declared between the loop header's other PHIs, as MPS's matrix multiplies leave a
 * buffer pointer their constants rule out: nothing may come between them. */
static const char *null_loop =
   "define void @null_loop(ptr addrspace(1) %out) {\n"
   "entry:\n%r = getelementptr i8, ptr addrspace(1) null, i64 16\nbr label %loop\n"
   "loop:\n%i = phi i32 [ 0, %entry ], [ %n, %loop ]\n"
   "%p = phi ptr addrspace(1) [ %r, %entry ], [ %q, %loop ]\n"
   "%j = phi i32 [ 10, %entry ], [ %m, %loop ]\n"
   "%q = getelementptr i8, ptr addrspace(1) %p, i64 4\n"
   "%n = add i32 %i, 1\n%m = add i32 %j, 10\n%c = icmp ult i32 %n, 4\n"
   "br i1 %c, label %loop, label %done\n"
   "done:\n%none = icmp eq ptr addrspace(1) %p, null\n%z = zext i1 %none to i32\n"
   "%t = add i32 %n, %z\n%s = add i32 %t, %j\n"
   "store i32 %s, ptr addrspace(1) %out, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n!2 = !{ptr @null_loop, !3, !4}\n!3 = !{}\n!4 = !{!40}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read_write\","
   " !\"air.address_space\", i32 1, !\"air.arg_type_size\", i32 4, !\"air.arg_type_align_size\","
   " i32 4, !\"air.arg_type_name\", !\"uint\", !\"air.arg_name\", !\"out\"}\n";

/* A pointer PHI rooted at null among a block's PHIs keeps them together,
 * and the loop and the comparison compute what they say. */
static void
null_pointer_phis_stay_among_phis(void)
{
   struct stela_air_error error;
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(null_loop, "null_loop", &specialization, &error);
   if (!shader)
      fprintf(stderr, "null_loop: %u %s\n", error.kind, error.detail);
   assert(shader);
   /* The pointer stepped off null is not null. */
   assert(run_summed(shader) == 4 + 40);
   stela_metal_release(shader);
}

/* Writes its thread's index plus 100 through the device address its
 * constant parameters hold as an integer, as MPS's kernels keep their
 * buffers' gpuAddress: the even threads' at it, the odd threads' 64 bytes
 * past it, joined by a PHI. */
static const char *addressed =
   "define void @addressed(ptr addrspace(2) %params, i32 %tid) {\n"
   "entry:\n%odd = and i32 %tid, 1\n%is_odd = icmp ne i32 %odd, 0\n"
   "%a = load i64, ptr addrspace(2) %params, align 8\n"
   "%pa = inttoptr i64 %a to ptr addrspace(1)\n"
   "br i1 %is_odd, label %second, label %join\n"
   "second:\n%pb = getelementptr i8, ptr addrspace(1) %pa, i64 64\nbr label %join\n"
   "join:\n%p = phi ptr addrspace(1) [ %pa, %entry ], [ %pb, %second ]\n"
   "%v = add i32 %tid, 100\nstore i32 %v, ptr addrspace(1) %p, align 4\nret void\n}\n"
   "!air.kernel = !{!2}\n!2 = !{ptr @addressed, !3, !4}\n!3 = !{}\n!4 = !{!40, !41}\n"
   "!40 = !{i32 0, !\"air.buffer\", !\"air.location_index\", i32 0, i32 1, !\"air.read\","
   " !\"air.address_space\", i32 2, !\"air.arg_type_size\", i32 16, !\"air.arg_type_align_size\","
   " i32 8, !\"air.arg_type_name\", !\"Params\", !\"air.arg_name\", !\"params\"}\n"
   "!41 = !{i32 1, !\"air.thread_position_in_grid\", !\"air.arg_type_name\", !\"uint\","
   " !\"air.arg_name\", !\"tid\"}\n";

/* A pointer made of a device address reaches the memory at that address,
 * and past it through a PHI. */
static void
pointers_made_of_addresses_reach_their_memory(void)
{
   struct stela_air_error error;
   const struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(addressed, "addressed", &specialization, &error);
   if (!shader)
      fprintf(stderr, "addressed: %u %s\n", error.kind, error.detail);
   assert(shader);
   for (uint32_t tid = 0; tid < 2; tid++) {
      static uint8_t memory[0x900];
      uint8_t push[STELA_METAL_PUSH_BYTES] = {0};
      memset(memory, 0, sizeof(memory));
      put64(push + STELA_METAL_PUSH_TABLES, BASE + TABLE);
      put64(memory + TABLE + STELA_METAL_TABLE_BUFFER_ADDRESSES, BASE + OUT);
      put64(memory + OUT, BASE + OUT + 0x40);
      const struct air_effect_environment environment = {
         .base = BASE, .bytes = memory, .size = sizeof(memory), .push = push,
         .push_size = sizeof(push), .local_index = tid,
      };
      struct air_effect_store store;
      struct air_effect_run run = air_effect_execute_in(shader, 100000, &store, 1, &environment);
      if (run.status != AIR_EFFECT_COMPLETE)
         fprintf(stderr, "addressed: %s\n", run.detail);
      assert(run.status == AIR_EFFECT_COMPLETE && run.stores == 1);
      expect_store(&store, false, BASE + OUT + (tid ? 0x80 : 0x40), (const uint32_t[]){tid + 100}, 1);
   }
   stela_metal_release(shader);
   /* Only a 64-bit address makes a device or constant pointer. */
   char *narrow = ralloc_asprintf(NULL, "%s", addressed);
   char *at = strstr(narrow, "%pa = inttoptr i64 %a");
   assert(at);
   memcpy(at, "%pa = inttoptr i32 %t", strlen("%pa = inttoptr i32 %t"));
   char *fixture = ralloc_asprintf(NULL, "%.*s%%t = trunc i64 %%a to i32\n%s", (int)(at - narrow),
                                   narrow, at);
   assert(!compile(fixture, "addressed", &specialization, &error) && error.kind == STELA_AIR_TYPE);
   ralloc_free(fixture);
   ralloc_free(narrow);
}

/* TEMPORARY host harness (not for commit). */
static void
harness(void)
{
   const char *path = getenv("STELA_HARNESS");
   if (!path)
      return;
   FILE *file = fopen(path, "rb");
   assert(file);
   fseek(file, 0, SEEK_END);
   long size = ftell(file);
   fseek(file, 0, SEEK_SET);
   char *bytes = malloc(size);
   assert(fread(bytes, 1, size, file) == (size_t)size);
   fclose(file);
   static struct stela_air_constant_value values[512];
   static uint8_t data[512][16];
   unsigned count = 0;
   const char *text = getenv("STELA_HARNESS_CONSTANTS");
   while (text && *text) {
      unsigned index = strtoul(text, (char **)&text, 10);
      assert(*text == '=');
      text++;
      unsigned bytes_count = 0;
      while (isxdigit((unsigned char)text[0]) && isxdigit((unsigned char)text[1])) {
         char pair[3] = {text[0], text[1], 0};
         data[count][bytes_count++] = strtoul(pair, NULL, 16);
         text += 2;
      }
      values[count] = (struct stela_air_constant_value){index, data[count], bytes_count};
      count++;
      if (*text == ',')
         text++;
   }
   static struct stela_metal_linked_varying varyings[32];
   unsigned varying_count = 0;
   char *list = getenv("STELA_HARNESS_VARYINGS") ? strdup(getenv("STELA_HARNESS_VARYINGS")) : NULL;
   for (char *name = list ? strtok(list, ",") : NULL; name; name = strtok(NULL, ","))
      varyings[varying_count++] = (struct stela_metal_linked_varying){name, strlen(name), 0};
   static struct stela_air_constexpr_sampler samplers[16];
   unsigned sampler_count = 0;
   for (const char *s = getenv("STELA_HARNESS_SAMPLERS"); s && *s; s += *s == ',') {
      samplers[sampler_count].state = strtoull(s, (char **)&s, 0);
      samplers[sampler_count].id = 0x5000 + sampler_count;
      sampler_count++;
   }
   struct stela_metal_specialization specialization = {
      .workgroup_size = {64, 1, 1}, .constants = values, .constant_count = count,
      .varyings = varyings, .varying_count = varying_count,
      .sampler_count = sampler_count, .samplers = samplers,
   };
   struct stela_air_error error = {0};
   nir_shader *shader = stela_metal_compile(bytes, size, getenv("STELA_HARNESS_ENTRY"), &target,
                                            &specialization, &error);
   if (!shader)
      fprintf(stderr, "refused %u %s\n", error.kind, error.detail);
   else
      nir_print_shader(shader, stdout);
   exit(0);
}

/* Each buffer a function loads from or stores to, on any path. */
static void
buffer_reads_name_each_accessed_slot(void)
{
   struct stela_air_error error = {0};
   struct stela_metal_specialization specialization = {.workgroup_size = {64, 1, 1}};
   nir_shader *shader = compile(guarded, "guarded", &specialization, &error);
   if (!shader)
      fprintf(stderr, "guarded: %u %s\n", error.kind, error.detail);
   assert(shader && stela_metal_buffer_reads(shader) == ((1u << 1) | (1u << 4)));
   stela_metal_release(shader);
}

int
main(void)
{
   glsl_type_singleton_init_or_ref();
   harness();
   modules_list_their_functions();
   modules_list_their_exports();
   discovery_reports_malformed_metadata();
   reflection_requires_a_unique_listed_entry();
   kernels_reflect_and_compile_their_threadgroup_memory();
   dynamic_kernels_take_their_dispatch_shape();
   untyped_threadgroup_buffers_take_their_dispatch_lengths();
   thread_constants_are_constant_data();
   threadgroup_globals_hold_bfloat();
   pointers_made_of_addresses_reach_their_memory();
   null_pointer_phis_stay_among_phis();
   render_stages_compile_at_their_linked_locations();
   patch_functions_compile_behind_their_tessellation_stages();
   kernels_compile_with_their_stage_in();
   vertex_inputs_decode_their_words();
   mesh_stages_compile_with_their_launch();
   function_constants_reflect_and_specialize();
   derived_constants_are_constructed_from_their_function_constants();
   constructed_globals_are_not_capped();
   constructor_priorities_are_integer_literals();
   absent_textures_take_no_location();
   function_constants_select_outputs();
   older_kernels_compile();
   unoptimized_kernels_compile();
   small_thread_arrays_live_in_registers();
   undefined_lanes_read_as_anything();
   function_constants_select_arguments();
   specialized_fragment_calls_preserve_graph_admission();
   arrays_reflect_each_slot();
   bindings_reflect_their_structures();
   bindings_list_conditional_arguments_until_specialized();
   imageblocks_are_bindings();
   defined_functions_inline_into_their_callers();
   unresolved_calls_are_rejected_without_a_call_graph();
   stitched_functions_call_their_nodes_in_order();
   thread_structs_carry_pointers_to_linked_functions();
   mps_identity_links_its_stitched_functions();
   mps_kernels_compile();
   unreachable_paths_end_the_invocation();
   constant_branches_take_one_edge();
   metalfx_kernels_compile();
   stitched_struct_arguments_pass_their_fields();
   stitched_kernels_read_their_builtins();
   stitched_buffer_addresses_read_their_kernels_buffers();
   stitched_early_returns_skip_what_follows();
   stitched_textures_and_samplers_read_their_kernels_bindings();
   stitched_wrapped_textures_and_samplers_read_their_kernels_bindings();
   stitched_libraries_pass_their_own_struct_copies();
   stitched_threadgroup_memory_is_its_kernels();
   stitched_specializations_pass_structs_by_their_abi();
   buffer_arrays_bind_consecutive_buffers();
   loops_return_without_return_jumps();
   os_log_appends_self_contained_records();
   linked_functions_resolve_tables_and_externals();
   recursion_unrolls_to_the_call_stack_depth();
   linked_modules_specialize_their_function_constants();
   command_encoding_reflects_barriers();
   buffer_reads_name_each_accessed_slot();
   framebuffer_reads_access_storage_attachments();
   framebuffer_reads_run_their_own_fragment_tests();
   extended_range_outputs_clamp();
   glsl_type_singleton_decref();
   puts("metal compiler: ok");
   return 0;
}
