/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "metal_compiler.h"
#include "threadgroup.h"
#include "util/ralloc.h"
#include "vulkan/vulkan_core.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static_assert(sizeof(struct stela_metal_reflection) == STELA_METAL_REFLECTION_BYTES,
              "metal/compiler.rs mirrors the reflection");
static_assert(sizeof(struct stela_metal_target) == STELA_METAL_TARGET_BYTES,
              "metal/compiler.rs mirrors the target");
static_assert(sizeof(struct stela_metal_linked_varying) == STELA_METAL_LINKED_VARYING_BYTES,
              "metal/compiler.rs mirrors the linked varyings");
static_assert(sizeof(struct stela_metal_specialization) == STELA_METAL_SPECIALIZATION_BYTES,
              "metal/compiler.rs mirrors the specialization");
static_assert(sizeof(struct stela_air_constant_value) == STELA_METAL_CONSTANT_VALUE_BYTES,
              "metal/compiler.rs mirrors the constant values");
static_assert(sizeof(struct stela_air_module) == STELA_METAL_MODULE_BYTES,
              "metal/compiler.rs mirrors the linked modules");
static_assert(sizeof(struct stela_air_linked_function) == STELA_METAL_LINKED_FUNCTION_BYTES,
              "metal/compiler.rs mirrors the linked functions");
static_assert(sizeof(struct stela_metal_tessellation) == STELA_METAL_PIPELINE_TESSELLATION_BYTES,
              "metal/compiler.rs mirrors the pipeline tessellation");
static_assert(sizeof(struct stela_metal_attachment) == STELA_METAL_ATTACHMENT_BYTES,
              "metal_compiler.h storage attachment layout");
static_assert(sizeof(struct stela_metal_stage_in) == STELA_METAL_STAGE_IN_BYTES,
              "metal/compiler.rs mirrors the stage-in layout");
static_assert(STELA_METAL_FUNCTION_CONSTANTS == AIR_MAX_FUNCTION_CONSTANTS,
              "the reflection holds every function constant the frontend admits");
static_assert(sizeof(struct stela_air_constexpr_sampler) == STELA_METAL_CONSTEXPR_SAMPLER_BYTES,
              "metal/compiler.rs mirrors the constexpr samplers");
static_assert(STELA_METAL_SAMPLER_STATES == AIR_MAX_SAMPLER_STATES,
              "a module's states fit the listing");
static_assert(sizeof(struct stela_metal_bindings) == STELA_METAL_BINDINGS_BYTES,
              "metal/compiler.rs mirrors the bindings");
static_assert(STELA_METAL_BINDINGS == AIR_MAX_ARGUMENTS, "every argument may be a binding");

/* Reports the first error; always false. */
static bool
fail(struct stela_air_error *error, enum stela_air_error_kind kind, const char *detail)
{
   if (error->kind == STELA_AIR_OK) {
      error->kind = kind;
      snprintf(error->detail, sizeof(error->detail), "%s", detail);
   }
   return false;
}

/* AIR entry stages: kernels, then
 * enum stela_air_stage. */
static const enum stela_metal_function_stage function_stages[] = {
   STELA_METAL_KERNEL, STELA_METAL_VERTEX, STELA_METAL_FRAGMENT, STELA_METAL_OBJECT,
   STELA_METAL_MESH, STELA_METAL_VISIBLE,
};

static uint32_t
buffer_access(enum stela_air_buffer_access access)
{
   return (access & STELA_AIR_READ ? STELA_METAL_ACCESS_READ : 0) |
          (access & STELA_AIR_WRITE ? STELA_METAL_ACCESS_WRITE : 0);
}

static uint32_t
texture_access(enum stela_air_texture_access access)
{
   switch (access) {
   case STELA_AIR_TEXTURE_SAMPLE: return STELA_METAL_ACCESS_SAMPLE;
   case STELA_AIR_TEXTURE_READ: return STELA_METAL_ACCESS_READ;
   case STELA_AIR_TEXTURE_WRITE: return STELA_METAL_ACCESS_WRITE;
   default: return STELA_METAL_ACCESS_READ | STELA_METAL_ACCESS_WRITE;
   }
}

/* A reflection in progress. */
struct reflecting {
   const char *entry;
   struct stela_metal_reflection *out;
   struct stela_air_error *error;
   bool found;
};

/* Copies bytes into names, which hold *used of capacity bytes, at *offset. */
static bool
copy_name(struct stela_air_error *error, char *names, uint32_t *used, uint32_t capacity,
          const char *bytes, size_t size, uint32_t *offset)
{
   if (size > capacity - *used)
      return fail(error, STELA_AIR_LIMIT, "reflected names exceed their capacity");
   memcpy(names + *used, bytes, size);
   *offset = *used;
   *used += (uint32_t)size;
   return true;
}

static bool
add_name(struct reflecting *r, const char *bytes, size_t size, uint32_t *offset)
{
   return copy_name(r->error, r->out->names, &r->out->names_bytes, STELA_METAL_NAME_BYTES, bytes,
                    size, offset);
}

static bool
module_facts(struct reflecting *r, const struct stela_air_inspection *info)
{
   struct stela_metal_reflection *out = r->out;
   out->commands = !!(info->commands & STELA_AIR_COMMANDS_ENCODE);
   out->command_barriers = !!(info->commands & STELA_AIR_COMMANDS_BARRIERS);
   for (unsigned i = 0; i < info->function_constant_count; i++) {
      const struct stela_air_function_constant *constant = &info->function_constants[i];
      struct stela_metal_function_constant *row = &out->function_constants[i];
      *row = (struct stela_metal_function_constant){
         .index = constant->index, .name_bytes = (uint32_t)constant->name_bytes,
         .type_bytes = (uint32_t)constant->type_bytes, .required = constant->required};
      if (!add_name(r, constant->name, constant->name_bytes, &row->name) ||
          !add_name(r, constant->type, constant->type_bytes, &row->type))
         return false;
   }
   out->function_constant_count = info->function_constant_count;
   if (!info->ident)
      return true;
   r->out->ident_bytes = (uint32_t)info->ident_bytes;
   return add_name(r, info->ident, info->ident_bytes, &r->out->ident);
}

static bool
add_buffer(struct reflecting *r, const struct stela_air_buffer *buffer, bool argument_buffer)
{
   struct stela_metal_reflection *out = r->out;
   if (buffer->address_space == 3) {
      if (out->threadgroup_count == STELA_METAL_BUFFERS)
         return fail(r->error, STELA_AIR_LIMIT, "threadgroup buffers exceed the table");
      out->threadgroup[out->threadgroup_count++] = (struct stela_metal_threadgroup_use){
         buffer->index, buffer->type_size};
      return true;
   }
   /* A buffer array's slots: one per buffer. */
   unsigned slots = buffer->array ? buffer->length : 1;
   for (unsigned i = 0; i < slots; i++) {
      if (out->buffer_count == STELA_METAL_BUFFERS)
         return fail(r->error, STELA_AIR_LIMIT, "buffers exceed the table");
      out->buffers[out->buffer_count++] = (struct stela_metal_buffer_use){
         buffer->index + i, buffer_access(buffer->access), argument_buffer};
   }
   out->function_tables |= buffer->function_table;
   return true;
}

/* A texture argument's slots: an array's, one per element. */
static bool
add_texture(struct reflecting *r, const struct stela_air_texture *texture)
{
   struct stela_metal_reflection *out = r->out;
   for (unsigned i = 0; i < (texture->length ? texture->length : 1); i++) {
      if (out->texture_count == STELA_METAL_TEXTURES)
         return fail(r->error, STELA_AIR_LIMIT, "textures exceed the table");
      out->textures[out->texture_count++] = (struct stela_metal_texture_use){
         texture->index + i, texture_access(texture->access), texture->type, texture->primitive};
   }
   return true;
}

static bool
add_sampler(struct reflecting *r, const struct stela_air_sampler *sampler)
{
   struct stela_metal_reflection *out = r->out;
   for (unsigned i = 0; i < (sampler->length ? sampler->length : 1); i++) {
      if (out->sampler_count == STELA_METAL_SAMPLERS)
         return fail(r->error, STELA_AIR_LIMIT, "samplers exceed the table");
      out->samplers[out->sampler_count++] = sampler->index + i;
   }
   return true;
}

static bool
add_attribute(struct reflecting *r, unsigned location, enum stela_air_primitive primitive,
              unsigned components, bool patch_control_point, bool active, const char *name,
              size_t name_bytes)
{
   struct stela_metal_reflection *out = r->out;
   if (out->attribute_count == STELA_METAL_ATTRIBUTES)
      return fail(r->error, STELA_AIR_LIMIT, "attributes exceed the table");
   struct stela_metal_attribute_use *attribute = &out->attributes[out->attribute_count++];
   *attribute = (struct stela_metal_attribute_use){
      .index = location, .primitive = primitive, .components = components,
      .patch_control_point = patch_control_point, .active = active,
      .name_bytes = (uint32_t)name_bytes};
   return add_name(r, name, name_bytes, &attribute->name);
}

static bool
reflect_kernel(const struct stela_air_inspection *info, const struct stela_air_entry *entry,
               void *data)
{
   struct reflecting *r = data;
   if (entry->name_bytes != strlen(r->entry) || memcmp(entry->name, r->entry, entry->name_bytes))
      return true;
   r->found = true;
   r->out->max_total_threads = entry->max_threads;
   r->out->threadgroup_bytes = entry->threadgroup_bytes;
   r->out->imageblock_bytes = entry->imageblock_bytes;
   for (unsigned i = 0; i < entry->argument_count; i++) {
      const struct stela_air_argument_info *argument = &entry->arguments[i];
      bool added = argument->absent ? true :
                   argument->kind == 1 ? add_buffer(r, &argument->buffer, argument->argument_buffer) :
                   argument->kind == 3 ? add_texture(r, &argument->texture) :
                   argument->kind == 4 ? add_sampler(r, &argument->sampler) :
                   argument->kind == 5 ?
                                         add_attribute(r, argument->location, argument->primitive,
                                                       argument->type_components, false,
                                                       argument->used, argument->name,
                                                       argument->name_bytes) :
                                         true;
      if (!added)
         return false;
   }
   return module_facts(r, info);
}

static bool
add_varying(struct reflecting *r, const struct stela_air_stage_io *io)
{
   struct stela_metal_reflection *out = r->out;
   if (out->varying_count == STELA_METAL_VARYINGS)
      return fail(r->error, STELA_AIR_LIMIT, "varyings exceed the linkable slots");
   struct stela_metal_varying *varying = &out->varyings[out->varying_count++];
   *varying = (struct stela_metal_varying){
      .identifier_bytes = (uint32_t)io->identifier_bytes, .primitive = io->primitive,
      .components = io->components, .per_primitive = io->per_primitive};
   return add_name(r, io->identifier, io->identifier_bytes, &varying->identifier);
}

static bool
add_input(struct reflecting *r, const struct stela_air_stage_io *io)
{
   struct stela_metal_reflection *out = r->out;
   if (io->absent)
      return true;
   switch (io->semantic) {
   case STELA_AIR_STAGE_BUFFER: return add_buffer(r, &io->buffer, io->argument_buffer);
   case STELA_AIR_STAGE_TEXTURE: return add_texture(r, &io->texture);
   case STELA_AIR_STAGE_SAMPLER: return add_sampler(r, &io->sampler);
   case STELA_AIR_VARYING:
      out->interpolants |= io->interpolant;
      return add_varying(r, io);
   case STELA_AIR_BARYCENTRIC:
      out->barycentrics = 1;
      return true;
   case STELA_AIR_ATTACHMENT:
      out->attachment_reads |= 1u << io->location;
      return true;
   case STELA_AIR_VERTEX_INPUT:
      return add_attribute(r, io->location, io->primitive, io->components, false, io->used,
                           io->name, io->name_bytes);
   case STELA_AIR_CONTROL_POINTS:
      for (unsigned i = 0; i < io->member_count; i++) {
         const struct stela_air_struct_member *member = &io->members[i];
         if (!add_attribute(r, member->location, member->primitive, member->components, true,
                            !member->unused, member->name, member->name_bytes))
            return false;
      }
      return true;
   case STELA_AIR_STAGE_BUILTIN:
      out->amplification |= io->builtin == STELA_AIR_AMPLIFICATION_ID;
      return true;
   default:
      /* The position, the payload, the mesh and its grid. */
      return true;
   }
}

static bool
add_output(struct reflecting *r, const struct stela_air_stage_io *io)
{
   struct stela_metal_reflection *out = r->out;
   switch (io->semantic) {
   case STELA_AIR_VARYING: return add_varying(r, io);
   case STELA_AIR_RENDER_TARGET:
      if (out->color_output_count == STELA_METAL_COLOR_OUTPUTS)
         return fail(r->error, STELA_AIR_LIMIT, "render targets exceed the attachments");
      out->color_outputs[out->color_output_count++] = (struct stela_metal_color_output){
         io->location, io->primitive, io->index};
      return true;
   case STELA_AIR_DEPTH: out->writes_depth = 1; return true;
   case STELA_AIR_STENCIL: out->writes_stencil = 1; return true;
   case STELA_AIR_SAMPLE_MASK: out->writes_sample_mask = 1; return true;
   default: return true;
   }
}

static bool
reflect_stage(const struct stela_air_inspection *info, const struct stela_air_stage_entry *entry,
              void *data)
{
   struct reflecting *r = data;
   r->found = true;
   r->out->max_total_threads = entry->max_threads;
   r->out->threadgroup_bytes = entry->threadgroup_bytes;
   r->out->discards = entry->discards;
   r->out->imageblock_bytes = entry->imageblock_bytes;
   r->out->patch = entry->patch;
   r->out->patch_control_points = entry->patch_control_points;
   for (unsigned i = 0; i < entry->input_count; i++) {
      if (!add_input(r, &entry->inputs[i]))
         return false;
   }
   for (unsigned i = 0; i < entry->output_count; i++) {
      if (!add_output(r, &entry->outputs[i]))
         return false;
   }
   return module_facts(r, info);
}

/* Inspects entry, which the AIR list naming it gives *stage: a kernel's
 * visitor sees every kernel, a stage's only entry. *found reports whether a
 * visitor took entry. */
static bool
inspect_entry(const void *air, size_t size, const char *entry,
              const struct stela_air_constants *constants, stela_air_visit_entry kernel,
              stela_air_visit_stage stage_entry, void *data, const bool *found, unsigned *stage,
              struct stela_air_error *error)
{
   bool inspected = stela_air_inspect_entry(air, size, constants, entry, stage, kernel, stage_entry,
                                           data, error);
   /* A receiver's refusal is reported as the inspector's; keep the cause. */
   return inspected ? *found || fail(error, STELA_AIR_ARGUMENT, "entry is not an AIR kernel") :
                      false;
}

struct listing {
   stela_metal_visit_function visit;
   void *data;
};

static bool
list_function(void *data, const char *name, size_t name_bytes, unsigned stage)
{
   struct listing *listing = data;
   return listing->visit(listing->data, name, name_bytes, function_stages[stage]);
}

bool
stela_metal_functions(const void *air, size_t size, stela_metal_visit_function visit, void *data,
                      struct stela_air_error *error)
{
   struct listing listing = {visit, data};
   return stela_air_functions(air, size, list_function, &listing, error);
}

bool
stela_metal_exports(const void *air, size_t size, stela_metal_visit_function visit, void *data,
                    struct stela_air_error *error)
{
   struct listing listing = {visit, data};
   return stela_air_exports(air, size, list_function, &listing, error);
}

bool
stela_metal_stitch(const struct stela_air_stitch_source *sources, uint32_t source_count,
                   const struct stela_air_stitch_graph *graphs, uint32_t graph_count,
                   stela_air_visit_bytes visit, void *data, struct stela_air_error *error)
{
   return stela_air_stitch(sources, source_count, graphs, graph_count, visit, data, error);
}

/* Reflects entry, and selects its stage by the AIR list that names it. */
static bool
reflect(const void *air, size_t size, const char *entry,
        const struct stela_air_constants *constants, struct reflecting *r)
{
   *r->out = (struct stela_metal_reflection){0};
   unsigned stage = 0;
   bool reflected = inspect_entry(air, size, entry, constants, reflect_kernel, reflect_stage, r,
                                  &r->found, &stage, r->error);
   r->out->stage = function_stages[stage];
   return reflected;
}

bool
stela_metal_reflect(const void *air, size_t size, const char *entry,
                    const struct stela_air_constant_value *constants, uint32_t constant_count,
                    struct stela_metal_reflection *reflection, struct stela_air_error *error)
{
   struct stela_air_error local = {0};
   struct reflecting r = {.entry = entry, .out = reflection, .error = error ? error : &local};
   *r.error = (struct stela_air_error){0};
   struct stela_air_constants values = {constants, constant_count};
   return reflection && entry && (constants || !constant_count) ?
             reflect(air, size, entry, constants ? &values : NULL, &r) :
             fail(r.error, STELA_AIR_INPUT, "invalid reflection request");
}

/* A binding reflection in progress: the structure each member row is or
 * points to, whose members follow once every row before it is filled. */
struct binding_reflection {
   const char *entry;
   const struct stela_air_inspection *info;
   struct stela_metal_bindings *out;
   const struct stela_air_layout *layouts[STELA_METAL_MEMBERS];
   struct stela_air_error *error;
   bool found;
};

static bool
binding_name(struct binding_reflection *r, const char *bytes, size_t size, uint32_t *offset)
{
   return copy_name(r->error, r->out->names, &r->out->names_bytes, STELA_METAL_BINDING_NAME_BYTES,
                    bytes, size, offset);
}

static bool
add_member(const struct stela_air_layout_member *member, void *data)
{
   struct binding_reflection *r = data;
   struct stela_metal_bindings *out = r->out;
   if (out->member_count == STELA_METAL_MEMBERS)
      return fail(r->error, STELA_AIR_LIMIT, "reflected members exceed their capacity");
   bool texture = member->kind == STELA_AIR_FIELD_TEXTURE;
   struct stela_metal_member *row = &out->members[out->member_count];
   *row = (struct stela_metal_member){
      .offset = member->offset, .size = member->size, .elements = member->elements,
      .count = member->count, .name_bytes = (uint32_t)member->name_bytes,
      .type_bytes = (uint32_t)member->type_name_bytes, .argument = member->argument,
      .kind = member->kind, .id = member->id,
      .access = texture ? texture_access(member->texture.access) :
                          buffer_access(member->buffer.access),
      .size_pointed = member->buffer.type_size, .alignment_pointed = member->buffer.type_alignment,
      .texture_type = member->texture.type, .primitive = member->texture.primitive,
      .depth = member->texture.depth,
   };
   r->layouts[out->member_count++] = member->layout;
   return binding_name(r, member->name, member->name_bytes, &row->name) &&
          binding_name(r, member->type_name, member->type_name_bytes, &row->type);
}

/* Appends layout's members, as *first and *count. */
static bool
add_members(struct binding_reflection *r, const struct stela_air_layout *layout, uint32_t *first,
            uint32_t *count)
{
   *first = r->out->member_count;
   if (layout && !stela_air_layout_members(r->info, layout, add_member, r))
      return false;
   *count = r->out->member_count - *first;
   return true;
}

static bool
add_binding(struct binding_reflection *r, enum stela_metal_binding_kind kind, unsigned index,
            unsigned length, const char *name, size_t name_bytes, const char *type,
            size_t type_bytes, bool used, const struct stela_air_layout *layout,
            struct stela_metal_binding binding)
{
   struct stela_metal_bindings *out = r->out;
   if (out->binding_count == STELA_METAL_BINDINGS)
      return fail(r->error, STELA_AIR_LIMIT, "bindings exceed the arguments");
   binding.kind = kind;
   binding.index = index;
   binding.count = length ? length : 1;
   binding.used = used;
   binding.name_bytes = (uint32_t)name_bytes;
   binding.type_bytes = (uint32_t)type_bytes;
   struct stela_metal_binding *row = &out->bindings[out->binding_count++];
   *row = binding;
   return binding_name(r, name, name_bytes, &row->name) &&
          binding_name(r, type, type_bytes, &row->type) &&
          add_members(r, layout, &row->members, &row->member_count);
}

static bool
add_buffer_binding(struct binding_reflection *r, const struct stela_air_buffer *buffer,
                   bool argument_buffer, const char *name, size_t name_bytes, const char *type,
                   size_t type_bytes, bool used, const struct stela_air_layout *layout)
{
   enum stela_metal_binding_kind kind =
      buffer->address_space == 3 ? STELA_METAL_BINDING_THREADGROUP :
      buffer->address_space == 6 ? STELA_METAL_BINDING_OBJECT_PAYLOAD :
      buffer->function_table ? STELA_METAL_BINDING_FUNCTION_TABLE : STELA_METAL_BINDING_BUFFER;
   return add_binding(r, kind, buffer->index, buffer->array ? buffer->length : 0, name, name_bytes,
                      type, type_bytes, used, layout,
                      (struct stela_metal_binding){
                         .access = buffer_access(buffer->access),
                         .argument_buffer = argument_buffer, .size = buffer->type_size,
                         .alignment = buffer->type_alignment});
}

static bool
add_texture_binding(struct binding_reflection *r, const struct stela_air_texture *texture,
                    const char *name, size_t name_bytes, const char *type, size_t type_bytes,
                    bool used)
{
   return add_binding(r, STELA_METAL_BINDING_TEXTURE, texture->index, texture->length, name,
                      name_bytes, type, type_bytes, used, NULL,
                      (struct stela_metal_binding){
                         .access = texture_access(texture->access), .texture_type = texture->type,
                         .primitive = texture->primitive, .depth = texture->depth});
}

static bool
add_sampler_binding(struct binding_reflection *r, const struct stela_air_sampler *sampler,
                    const char *name, size_t name_bytes, const char *type, size_t type_bytes,
                    bool used)
{
   return add_binding(r, STELA_METAL_BINDING_SAMPLER, sampler->index, sampler->length, name,
                      name_bytes, type, type_bytes, used, NULL,
                      (struct stela_metal_binding){.access = STELA_METAL_ACCESS_READ});
}

/* Fills the members of the structures member rows are or point to, each
 * after every row before it, so that each structure's are contiguous. */
static bool
nested_members(struct binding_reflection *r)
{
   for (uint32_t i = 0; i < r->out->member_count; i++) {
      struct stela_metal_member *row = &r->out->members[i];
      if (!add_members(r, r->layouts[i], &row->members, &row->member_count))
         return false;
   }
   return true;
}

static bool
bind_kernel(const struct stela_air_inspection *info, const struct stela_air_entry *entry,
            void *data)
{
   struct binding_reflection *r = data;
   if (entry->name_bytes != strlen(r->entry) || memcmp(entry->name, r->entry, entry->name_bytes))
      return true;
   r->found = true;
   r->info = info;
   for (unsigned i = 0; i < entry->argument_count; i++) {
      const struct stela_air_argument_info *a = &entry->arguments[i];
      /* The unspecialized function's reflection lists the arguments its
       * constants may yet leave out. */
      bool added = a->absent && !a->conditional ? true :
                   a->kind == 1 ? add_buffer_binding(r, &a->buffer, a->argument_buffer, a->name,
                                                     a->name_bytes, a->type_name,
                                                     a->type_name_bytes, a->used, a->layout) :
                   a->kind == 3 ? add_texture_binding(r, &a->texture, a->name, a->name_bytes,
                                                      a->type_name, a->type_name_bytes, a->used) :
                   a->kind == 4 ? add_sampler_binding(r, &a->sampler, a->name, a->name_bytes,
                                                      a->type_name, a->type_name_bytes, a->used) :
                   /* A kernel's one imageblock, which Metal lists at index 0. */
                   a->kind == 6 ? add_binding(r, STELA_METAL_BINDING_IMAGEBLOCK, 0, 0, a->name,
                                              a->name_bytes, a->type_name, a->type_name_bytes,
                                              a->used, NULL,
                                              (struct stela_metal_binding){
                                                 .access = STELA_METAL_ACCESS_READ}) :
                                  true;
      if (!added)
         return false;
   }
   return nested_members(r);
}

static bool
bind_stage(const struct stela_air_inspection *info, const struct stela_air_stage_entry *entry,
           void *data)
{
   struct binding_reflection *r = data;
   r->found = true;
   r->info = info;
   for (unsigned i = 0; i < entry->input_count; i++) {
      const struct stela_air_stage_io *io = &entry->inputs[i];
      bool added = io->absent && !io->conditional ? true :
                   io->semantic == STELA_AIR_STAGE_BUFFER || io->semantic == STELA_AIR_PAYLOAD ?
                      add_buffer_binding(r, &io->buffer, io->argument_buffer, io->name,
                                         io->name_bytes, io->type_name, io->type_name_bytes,
                                         io->used, io->layout) :
                   io->semantic == STELA_AIR_STAGE_TEXTURE ?
                      add_texture_binding(r, &io->texture, io->name, io->name_bytes,
                                          io->type_name, io->type_name_bytes, io->used) :
                   io->semantic == STELA_AIR_STAGE_SAMPLER ?
                      add_sampler_binding(r, &io->sampler, io->name, io->name_bytes,
                                          io->type_name, io->type_name_bytes, io->used) :
                      true;
      if (!added)
         return false;
   }
   return nested_members(r);
}

bool
stela_metal_reflect_bindings(const void *air, size_t size, const char *entry,
                             const struct stela_air_constant_value *constants,
                             uint32_t constant_count, struct stela_metal_bindings *bindings,
                             struct stela_air_error *error)
{
   struct stela_air_error local = {0};
   error = error ? error : &local;
   *error = (struct stela_air_error){0};
   if (!bindings || !entry || (!constants && constant_count))
      return fail(error, STELA_AIR_INPUT, "invalid binding reflection request");
   struct binding_reflection *r = calloc(1, sizeof(*r));
   if (!r)
      return fail(error, STELA_AIR_ALLOCATION, "binding reflection allocation failed");
   *r = (struct binding_reflection){.entry = entry, .out = bindings, .error = error};
   bindings->binding_count = bindings->member_count = bindings->names_bytes = 0;
   struct stela_air_constants values = {constants, constant_count};
   unsigned stage = 0;
   bool reflected = inspect_entry(air, size, entry, constants ? &values : NULL, bind_kernel,
                                  bind_stage, r, &r->found, &stage, error);
   free(r);
   return reflected;
}

bool
stela_metal_sampler_states(const void *air, size_t size,
                           uint64_t states[STELA_METAL_SAMPLER_STATES], uint32_t *count,
                           struct stela_air_error *error)
{
   unsigned listed;
   bool valid = stela_air_sampler_states(air, size, states, &listed, error);
   *count = listed;
   return valid;
}

/* The binding data of the metal_abi callbacks, which read its first member,
 * and of the threadgroup callback. */
struct binding {
   struct stela_metal_stage stage;
   const struct stela_metal_specialization *specialization;
};

/* The dispatch's setThreadgroupMemoryLength:atIndex: of the buffer. */
static bool
bind_threadgroup(const struct stela_air_buffer *buffer, struct stela_air_threadgroup_view *view,
                 void *data)
{
   const struct stela_metal_specialization *specialization =
      ((const struct binding *)data)->specialization;
   for (unsigned i = 0; i < specialization->threadgroup_length_count; i++) {
      const struct stela_metal_threadgroup_length *length = &specialization->threadgroup_lengths[i];
      if (length->index != buffer->index)
         continue;
      *view = (struct stela_air_threadgroup_view){
         .size = length->bytes,
         .alignment = MAX2(buffer->type_alignment, STELA_METAL_THREADGROUP_ALIGNMENT),
         .available_access = STELA_AIR_READ_WRITE,
      };
      return length->bytes != 0;
   }
   return false;
}

/* stela_air_fetch_attribute over the pipeline's stage-in layout: at a
 * patch's IDs for post-tessellation vertex functions, else at a kernel's
 * position in the grid. */
static nir_def *
fetch_attribute(nir_builder *b, unsigned location, nir_def *at, nir_def *point, bool integer,
                void *data)
{
   const struct stela_metal_specialization *specialization =
      ((const struct binding *)data)->specialization;
   if (!specialization->stage_in)
      return NULL;
   if (specialization->tessellation)
      return stela_metal_fetch_patch_attribute(b, specialization->stage_in,
                                               specialization->tessellation, location, at, point,
                                               integer);
   return point ? NULL : stela_metal_fetch_stage_in(b, specialization->stage_in, location, at, integer);
}

/* decode_vertex_input of the vertex inputs the pipeline fetches as words,
 * in the formats the specialization names. */
static nir_def *
decode_vertex_input(nir_builder *b, unsigned location, nir_def *words, void *data)
{
   const struct stela_metal_specialization *specialization =
      ((const struct binding *)data)->specialization;
   unsigned format = location < ARRAY_SIZE(specialization->vertex_formats) ?
      specialization->vertex_formats[location] : 0;
   unsigned bytes = stela_metal_vertex_format_bytes(format);
   if (!bytes || words->bit_size != 32 || words->num_components * 4 != bytes)
      return NULL;
   return stela_metal_decode_attribute(b, words, format);
}

/* The 32-bit words of each vertex input the pipeline fetches as words, by
 * location; false when a format named is none of whole words. */
static bool
vertex_words(const struct stela_metal_specialization *specialization,
             uint8_t words[static AIR_MAX_VERTEX_ATTRIBUTES], bool *any)
{
   *any = false;
   for (unsigned location = 0; location < ARRAY_SIZE(specialization->vertex_formats); location++) {
      unsigned format = specialization->vertex_formats[location];
      if (!format)
         continue;
      unsigned bytes = stela_metal_vertex_format_bytes(format);
      if (location >= AIR_MAX_VERTEX_ATTRIBUTES || !bytes || bytes % 4)
         return false;
      words[location] = bytes / 4;
      *any = true;
   }
   return true;
}

static enum stela_metal_table
stage_table(enum stela_metal_function_stage stage)
{
   switch (stage) {
   case STELA_METAL_FRAGMENT: return STELA_METAL_TABLE_FRAGMENT;
   case STELA_METAL_MESH: return STELA_METAL_TABLE_MESH;
   default: return STELA_METAL_TABLE_PRIMARY;
   }
}

/* The stage's rows of the pipeline's varyings: those it reads or writes, at
 * their linked locations, of its own types. */
static bool
varying_rows(const struct stela_metal_reflection *reflection,
             const struct stela_metal_specialization *specialization,
             struct stela_air_varying_slot *rows, unsigned *count, struct stela_air_error *error)
{
   *count = 0;
   if (specialization->varying_count > STELA_METAL_VARYINGS)
      return fail(error, STELA_AIR_LIMIT, "linked varyings exceed the slots");
   for (unsigned i = 0; i < reflection->varying_count; i++) {
      const struct stela_metal_varying *varying = &reflection->varyings[i];
      const char *identifier = reflection->names + varying->identifier;
      unsigned location = 0;
      while (location < specialization->varying_count &&
             (specialization->varyings[location].identifier_bytes != varying->identifier_bytes ||
              memcmp(specialization->varyings[location].identifier, identifier,
                     varying->identifier_bytes)))
         location++;
      if (location == specialization->varying_count)
         return fail(error, STELA_AIR_ARGUMENT, "a varying is missing from the pipeline's linkage");
      const struct stela_metal_linked_varying *linked = &specialization->varyings[location];
      rows[(*count)++] = (struct stela_air_varying_slot){
         .identifier = linked->identifier, .identifier_bytes = linked->identifier_bytes,
         .primitive = varying->primitive, .components = varying->components,
         .location = location, .per_primitive = linked->per_primitive,
      };
   }
   return true;
}

/* The texel layout of a renderable color VkFormat, as Mesa's Vulkan
 * runtime maps it (vk_format_to_pipe_format), which the hosted compiler
 * does not link; PIPE_FORMAT_NONE for any other. */
static enum pipe_format
attachment_format(uint32_t format)
{
   switch ((VkFormat)format) {
   case VK_FORMAT_A8_UNORM_KHR: return PIPE_FORMAT_A8_UNORM;
   case VK_FORMAT_R8_UNORM: return PIPE_FORMAT_R8_UNORM;
   case VK_FORMAT_R8_SRGB: return PIPE_FORMAT_R8_SRGB;
   case VK_FORMAT_R8_SNORM: return PIPE_FORMAT_R8_SNORM;
   case VK_FORMAT_R8_UINT: return PIPE_FORMAT_R8_UINT;
   case VK_FORMAT_R8_SINT: return PIPE_FORMAT_R8_SINT;
   case VK_FORMAT_R16_UNORM: return PIPE_FORMAT_R16_UNORM;
   case VK_FORMAT_R16_SNORM: return PIPE_FORMAT_R16_SNORM;
   case VK_FORMAT_R16_UINT: return PIPE_FORMAT_R16_UINT;
   case VK_FORMAT_R16_SINT: return PIPE_FORMAT_R16_SINT;
   case VK_FORMAT_R16_SFLOAT: return PIPE_FORMAT_R16_FLOAT;
   case VK_FORMAT_R8G8_UNORM: return PIPE_FORMAT_R8G8_UNORM;
   case VK_FORMAT_R8G8_SRGB: return PIPE_FORMAT_R8G8_SRGB;
   case VK_FORMAT_R8G8_SNORM: return PIPE_FORMAT_R8G8_SNORM;
   case VK_FORMAT_R8G8_UINT: return PIPE_FORMAT_R8G8_UINT;
   case VK_FORMAT_R8G8_SINT: return PIPE_FORMAT_R8G8_SINT;
   case VK_FORMAT_R5G6B5_UNORM_PACK16: return PIPE_FORMAT_B5G6R5_UNORM;
   case VK_FORMAT_R5G5B5A1_UNORM_PACK16: return PIPE_FORMAT_A1B5G5R5_UNORM;
   case VK_FORMAT_R4G4B4A4_UNORM_PACK16: return PIPE_FORMAT_A4B4G4R4_UNORM;
   case VK_FORMAT_A1R5G5B5_UNORM_PACK16: return PIPE_FORMAT_B5G5R5A1_UNORM;
   case VK_FORMAT_R8G8B8_UNORM: return PIPE_FORMAT_R8G8B8_UNORM;
   case VK_FORMAT_R8G8B8_SNORM: return PIPE_FORMAT_R8G8B8_SNORM;
   case VK_FORMAT_R8G8B8_UINT: return PIPE_FORMAT_R8G8B8_UINT;
   case VK_FORMAT_R8G8B8_SINT: return PIPE_FORMAT_R8G8B8_SINT;
   case VK_FORMAT_R32_UINT: return PIPE_FORMAT_R32_UINT;
   case VK_FORMAT_R32_SINT: return PIPE_FORMAT_R32_SINT;
   case VK_FORMAT_R32_SFLOAT: return PIPE_FORMAT_R32_FLOAT;
   case VK_FORMAT_R16G16_UNORM: return PIPE_FORMAT_R16G16_UNORM;
   case VK_FORMAT_R16G16_SNORM: return PIPE_FORMAT_R16G16_SNORM;
   case VK_FORMAT_R16G16_UINT: return PIPE_FORMAT_R16G16_UINT;
   case VK_FORMAT_R16G16_SINT: return PIPE_FORMAT_R16G16_SINT;
   case VK_FORMAT_R16G16_SFLOAT: return PIPE_FORMAT_R16G16_FLOAT;
   case VK_FORMAT_R8G8B8A8_UNORM: return PIPE_FORMAT_R8G8B8A8_UNORM;
   case VK_FORMAT_R8G8B8A8_SRGB: return PIPE_FORMAT_R8G8B8A8_SRGB;
   case VK_FORMAT_R8G8B8A8_SNORM: return PIPE_FORMAT_R8G8B8A8_SNORM;
   case VK_FORMAT_R8G8B8A8_UINT: return PIPE_FORMAT_R8G8B8A8_UINT;
   case VK_FORMAT_R8G8B8A8_SINT: return PIPE_FORMAT_R8G8B8A8_SINT;
   case VK_FORMAT_B8G8R8A8_UNORM: return PIPE_FORMAT_B8G8R8A8_UNORM;
   case VK_FORMAT_B8G8R8A8_SRGB: return PIPE_FORMAT_B8G8R8A8_SRGB;
   case VK_FORMAT_A2B10G10R10_UNORM_PACK32: return PIPE_FORMAT_R10G10B10A2_UNORM;
   case VK_FORMAT_A2B10G10R10_UINT_PACK32: return PIPE_FORMAT_R10G10B10A2_UINT;
   case VK_FORMAT_B10G11R11_UFLOAT_PACK32: return PIPE_FORMAT_R11G11B10_FLOAT;
   case VK_FORMAT_E5B9G9R9_UFLOAT_PACK32: return PIPE_FORMAT_R9G9B9E5_FLOAT;
   case VK_FORMAT_A2R10G10B10_UNORM_PACK32: return PIPE_FORMAT_B10G10R10A2_UNORM;
   case VK_FORMAT_R16G16B16_UNORM: return PIPE_FORMAT_R16G16B16_UNORM;
   case VK_FORMAT_R16G16B16_SNORM: return PIPE_FORMAT_R16G16B16_SNORM;
   case VK_FORMAT_R16G16B16_UINT: return PIPE_FORMAT_R16G16B16_UINT;
   case VK_FORMAT_R16G16B16_SINT: return PIPE_FORMAT_R16G16B16_SINT;
   case VK_FORMAT_R16G16B16_SFLOAT: return PIPE_FORMAT_R16G16B16_FLOAT;
   case VK_FORMAT_R32G32_UINT: return PIPE_FORMAT_R32G32_UINT;
   case VK_FORMAT_R32G32_SINT: return PIPE_FORMAT_R32G32_SINT;
   case VK_FORMAT_R32G32_SFLOAT: return PIPE_FORMAT_R32G32_FLOAT;
   case VK_FORMAT_R16G16B16A16_UNORM: return PIPE_FORMAT_R16G16B16A16_UNORM;
   case VK_FORMAT_R16G16B16A16_SNORM: return PIPE_FORMAT_R16G16B16A16_SNORM;
   case VK_FORMAT_R16G16B16A16_UINT: return PIPE_FORMAT_R16G16B16A16_UINT;
   case VK_FORMAT_R16G16B16A16_SINT: return PIPE_FORMAT_R16G16B16A16_SINT;
   case VK_FORMAT_R16G16B16A16_SFLOAT: return PIPE_FORMAT_R16G16B16A16_FLOAT;
   case VK_FORMAT_R32G32B32_UINT: return PIPE_FORMAT_R32G32B32_UINT;
   case VK_FORMAT_R32G32B32_SINT: return PIPE_FORMAT_R32G32B32_SINT;
   case VK_FORMAT_R32G32B32_SFLOAT: return PIPE_FORMAT_R32G32B32_FLOAT;
   case VK_FORMAT_R32G32B32A32_UINT: return PIPE_FORMAT_R32G32B32A32_UINT;
   case VK_FORMAT_R32G32B32A32_SINT: return PIPE_FORMAT_R32G32B32A32_SINT;
   case VK_FORMAT_R32G32B32A32_SFLOAT: return PIPE_FORMAT_R32G32B32A32_FLOAT;
   default: return PIPE_FORMAT_NONE;
   }
}

/* Each attachment's three bits name an enum stela_air_extended_range; the
 * bits past the last attachment's are zero. */
static bool
extended_range_valid(uint32_t kinds)
{
   for (unsigned n = 0; n < AIR_MAX_RENDER_TARGETS; n++) {
      unsigned kind = kinds >> (n * STELA_AIR_EXTENDED_RANGE_BITS) &
                      BITFIELD_MASK(STELA_AIR_EXTENDED_RANGE_BITS);
      if (kind > STELA_AIR_EXTENDED_RANGE_PLANES)
         return false;
   }
   return (uint64_t)kinds >> (AIR_MAX_RENDER_TARGETS * STELA_AIR_EXTENDED_RANGE_BITS) == 0;
}

/* The specialization's storage attachments as the translator takes them. */
static bool
attachment_accesses(const struct stela_metal_specialization *specialization,
                    struct stela_air_attachment_access *accesses, struct stela_air_error *error)
{
   if (!specialization->attachment_count)
      return true;
   if (specialization->attachment_count > AIR_MAX_RENDER_TARGETS || !specialization->attachments ||
       !specialization->samples)
      return fail(error, STELA_AIR_INPUT, "invalid storage attachments");
   for (uint32_t n = 0; n < specialization->attachment_count; n++) {
      const struct stela_metal_attachment *attachment = &specialization->attachments[n];
      if (!attachment->format)
         continue;
      enum pipe_format format = attachment_format(attachment->format);
      if (format == PIPE_FORMAT_NONE || util_format_get_blocksize(format) != attachment->bytes ||
          attachment->rgb_op > STELA_AIR_BLEND_MAX || attachment->alpha_op > STELA_AIR_BLEND_MAX ||
          attachment->source_rgb > STELA_AIR_BLEND_ONE_MINUS_SOURCE1_ALPHA ||
          attachment->destination_rgb > STELA_AIR_BLEND_ONE_MINUS_SOURCE1_ALPHA ||
          attachment->source_alpha > STELA_AIR_BLEND_ONE_MINUS_SOURCE1_ALPHA ||
          attachment->destination_alpha > STELA_AIR_BLEND_ONE_MINUS_SOURCE1_ALPHA ||
          attachment->write_mask > 0xf ||
          (attachment->planes &&
           (attachment->planes & STELA_AIR_PLANES_SUBSAMPLING) != STELA_AIR_PLANES_FULL) ||
          attachment->planes > (STELA_AIR_PLANES_FULL | STELA_AIR_PLANES_YCBCR))
         return fail(error, STELA_AIR_INPUT, "invalid storage attachment");
      /* MTLColorWriteMask names red to alpha from its fourth bit down. */
      unsigned mask = attachment->write_mask;
      accesses[n] = (struct stela_air_attachment_access){
         .format = format,
         .blend = attachment->blend != 0,
         .write_mask = (mask >> 3 & 1) | (mask >> 1 & 2) | (mask << 1 & 4) | (mask << 3 & 8),
         .rgb_op = attachment->rgb_op,
         .alpha_op = attachment->alpha_op,
         .source_rgb = attachment->source_rgb,
         .destination_rgb = attachment->destination_rgb,
         .source_alpha = attachment->source_alpha,
         .destination_alpha = attachment->destination_alpha,
         .planes = attachment->planes,
      };
   }
   return true;
}

/* Thread memory of at most this many 32-bit words lives in registers. */
#define REGISTER_THREAD_WORDS 16

/* Moves small thread memory out of scratch into registers. The front end
 * lays a function's thread memory out as scratch; when it fits in
 * REGISTER_THREAD_WORDS words, scratch becomes one array of words, every
 * dynamic index into it a select tree over its elements, and each element
 * an SSA value. Larger thread memory stays in scratch: more registers per
 * thread would cost occupancy. Thread memory is private to its thread, so
 * only its storage changes, never a value. */
static void
thread_memory_to_registers(nir_shader *shader)
{
   if (!shader->scratch_size || shader->scratch_size > REGISTER_THREAD_WORDS * 4)
      return;
   nir_lower_scratch_to_var(shader);
   nir_lower_indirect_derefs_to_if_else_trees(shader, nir_var_function_temp,
                                              REGISTER_THREAD_WORDS);
   nir_lower_vars_to_ssa(shader);
   nir_remove_dead_variables(shader, nir_var_function_temp, NULL);
   /* Words read before any store become undefined values carried through
    * loops; zero is one of their values, and what NAK picks for them too. */
   nir_lower_undef_to_zero(shader, NULL);
   nir_opt_dce(shader);
}

nir_shader *
stela_metal_compile_reflected(const void *air, size_t size, const char *entry,
                              const struct stela_metal_target *target,
                              const struct stela_metal_specialization *specialization,
                              const struct stela_metal_reflection *reflection,
                              struct stela_air_error *error)
{
   struct stela_air_error local = {0};
   if (!error)
      error = &local;
   *error = (struct stela_air_error){0};
   if (!entry || !target || !target->nir || !specialization ||
       (specialization->threadgroup_length_count && !specialization->threadgroup_lengths) ||
       (specialization->varying_count && !specialization->varyings) ||
       (specialization->constant_count && !specialization->constants) ||
       (specialization->module_count && !specialization->modules) ||
       (specialization->linked_function_count && !specialization->linked_functions) ||
       (specialization->sampler_count && !specialization->samplers)) {
      fail(error, STELA_AIR_INPUT, "invalid compilation request");
      return NULL;
   }
   struct stela_metal_reflection *owned_reflection = NULL;
   nir_shader *shader = NULL;
   if (!reflection) {
      owned_reflection = malloc(sizeof(*owned_reflection));
      if (!owned_reflection) {
         fail(error, STELA_AIR_ALLOCATION, "reflection allocation failed");
         return NULL;
      }
      struct reflecting r = {.entry = entry, .out = owned_reflection, .error = error};
      struct stela_air_constants constants = {specialization->constants,
                                              specialization->constant_count};
      if (!reflect(air, size, entry, &constants, &r))
         goto done;
      reflection = owned_reflection;
   }
   enum stela_metal_function_stage stage = reflection->stage;
   if (stage == STELA_METAL_VISIBLE) {
      fail(error, STELA_AIR_ARGUMENT, "visible functions compile into the functions that link them");
      goto done;
   }
   bool compute = stage == STELA_METAL_KERNEL || stage == STELA_METAL_OBJECT ||
                  stage == STELA_METAL_MESH;
   /* A post-tessellation vertex function is its pipeline's evaluation
    * stage, which no vertex stage can take the place of. */
   const struct stela_metal_tessellation *tessellation = specialization->tessellation;
   bool patch = reflection->patch != STELA_AIR_PATCH_NONE;
   if (patch && !tessellation) {
      fail(error, STELA_AIR_ARGUMENT, "post-tessellation vertex functions need tessellation stages");
      goto done;
   }
   if (tessellation && (!patch || tessellation->quad != (reflection->patch == STELA_AIR_PATCH_QUAD))) {
      fail(error, STELA_AIR_INPUT, "tessellation of another patch");
      goto done;
   }
   /* Only kernels and patch functions fetch their stage-in. */
   const struct stela_metal_stage_in *stage_in = specialization->stage_in;
   if (stage_in && !patch && stage != STELA_METAL_KERNEL) {
      fail(error, STELA_AIR_INPUT, "stage-in for a function that does not fetch it");
      goto done;
   }
   /* Only a vertex input fetches words, for a vertex function. */
   uint8_t words[AIR_MAX_VERTEX_ATTRIBUTES] = {0};
   bool fetches_words;
   if (!vertex_words(specialization, words, &fetches_words) ||
       (fetches_words && (stage != STELA_METAL_VERTEX || patch))) {
      fail(error, STELA_AIR_INPUT, "vertex input words for a function that does not take them");
      goto done;
   }
   struct binding binding = {{stage_table(stage)}, specialization};
   struct stela_air_options resources = {
      .max_instructions = target->max_instructions,
      .bind_buffer = stela_metal_bind_buffer,
      .binding_data = &binding,
      .fp_profile = STELA_AIR_FP_METAL,
      .max_private_bytes = target->max_private_bytes,
      .max_constant_bytes = target->max_constant_bytes,
      .bind_texture = stela_metal_bind_texture,
      .bind_sampler = stela_metal_bind_sampler,
      .bind_attachment = stela_metal_bind_attachment,
      .bind_depth_stencil = stela_metal_bind_depth_stencil,
      .bind_texture_id = stela_metal_bind_texture_id,
      .texture_view_word = stela_metal_texture_view_word,
      .bind_sampler_id = stela_metal_bind_sampler_id,
      .resource_id = stela_metal_resource_id,
      .constexpr_samplers = specialization->samplers,
      .constexpr_sampler_count = specialization->sampler_count,
      .texture_levels = stela_metal_texture_levels,
      .texture_null = stela_metal_texture_null,
      .texture_srgb_storage = stela_metal_texture_srgb_storage,
      .texture_extended_storage = stela_metal_texture_extended_storage,
      .sampler_never = stela_metal_sampler_never,
      .sampler_filter = stela_metal_sampler_filter,
      .texture_filter_class = stela_metal_texture_filter_class,
      .dynamic_bicubic = (specialization->bicubic & STELA_METAL_BICUBIC_IDS) != 0,
      .bicubic_samplers = specialization->bicubic & ~STELA_METAL_BICUBIC_IDS,
      .texture_planes = stela_metal_texture_planes,
      .dynamic_planes = true,
      .command_count = stela_metal_command_count,
      .encode_command = stela_metal_encode_command,
      .load_device_value = stela_metal_load_device_value,
      .fetch_attribute = patch || stage_in ? fetch_attribute : NULL,
      .constants = specialization->constants,
      .constant_count = specialization->constant_count,
      .modules = specialization->modules,
      .module_count = specialization->module_count,
      .linked_functions = specialization->linked_functions,
      .linked_function_count = specialization->linked_function_count,
      .max_call_stack_depth = specialization->max_call_stack_depth,
      .round_toward_zero = specialization->round_toward_zero != 0,
   };
   if (compute) {
      memcpy(resources.workgroup_size, specialization->workgroup_size,
             sizeof(resources.workgroup_size));
      memcpy(resources.imageblock_size, specialization->imageblock_size,
             sizeof(resources.imageblock_size));
      resources.bind_threadgroup = bind_threadgroup;
      resources.max_threadgroup_bytes = target->max_threadgroup_bytes;
   }
   if (specialization->dynamic_threadgroups) {
      if (stage != STELA_METAL_KERNEL || specialization->threadgroup_length_count) {
         fail(error, STELA_AIR_INPUT, "dynamic threadgroups of a kernel without lengths");
         goto done;
      }
      /* Threadgroup buffers share what the function's own variables leave. */
      uint64_t free_bytes = target->max_threadgroup_bytes -
                            MIN2(reflection->threadgroup_bytes, target->max_threadgroup_bytes);
      resources.dynamic_shape = true;
      resources.bind_threadgroup = NULL;
      resources.dynamic_threadgroup_bytes = (unsigned)(free_bytes & ~(uint64_t)15);
      resources.threadgroup_offset = stela_metal_threadgroup_offset;
   }
   if (!extended_range_valid(specialization->extended_range) ||
       (specialization->extended_range && stage != STELA_METAL_FRAGMENT)) {
      fail(error, STELA_AIR_INPUT, "invalid extended-range attachments");
      goto done;
   }
   struct stela_air_tile tile = {.samples = specialization->samples};
   if (specialization->tile) {
      if (stage != STELA_METAL_KERNEL || !specialization->dynamic_threadgroups ||
          specialization->attachment_count > AIR_MAX_RENDER_TARGETS ||
          (specialization->attachment_count && !specialization->attachments)) {
         fail(error, STELA_AIR_INPUT, "a tile function is a kernel of dynamic threadgroups");
         goto done;
      }
      for (uint32_t n = 0; n < specialization->attachment_count; n++) {
         const struct stela_metal_attachment *attachment = &specialization->attachments[n];
         if (!attachment->format)
            continue;
         tile.formats[n] = attachment_format(attachment->format);
         if (tile.formats[n] == PIPE_FORMAT_NONE ||
             util_format_get_blocksize(tile.formats[n]) != attachment->bytes) {
            fail(error, STELA_AIR_INPUT, "invalid tile color attachment");
            goto done;
         }
      }
      resources.tile = &tile;
   }
   if (stage == STELA_METAL_KERNEL) {
      shader = stela_air_to_nir(air, size, entry, target->nir, &resources, error);
      goto done;
   }
   struct stela_air_varying_slot rows[STELA_METAL_VARYINGS];
   unsigned row_count;
   if (!varying_rows(reflection, specialization, rows, &row_count, error))
      goto done;
   static const enum stela_air_stage air_stages[] = {
      [STELA_METAL_VERTEX] = STELA_AIR_VERTEX, [STELA_METAL_FRAGMENT] = STELA_AIR_FRAGMENT,
      [STELA_METAL_MESH] = STELA_AIR_MESH, [STELA_METAL_OBJECT] = STELA_AIR_OBJECT,
   };
   struct stela_air_attachment_access attachments[AIR_MAX_RENDER_TARGETS] = {0};
   if (!attachment_accesses(specialization, attachments, error))
      goto done;
   struct stela_air_stage_options options = {
      .stage = air_stages[stage],
      .resources = resources,
      .varyings = rows,
      .varying_count = row_count,
      .max_varying_slots = target->max_varying_slots,
      .amplified = specialization->amplified,
      .launched_by_grid = specialization->launched_by_grid,
      .max_payload_bytes = target->max_payload_bytes,
      .max_mesh_vertices = target->max_mesh_vertices,
      .max_mesh_primitives = target->max_mesh_primitives,
      .stencil_export = target->stencil_export != 0,
      .fragment_tests = target->fragment_tests != 0,
      .self_tests = (specialization->fragment_tests & STELA_METAL_SELF_TESTS) != 0,
      .alpha_to_coverage = (specialization->fragment_tests & STELA_METAL_ALPHA_TO_COVERAGE) != 0,
      .alpha_to_one = (specialization->fragment_tests & STELA_METAL_ALPHA_TO_ONE) != 0,
      .raster_order_groups = target->raster_order_groups != 0,
      .attachments = specialization->attachment_count ? attachments : NULL,
      .samples = specialization->samples,
      .extended_range = specialization->extended_range,
      .vertex_input_words = fetches_words ? words : NULL,
      .decode_vertex_input = decode_vertex_input,
   };
   shader = stela_air_metal_stage_to_nir(air, size, entry, target->nir, &options, error);
done:
   if (shader)
      thread_memory_to_registers(shader);
   free(owned_reflection);
   return shader;
}

nir_shader *
stela_metal_compile(const void *air, size_t size, const char *entry,
                    const struct stela_metal_target *target,
                    const struct stela_metal_specialization *specialization,
                    struct stela_air_error *error)
{
   return stela_metal_compile_reflected(air, size, entry, target, specialization, NULL, error);
}

void
stela_metal_tessellation_stages(const struct stela_metal_target *target,
                                const struct stela_metal_tessellation *tessellation,
                                nir_shader *stages[2])
{
   stages[0] = stela_metal_tessellation_vertex(target->nir);
   stages[1] = stela_metal_tessellation_control(target->nir, tessellation);
}

void
stela_metal_release(nir_shader *shader)
{
   ralloc_free(shader);
}

/* The buffer slots def's address derives from, one bit each: loads of a
 * stage table's buffer addresses (metal_abi.h) at constant offsets, through
 * address arithmetic and selections. */
static uint32_t
address_slots(nir_def *def, unsigned depth)
{
   if (!depth)
      return 0;
   nir_instr *instr = nir_def_instr(def);
   if (instr->type == nir_instr_type_phi) {
      uint32_t slots = 0;
      nir_foreach_phi_src(src, nir_instr_as_phi(instr))
         slots |= address_slots(src->src.ssa, depth - 1);
      return slots;
   }
   if (instr->type == nir_instr_type_alu) {
      nir_alu_instr *alu = nir_instr_as_alu(instr);
      uint32_t slots = 0;
      for (unsigned i = 0; i < nir_op_infos[alu->op].num_inputs; i++) {
         if (alu->src[i].src.ssa->bit_size == 64)
            slots |= address_slots(alu->src[i].src.ssa, depth - 1);
      }
      return slots;
   }
   if (instr->type != nir_instr_type_intrinsic)
      return 0;
   nir_intrinsic_instr *load = nir_instr_as_intrinsic(instr);
   nir_instr *sum = nir_def_instr(load->src[0].ssa);
   if (load->intrinsic != nir_intrinsic_load_global || def->bit_size != 64 ||
       sum->type != nir_instr_type_alu || nir_instr_as_alu(sum)->op != nir_op_iadd)
      return 0;
   nir_alu_instr *add = nir_instr_as_alu(sum);
   for (unsigned i = 0; i < 2; i++) {
      nir_instr *table = nir_def_instr(add->src[i].src.ssa);
      nir_instr *offset = nir_def_instr(add->src[!i].src.ssa);
      if (table->type != nir_instr_type_intrinsic || offset->type != nir_instr_type_load_const)
         continue;
      nir_intrinsic_instr *push = nir_instr_as_intrinsic(table);
      if (push->intrinsic != nir_intrinsic_load_push_constant || !nir_src_is_const(push->src[0]) ||
          nir_src_as_uint(push->src[0]) + nir_intrinsic_base(push) - STELA_METAL_PUSH_TABLES >=
             8u * (STELA_METAL_TABLE_MESH + 1u))
         continue;
      uint64_t at = nir_const_value_as_uint(
                       nir_instr_as_load_const(offset)->value[add->src[!i].swizzle[0]], 64) -
                    STELA_METAL_TABLE_BUFFER_ADDRESSES;
      if (at % 8u || at >= 8u * STELA_METAL_BUFFERS)
         continue;
      uint32_t slots = 0;
      for (unsigned c = 0; c < def->num_components && at / 8u + c < STELA_METAL_BUFFERS; c++)
         slots |= 1u << (at / 8u + c);
      return slots;
   }
   return 0;
}

uint32_t
stela_metal_buffer_reads(const nir_shader *shader)
{
   uint32_t reads = 0;
   nir_foreach_function_impl(impl, shader) {
      nir_foreach_block(block, impl) {
         nir_foreach_instr(instr, block) {
            if (instr->type != nir_instr_type_intrinsic)
               continue;
            nir_intrinsic_instr *access = nir_instr_as_intrinsic(instr);
            switch (access->intrinsic) {
            case nir_intrinsic_load_global:
            case nir_intrinsic_load_global_constant:
            case nir_intrinsic_global_atomic:
            case nir_intrinsic_global_atomic_swap:
               reads |= address_slots(access->src[0].ssa, 16);
               break;
            case nir_intrinsic_store_global:
               reads |= address_slots(access->src[1].ssa, 16);
               break;
            default:
               break;
            }
         }
      }
   }
   return reads;
}
