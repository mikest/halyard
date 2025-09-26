#pragma once

#include "property_utils.h"
#include "rope.h"
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/core/gdvirtual.gen.inc>
#include <godot_cpp/core/type_info.hpp>
#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

class RopeAttachmentsBase : public Resource {
	GDCLASS(RopeAttachmentsBase, Resource)

protected:
	static void _bind_methods();

public:
	RopeAttachmentsBase() = default;
	virtual ~RopeAttachmentsBase() override = default;

	// return the number of rope attachments
	GDVIRTUAL1RC(uint64_t, get_attachment_count, const Rope *);
	virtual uint64_t get_attachment_count(const Rope *) const;

	// return the scalar position for the attachment at the index
	// 0 is start, and 1 is end position.
	GDVIRTUAL2RC(float, get_attachment_position, uint64_t, const Rope *);
	virtual float get_attachment_position(uint64_t, const Rope *) const;

	// return the nodepath for the attachment
	GDVIRTUAL2RC(NodePath, get_attachment_nodepath, uint64_t, const Rope *);
	virtual NodePath get_attachment_nodepath(uint64_t, const Rope *) const;
};
