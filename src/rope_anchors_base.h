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
class RopeAnchorsBase : public Resource {
	GDCLASS(RopeAnchorsBase, Resource)

protected:
	static void _bind_methods();

public:
	RopeAnchorsBase() = default;
	virtual ~RopeAnchorsBase() override = default;

	// return the number of rope anchors
	GDVIRTUAL1RC(uint64_t, get_count, const Rope *);
	virtual uint64_t get_count(const Rope *) const;

	// return the scalar positions the rope should anchor at.
	GDVIRTUAL2RC(float, get_position, uint64_t, const Rope *);
	virtual float get_position(uint64_t, const Rope *) const;

	// return the anchor transforms for the index
	GDVIRTUAL2RC(Transform3D, get_transform, uint64_t, const Rope *);
	virtual Transform3D get_transform(uint64_t, const Rope *) const;
};
