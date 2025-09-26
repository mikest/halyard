#pragma once

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

/*
RopePositions is an abstract class used to manage a stored list of
nodepaths and float values.

The float values can represent a scalar distance along the rope regardless of length
or they can represent an offset distance from either the start or the end of the rope.

Due to the way godot::Wrapped works the implementations of the property calls are
implemented as passthrough functions by derrived classes.
*/

using namespace godot;

// performance above this value is going to be... dissappointing.
const int MAX_POSITIONS = 1000;
#define POSITIONS_HINT "Scalar,DistanceFromStart,DistanceFromEnd"

class RopePositions {
public:
	RopePositions() = default;
	virtual ~RopePositions() = default;

	enum Spacing {
		Scalar = 0,
		DistanceFromStart = 1,
		DistanceFromEnd = 2,
	};

protected:
	struct Position {
		float position = 0.0;
		NodePath node = "";
	};

	Vector<Position> _positions;
	Spacing _spacing = Scalar;

	// dynamic list building for properties
	void _rp_get_property_list(List<PropertyInfo> *p_list) const;
	bool _rp_property_can_revert(const StringName &p_name) const;
	bool _rp_property_get_revert(const StringName &p_name, Variant &r_property) const;
	bool _rp_set(const StringName &p_name, const Variant &p_property);
	bool _rp_get(const StringName &p_name, Variant &r_property) const;

	Pair<uint64_t, String> _get_propname_with_index(const StringName &p_name) const;

	// these all call notify on changes
	void _set_count(uint64_t count);
	uint64_t _get_count() const;

	void _set_position(uint64_t idx, float val);
	float _get_position(uint64_t idx) const;
	void _set_nodepath(uint64_t idx, const NodePath &val);
	NodePath _get_nodepath(uint64_t idx) const;

	// this class stores raw position data, derrived classes will need to use the conversion functions
	// as they have the rope length
	void _set_spacing(Spacing val);
	Spacing _get_spacing() const;

	// if spacing is "scalar" these just return the value unmodified
	float _position_for_distance(float distance, float rope_length) const;
	float _distance_for_position(float position, float rope_length) const;

	// to be implemented by subclassers as this class cannot call Object methods.
	virtual void _notify_changed() = 0;
};

VARIANT_ENUM_CAST(RopePositions::Spacing)