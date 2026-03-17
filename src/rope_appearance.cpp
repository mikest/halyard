#include "rope_appearance.h"

// ---------------------------------------------------------------------------
// Helper: register an array-count property (mirrors rope.cpp's ADD_ARRAY_COUNT)
// ---------------------------------------------------------------------------
#ifndef ADD_ARRAY_COUNT
#define ADD_ARRAY_COUNT(m_label, m_count_property, m_count_property_setter, m_count_property_getter, m_prefix) \
	ClassDB::add_property(RopeAppearance::get_class_static(),                                                  \
		PropertyInfo(Variant::INT, m_count_property, PROPERTY_HINT_NONE, "",                                   \
			PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_ARRAY,                                                     \
			vformat("%s,%s", m_label, m_prefix)),                                                              \
		m_count_property_setter, m_count_property_getter)
#endif

#define ATTACHMENT_DISTRIBUTION_HINT "Absolute:0,Relative:1,Uniform:2,Scalar:3"
#define ATTACHMENT_FROM_HINT "Start:0,End:1"

const char ATTACHMENTS_KEY[] = "attachments/";
const char ATT_OFFSET_KEY[] = "offset";
const char ATT_FROM_KEY[] = "from";
const char ATT_NODE_PATH_KEY[] = "node_path";

// ---------------------------------------------------------------------------

void RopeAppearance::_bind_methods() {
	// Distribution enum constants
	BIND_ENUM_CONSTANT(ABSOLUTE);
	BIND_ENUM_CONSTANT(RELATIVE);
	BIND_ENUM_CONSTANT(UNIFORM);
	BIND_ENUM_CONSTANT(SCALAR);

	// rope parameters
	EXPORT_PROPERTY(Variant::INT, rope_sides, RopeAppearance);
	EXPORT_PROPERTY(Variant::FLOAT, rope_twist, RopeAppearance);
	EXPORT_PROPERTY(Variant::INT, rope_lod, RopeAppearance);

	ClassDB::bind_method(D_METHOD("set_material", "material"), &RopeAppearance::set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &RopeAppearance::get_material);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "material", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_material", "get_material");

	ClassDB::bind_method(D_METHOD("set_link_mesh", "array_mesh"), &RopeAppearance::set_array_mesh);
	ClassDB::bind_method(D_METHOD("get_link_mesh"), &RopeAppearance::get_array_mesh);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "link_mesh", PROPERTY_HINT_RESOURCE_TYPE, "ArrayMesh"), "set_link_mesh", "get_link_mesh");

	// geometry trimming
	EXPORT_PROPERTY(Variant::FLOAT, start_offset, RopeAppearance);
	EXPORT_PROPERTY(Variant::FLOAT, end_offset, RopeAppearance);

	// Attachment distribution + array
	EXPORT_PROPERTY_ENUM(attachment_distribution, ATTACHMENT_DISTRIBUTION_HINT, RopeAppearance);

	ClassDB::bind_method(D_METHOD("set_attachment_count", "count"), &RopeAppearance::set_attachment_count);
	ClassDB::bind_method(D_METHOD("get_attachment_count"), &RopeAppearance::get_attachment_count);
	ClassDB::bind_method(D_METHOD("clear_attachments"), &RopeAppearance::clear_attachments);
	ADD_ARRAY_COUNT("Attachments", "attachment_count", "set_attachment_count", "get_attachment_count", ATTACHMENTS_KEY);

	ClassDB::bind_method(D_METHOD("set_attachment_offset", "idx", "offset"), &RopeAppearance::set_attachment_offset);
	ClassDB::bind_method(D_METHOD("get_attachment_offset", "idx"), &RopeAppearance::get_attachment_offset);

	ClassDB::bind_method(D_METHOD("set_attachment_from", "idx", "from"), &RopeAppearance::set_attachment_from);
	ClassDB::bind_method(D_METHOD("get_attachment_from", "idx"), &RopeAppearance::get_attachment_from);

	ClassDB::bind_method(D_METHOD("set_attachment_nodepath", "idx", "path"), &RopeAppearance::set_attachment_nodepath);
	ClassDB::bind_method(D_METHOD("get_attachment_nodepath", "idx"), &RopeAppearance::get_attachment_nodepath);
}

// ---------------------------------------------------------------------------
// Dynamic property list for the attachment array
// ---------------------------------------------------------------------------

void RopeAppearance::_get_property_list(List<PropertyInfo> *p_list) const {
	ERR_FAIL_NULL(p_list);

	for (uint32_t i = 0; i < _attachments.size(); i++) {
		uint32_t usage;
		String path = ATTACHMENTS_KEY + itos(i) + "/";

		// offset is hidden for UNIFORM and REAL (position is computed, not user-set)
		// Note: Distribution::REAL == 4; avoid the identifier directly due to macro conflicts.
		const bool is_computed = _attachment_distribution == Distribution::UNIFORM
				|| static_cast<int>(_attachment_distribution) == 4;
		usage = is_computed ? PROPERTY_USAGE_NONE : PROPERTY_USAGE_DEFAULT;
		p_list->push_back(PropertyInfo(Variant::FLOAT, path + ATT_OFFSET_KEY, PROPERTY_HINT_NONE, "suffix:m", usage));

		// from (start/end) is only meaningful for ABSOLUTE
		usage = (_attachment_distribution == Distribution::ABSOLUTE) ? PROPERTY_USAGE_DEFAULT : PROPERTY_USAGE_NONE;
		p_list->push_back(PropertyInfo(Variant::INT, path + ATT_FROM_KEY, PROPERTY_HINT_ENUM, ATTACHMENT_FROM_HINT, usage));

		p_list->push_back(PropertyInfo(Variant::NODE_PATH, path + ATT_NODE_PATH_KEY,
				PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node3D"));
	}
}

bool RopeAppearance::_set(const StringName &p_name, const Variant &p_property) {
	String path = p_name;
	if (path.begins_with(ATTACHMENTS_KEY)) {
		int which = path.get_slicec('/', 1).to_int();
		String what = path.get_slicec('/', 2);
		ERR_FAIL_INDEX_V(which, (int)_attachments.size(), false);

		if (what == ATT_OFFSET_KEY) {
			set_attachment_offset(which, (float)p_property);
		} else if (what == ATT_FROM_KEY) {
			set_attachment_from(which, (int)p_property);
		} else if (what == ATT_NODE_PATH_KEY) {
			set_attachment_nodepath(which, (NodePath)p_property);
		} else {
			return false;
		}
		return true;
	}
	return false;
}

bool RopeAppearance::_get(const StringName &p_name, Variant &r_property) const {
	String path = p_name;
	if (path.begins_with(ATTACHMENTS_KEY)) {
		int which = path.get_slicec('/', 1).to_int();
		String what = path.get_slicec('/', 2);
		ERR_FAIL_INDEX_V(which, (int)_attachments.size(), false);

		if (what == ATT_OFFSET_KEY) {
			r_property = get_attachment_offset(which);
		} else if (what == ATT_FROM_KEY) {
			r_property = get_attachment_from(which);
		} else if (what == ATT_NODE_PATH_KEY) {
			r_property = (NodePath)get_attachment_nodepath(which);
		} else {
			return false;
		}
		return true;
	}
	return false;
}

// ---------------------------------------------------------------------------
// Basic appearance properties
// ---------------------------------------------------------------------------

void RopeAppearance::set_rope_sides(int p_rope_sides) {
	_rope_sides = Math::clamp(p_rope_sides, -1, MAX_ROPE_SIDES);
	_notify_change();
}

int RopeAppearance::get_rope_sides() const {
	return _rope_sides;
}

void RopeAppearance::set_material(const Ref<Material> &p_material) {
	_material = p_material;
	_notify_change();
}

Ref<Material> RopeAppearance::get_material() const {
	return _material;
}

void RopeAppearance::set_array_mesh(const Ref<ArrayMesh> &p_mesh) {
	_mesh = p_mesh;
	_notify_change();
}

Ref<ArrayMesh> RopeAppearance::get_array_mesh() const {
	return _mesh;
}

// ---------------------------------------------------------------------------
// Attachment count / distribution
// ---------------------------------------------------------------------------

void RopeAppearance::set_attachment_count(int count) {
	ERR_FAIL_COND(count < 0);
	if ((int)_attachments.size() != count) {
		_attachments.resize(count);
		_notify_attachments_changed();
	}
}

int RopeAppearance::get_attachment_count() const {
	return (int)_attachments.size();
}

void RopeAppearance::clear_attachments() {
	_attachments.clear();
	_notify_attachments_changed();
}

void RopeAppearance::set_attachment_distribution(int val) {
	const Distribution new_dist = (Distribution)val;
	if (new_dist == _attachment_distribution) {
		return;
	}
	_attachment_distribution = new_dist;
	_notify_attachments_changed();
}

int RopeAppearance::get_attachment_distribution() const {
	return (int)_attachment_distribution;
}

// ---------------------------------------------------------------------------
// Per-element attachment accessors
// ---------------------------------------------------------------------------

void RopeAppearance::set_attachment_offset(int idx, float offset) {
	ERR_FAIL_INDEX(idx, (int)_attachments.size());
	if (_attachments[idx].offset != offset) {
		_attachments[idx].offset = offset;
		_notify_attachments_changed();
	}
}

float RopeAppearance::get_attachment_offset(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_attachments.size(), 0.0f);
	return _attachments[idx].offset;
}

void RopeAppearance::set_attachment_from(int idx, int from) {
	ERR_FAIL_INDEX(idx, (int)_attachments.size());
	bool new_from_end = (from == 1);
	if (_attachments[idx].from_end != new_from_end) {
		_attachments[idx].from_end = new_from_end;
		_notify_attachments_changed();
	}
}

int RopeAppearance::get_attachment_from(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_attachments.size(), 0);
	return _attachments[idx].from_end ? 1 : 0;
}

void RopeAppearance::set_attachment_nodepath(int idx, const NodePath &path) {
	ERR_FAIL_INDEX(idx, (int)_attachments.size());
	if (_attachments[idx].node_path != path) {
		_attachments[idx].node_path = path;
		_notify_attachments_changed();
	}
}

NodePath RopeAppearance::get_attachment_nodepath(int idx) const {
	ERR_FAIL_INDEX_V(idx, (int)_attachments.size(), NodePath());
	return _attachments[idx].node_path;
}
