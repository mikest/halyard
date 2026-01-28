#include "liquid_area.h"


void LiquidArea::_bind_methods() {
    EXPORT_PROPERTY_RANGED(Variant::FLOAT, density, LiquidArea, "1,10000,1,hide_slider,suffix:kg/m^3");
    EXPORT_PROPERTY(Variant::VECTOR3, current_speed, LiquidArea);

	ClassDB::bind_method(D_METHOD("is_point_submerged", "global_point"), &LiquidArea::is_point_submerged);
    ClassDB::bind_method(D_METHOD("get_liquid_transform", "global_point"), &LiquidArea::get_liquid_transform);

	// virtuals
	GDVIRTUAL_BIND(update_transforms_for_points, "global_points", "transforms")
}


LiquidArea::LiquidArea() {
}


LiquidArea::~LiquidArea() {
}


LiquidArea *LiquidArea::get_liquid_area(SceneTree *p_tree) {
    LiquidArea *liquid_area = nullptr;
    
    if (p_tree) {
        Node* root = p_tree->get_current_scene();
        if (root) {
            auto nodes = root->find_children("*", "LiquidArea", true, false);
            if (nodes.size()) {
                liquid_area = Object::cast_to<LiquidArea>(nodes.front());
            }
        }
    }

    return liquid_area;
}


bool LiquidArea::is_point_submerged(const Vector3 &global_point) const {
    Transform3D liquid_transform = get_global_transform();
    Vector3 local_point = liquid_transform.affine_inverse().xform(global_point);
    return local_point.y < 0.0f;
}


Transform3D LiquidArea::get_liquid_transform(const Vector3 &global_point) const {
    if (GDVIRTUAL_IS_OVERRIDDEN(update_transforms_for_points)) {
		TypedArray<Transform3D> ret_val;
        PackedVector3Array points;
        points.append(global_point);

		GDVIRTUAL_CALL(update_transforms_for_points, points, ret_val);
		return ret_val[0];
	}
    else
    {
        // Fast return if not overridden: just return the area's global transform.
        return get_global_transform();
    }
}


void LiquidArea::update_transforms_for_points(const PackedVector3Array &global_points,
    TypedArray<Transform3D> r_transforms) const {
    
    if (GDVIRTUAL_IS_OVERRIDDEN(update_transforms_for_points)) {
		Transform3D ret_val;
		GDVIRTUAL_CALL(update_transforms_for_points, global_points, r_transforms);
		return;
	}
    else
    {
        // Default implementation: fill all transforms with the area's global transform.
        int point_count = global_points.size();
        r_transforms.resize(point_count);
        Transform3D area_transform = get_global_transform();

        for (int i = 0; i < point_count; i++) {
            r_transforms[i] = area_transform;
        }
    }
}
