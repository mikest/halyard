#include "node_debug.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

using namespace godot;

NodeDebug::NodeDebug() {
}

NodeDebug::~NodeDebug() {
    _internal_destroy_debug_mesh();
}

void NodeDebug::_debug_notification(int p_what) {
    switch (p_what) {
        case Node::NOTIFICATION_EXIT_TREE: {
            _internal_destroy_debug_mesh();
            _node = nullptr;
        } break;

        case Node::NOTIFICATION_INTERNAL_PROCESS: {
            // lazy add/remove the debug meshes
            if (_show_debug) {
                if (!_debug_mesh_instance && _node) {
                    _internal_create_debug_mesh();
                }
            } else if (_debug_mesh_instance) {
                _internal_destroy_debug_mesh();
            }

            // update the mesh if dirty
            if ( _show_debug && _debug_mesh_dirty && _debug_mesh_instance) {
                _internal_update_debug_mesh();
                _debug_mesh_dirty = false;
            }
        } break;
    }

}

void NodeDebug::_internal_create_debug_mesh() {
    ERR_FAIL_NULL_MSG(_node, "NodeDebug::_internal_create_debug_mesh called but _node is null");

    // skip for now if not in tree
	if (_node->is_inside_tree() == false) return;

    // attach the mesh instance
	if (!_debug_mesh_instance) {
		_debug_mesh_instance = memnew(MeshInstance3D);
		_debug_mesh_instance->set_name(String("_") + _node->get_name() + String("_DebugMesh"));
		
		// add the child to this node
		_node->add_child(_debug_mesh_instance, true,  Node::INTERNAL_MODE_BACK);

		// attach mesh
		if (!_debug_mesh.is_valid()) {
			_debug_mesh.instantiate();
		}
		_debug_mesh_instance->set_mesh(_debug_mesh);
		_debug_mesh_dirty = true;
	}

    // create the material
    if (!_debug_material.is_valid()) {
        _debug_material.instantiate();
        
        // create the material, will be added to surfaces later
        _debug_material->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
        _debug_material->set_depth_draw_mode(StandardMaterial3D::DEPTH_DRAW_DISABLED);
        _debug_material->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
        _debug_material->set_flag(StandardMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
        _debug_material->set_albedo(_debug_color);

        // always render last
        _debug_material->set_render_priority(StandardMaterial3D::RENDER_PRIORITY_MAX);

        // disable depth test in game so we can see the mesh
        if (!Engine::get_singleton()->is_editor_hint()) {
            _debug_material->set_flag(StandardMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
        }
    }

    // a material for the markers
    if (!_marker_material.is_valid()) {
        _marker_material.instantiate();
        
        // marker is billboarded
        _marker_material->set_billboard_mode(StandardMaterial3D::BILLBOARD_ENABLED);

        _marker_material->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
        _marker_material->set_depth_draw_mode(StandardMaterial3D::DEPTH_DRAW_DISABLED);
        _marker_material->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
        _marker_material->set_flag(StandardMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
        _marker_material->set_albedo(_debug_color);

        // disable depth test in game so we can see the mesh
        if (!Engine::get_singleton()->is_editor_hint()) {
            _marker_material->set_flag(StandardMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
        }

        _inverted_material = _marker_material->duplicate();
        _inverted_material->set_albedo(_debug_color.inverted());
    }

    _create_debug_mesh();
}

void NodeDebug::_internal_update_debug_mesh() {
    if (_debug_mesh_instance == nullptr) return;
	if (_debug_mesh.is_valid() == false) return;
    if (_debug_material.is_valid() == false) return;
    
    // clear previous surfaces
	_debug_mesh->clear_surfaces();

    // move debug mesh
    if (_node) {
	    _debug_mesh_instance->set_global_transform(_node->get_global_transform());
    }

    // build new surfaces
    _update_debug_mesh();
}


void NodeDebug::_internal_destroy_debug_mesh() {
    // additional destruction.
    _destroy_debug_mesh();

    // clean up
	if (_debug_mesh_instance) {
        if(_node && _debug_mesh_instance->get_parent() == _node) {
            _node->call_deferred("remove_child", _debug_mesh_instance);
            // _node->remove_child(_debug_mesh_instance);
        }
		_debug_mesh_instance->queue_free();
		_debug_mesh_instance = nullptr;
	}

	if (_debug_mesh.is_valid()) {
		_debug_mesh.unref();
	}

    if (_debug_material.is_valid()) {
        _debug_material.unref();
    }
}


void NodeDebug::_add_marker_surface(Vector3 p_position, bool inverted) {
    if (_debug_mesh.is_valid() == false) return;
    if (_debug_material.is_valid() == false) return;


    // create a marker at the p_position.
    // marker is a two 90 degre arcs, one in the upper left and one in the lower right, opposite each other with a cross in the middle
    PackedVector3Array vertices;
    const float radius = 0.1f;
    const int segments = 6; // segments per 90-degree arc

    float first_quarter = inverted ? 1.0 : 2.0 ;
    float second_quarter = inverted ? 3.0 : 4.0;
    
    // First arc: upper left (90 degrees from 90° to 180°)
    for (int i = 0; i <= segments; ++i) {
        float angle = (first_quarter * Math_PI / 2.0f) + (Math_PI / 2.0f) * float(i) / float(segments);
        float x = radius * Math::cos(angle);
        float y = radius * Math::sin(angle);
        vertices.append(p_position + Vector3(x, y, 0));
    }
    
    // Second arc: lower right (90 degrees from 270° to 360°)
    for (int i = 0; i <= segments; ++i) {
        float angle = (second_quarter * Math_PI / 2.0f) + (Math_PI / 2.0f) * float(i) / float(segments);
        float x = radius * Math::cos(angle);
        float y = radius * Math::sin(angle);
        vertices.append(p_position + Vector3(x, y, 0));
    }
    
    // cross lines
    vertices.append(p_position + Vector3(-radius, 0, 0));
    vertices.append(p_position + Vector3(radius, 0, 0));
    vertices.append(p_position + Vector3(0, -radius, 0));
    vertices.append(p_position + Vector3(0, radius, 0));
    
    PackedInt32Array indices;
    // First arc line segments
    for (int i = 0; i < segments; ++i) {
        indices.append(i);
        indices.append(i + 1);
    }
    // Second arc line segments
    int second_arc_start = segments + 1;
    for (int i = 0; i < segments; ++i) {
        indices.append(second_arc_start + i);
        indices.append(second_arc_start + i + 1);
    }
    // cross lines
    int cross_start = second_arc_start + segments + 1;
    indices.append(cross_start);
    indices.append(cross_start + 1);
    indices.append(cross_start + 2);
    indices.append(cross_start + 3);

    Array arrays;
    arrays.resize(Mesh::ARRAY_MAX);
    arrays[Mesh::ARRAY_VERTEX] = vertices;
    arrays[Mesh::ARRAY_INDEX] = indices;

    int surf_lines = _debug_mesh->get_surface_count();
    _debug_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, arrays);

    _debug_mesh->surface_set_material(surf_lines, inverted ? _inverted_material : _marker_material);
}