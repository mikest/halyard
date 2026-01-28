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
                if (!_debug_mesh_instance) {
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
        _debug_material->set_albedo(_debug_color);

        // always render last
        _debug_material->set_render_priority(StandardMaterial3D::RENDER_PRIORITY_MAX);

        // disable depth test in game so we can see the mesh
        if (!Engine::get_singleton()->is_editor_hint()) {
            _debug_material->set_flag(StandardMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);
        }
    }

    _create_debug_mesh();
}

void NodeDebug::_internal_update_debug_mesh() {
    if (_debug_mesh_instance == nullptr) return;
	if (_debug_mesh.is_valid() == false) return;
    if (_debug_material.is_valid() == false) return;
    
    // clear previous surfaces
	_debug_mesh->clear_surfaces();

    // build new surfaces
    _update_debug_mesh();

    // move debug mesh
    if (_node) {
	    _debug_mesh_instance->set_global_transform(_node->get_global_transform());
    }
}


void NodeDebug::_internal_destroy_debug_mesh() {
    // additional destruction.
    _destroy_debug_mesh();

    // clean up
	if (_debug_mesh_instance) {
        if(_node)
            _node->remove_child(_debug_mesh_instance);
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