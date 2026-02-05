/* NodeDebug

Utility class for adding debug visualization to nodes.
This removes some boilerplate from nodes that want to add debug meshes.

Sublasses should implement _update_debug_mesh at the very least, as well as
accessors for debug properties.
*/

#pragma once

#include <godot_cpp/classes/area3d.hpp>

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>

#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

using namespace godot;

class NodeDebug {
    bool _debug_mesh_dirty = false;
    Node *_self = nullptr;

protected:
    // the class the mesh instance will be attached to
    Node3D *_node = nullptr;

	bool _show_debug = false;
	Color _debug_color = Color(0.0f, 1.0f, 0.5f, 0.8f);
	Ref<ArrayMesh> _debug_mesh;
	MeshInstance3D* _debug_mesh_instance = nullptr;
    Ref<StandardMaterial3D> _debug_material;
    Ref<StandardMaterial3D> _marker_material;
    Ref<StandardMaterial3D> _inverted_material;

    // sets both node and node3d pointers
    void _set_debug_owner_node(Node3D* p_node) { _node = p_node; }   

    // optional creation/destruction hooks for subclasses
	virtual void _create_debug_mesh() {}
	virtual void _destroy_debug_mesh() {}

    // required mesh building
    virtual void _update_debug_mesh() = 0;

    // Must be called by the node's notification handler
    void _debug_notification(int p_what);

    // can be called by subclasses to create/destroy the debug mesh
    void _internal_create_debug_mesh();
    void _internal_update_debug_mesh();
	void _internal_destroy_debug_mesh();

    void _add_marker_surface(Vector3 p_position, bool inverted);

public:
    NodeDebug(Node *p_self);
	virtual ~NodeDebug();

    void set_debug_mesh_dirty(bool dirty=true) { _debug_mesh_dirty = dirty; }
    bool is_debug_mesh_dirty() const { return _debug_mesh_dirty; }
};