package main

import "core:math"
import "core:math/linalg"

Octree :: struct {
	bounds:      struct {
		min, max: [3]f32,
	}, // 3D bounds (x, y, z)
	max_objects: int, // Max objects per node before splitting
	max_levels:  int, // Max recursion depth
	level:       int, // Current depth
	objects:     [dynamic]int, // Stores indices of spheres
	nodes:       [8]^Octree, // 8 children (octants)
}

// Initialize a new Octree
new_octree :: proc(bounds: struct {
		min, max: [3]f32,
	}, max_objects := 4, max_levels := 5, level := 0) -> ^Octree {
	ot := new(Octree)
	ot.bounds = bounds
	ot.max_objects = max_objects
	ot.max_levels = max_levels
	ot.level = level
	return ot
}

// Clear the Octree (recursively)
clear_octree :: proc(ot: ^Octree) {
	if ot == nil do return

	// Clear objects in this node
	clear(&ot.objects)

	// Recursively clear and free child nodes
	for &node in ot.nodes {
		if node != nil {
			clear_octree(node)
			free(node)
			node = nil
		}
	}
}

// Insert a sphere into the Octree
insert_octree :: proc(ot: ^Octree, spheres: []PhysicsBody, index: int) {
	pos := spheres[index].pos

	// If position is outside this node, skip
	if !contains_octree(ot, pos) do return

	// If we can still insert or reached max depth, add here
	if len(ot.objects) < ot.max_objects || ot.level >= ot.max_levels {
		append(&ot.objects, index)
		return
	}

	// Otherwise, split and insert into children
	if ot.nodes[0] == nil do split_octree(ot)
	for node in ot.nodes do insert_octree(node, spheres, index)
}

// Split the Octree into 8 children
split_octree :: proc(ot: ^Octree) {
	if ot.level >= ot.max_levels do return

	mid_x := (ot.bounds.max.x + ot.bounds.min.x) * 0.5
	mid_y := (ot.bounds.max.y + ot.bounds.min.y) * 0.5
	mid_z := (ot.bounds.max.z + ot.bounds.min.z) * 0.5

	// Define the 8 octants
	ot.nodes[0] = new_octree(
		{{ot.bounds.min.x, ot.bounds.min.y, ot.bounds.min.z}, {mid_x, mid_y, mid_z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Bottom-left-front
	ot.nodes[1] = new_octree(
		{{mid_x, ot.bounds.min.y, ot.bounds.min.z}, {ot.bounds.max.x, mid_y, mid_z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Bottom-right-front
	ot.nodes[2] = new_octree(
		{{ot.bounds.min.x, mid_y, ot.bounds.min.z}, {mid_x, ot.bounds.max.y, mid_z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Top-left-front
	ot.nodes[3] = new_octree(
		{{mid_x, mid_y, ot.bounds.min.z}, {ot.bounds.max.x, ot.bounds.max.y, mid_z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Top-right-front
	ot.nodes[4] = new_octree(
		{{ot.bounds.min.x, ot.bounds.min.y, mid_z}, {mid_x, mid_y, ot.bounds.max.z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Bottom-left-back
	ot.nodes[5] = new_octree(
		{{mid_x, ot.bounds.min.y, mid_z}, {ot.bounds.max.x, mid_y, ot.bounds.max.z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Bottom-right-back
	ot.nodes[6] = new_octree(
		{{ot.bounds.min.x, mid_y, mid_z}, {mid_x, ot.bounds.max.y, ot.bounds.max.z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Top-left-back
	ot.nodes[7] = new_octree(
		{{mid_x, mid_y, mid_z}, {ot.bounds.max.x, ot.bounds.max.y, ot.bounds.max.z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Top-right-back
}

// Check if a point is inside this node
contains_octree :: proc(ot: ^Octree, pos: [3]f32) -> bool {
	return(
		pos.x >= ot.bounds.min.x &&
		pos.x <= ot.bounds.max.x &&
		pos.y >= ot.bounds.min.y &&
		pos.y <= ot.bounds.max.y &&
		pos.z >= ot.bounds.min.z &&
		pos.z <= ot.bounds.max.z \
	)
}

// Query nearby objects (returns indices of potential collisions)
query_octree :: proc(
	ot: ^Octree,
	spheres: []PhysicsBody,
	pos: [3]f32,
	radius: f32,
	results: ^[dynamic]int,
) {
	if ot == nil do return

	// Skip if the sphere doesn't intersect this node
	if !intersects_sphere_aabb(pos, radius, ot.bounds) do return

	// Check objects in this node
	for obj_index in ot.objects {
		other_pos := spheres[obj_index].pos
		if linalg.distance(pos, other_pos) <= radius * 2.0 { 	// Check 2*radius for safety
			append(results, obj_index)
		}
	}

	// Recursively check children
	for node in ot.nodes do query_octree(node, spheres, pos, radius, results)
}

// Helper: Check if a sphere intersects an AABB (axis-aligned bounding box)
intersects_sphere_aabb :: proc(sphere_pos: [3]f32, radius: f32, aabb: struct {
		min, max: [3]f32,
	}) -> bool {
	closest_x := max(aabb.min.x, min(sphere_pos.x, aabb.max.x))
	closest_y := max(aabb.min.y, min(sphere_pos.y, aabb.max.y))
	closest_z := max(aabb.min.z, min(sphere_pos.z, aabb.max.z))
	distance_sq :=
		math.pow(sphere_pos.x - closest_x, 2) +
		math.pow(sphere_pos.y - closest_y, 2) +
		math.pow(sphere_pos.z - closest_z, 2)
	return distance_sq <= (radius * radius)
}
