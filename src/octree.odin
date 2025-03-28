package main

import "core:math"
import "core:math/linalg"
import gl "vendor:OpenGL"

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
octree_new :: proc(bounds: struct {
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
octree_clear :: proc(ot: ^Octree) {
	if ot == nil do return

	// Clear objects in this node
	clear(&ot.objects)

	// Recursively clear and free child nodes
	for &node in ot.nodes {
		if node != nil {
			octree_clear(node)
			free(node)
			node = nil
		}
	}
}

// Insert a sphere into the Octree
octree_insert :: proc(ot: ^Octree, spheres: []PhysicsBody, index: int) {
	pos := spheres[index].pos

	// If position is outside this node, skip
	if !octree_contains(ot, pos) do return

	// If we can still insert or reached max depth, add here
	if len(ot.objects) < ot.max_objects || ot.level >= ot.max_levels {
		append(&ot.objects, index)
		return
	}

	// Otherwise, split and insert into children
	if ot.nodes[0] == nil do octree_split(ot)
	for node in ot.nodes do octree_insert(node, spheres, index)
}

// Split the Octree into 8 children
octree_split :: proc(ot: ^Octree) {
	if ot.level >= ot.max_levels do return

	mid_x := (ot.bounds.max.x + ot.bounds.min.x) * 0.5
	mid_y := (ot.bounds.max.y + ot.bounds.min.y) * 0.5
	mid_z := (ot.bounds.max.z + ot.bounds.min.z) * 0.5

	// Define the 8 octants
	ot.nodes[0] = octree_new(
		{{ot.bounds.min.x, ot.bounds.min.y, ot.bounds.min.z}, {mid_x, mid_y, mid_z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Bottom-left-front
	ot.nodes[1] = octree_new(
		{{mid_x, ot.bounds.min.y, ot.bounds.min.z}, {ot.bounds.max.x, mid_y, mid_z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Bottom-right-front
	ot.nodes[2] = octree_new(
		{{ot.bounds.min.x, mid_y, ot.bounds.min.z}, {mid_x, ot.bounds.max.y, mid_z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Top-left-front
	ot.nodes[3] = octree_new(
		{{mid_x, mid_y, ot.bounds.min.z}, {ot.bounds.max.x, ot.bounds.max.y, mid_z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Top-right-front
	ot.nodes[4] = octree_new(
		{{ot.bounds.min.x, ot.bounds.min.y, mid_z}, {mid_x, mid_y, ot.bounds.max.z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Bottom-left-back
	ot.nodes[5] = octree_new(
		{{mid_x, ot.bounds.min.y, mid_z}, {ot.bounds.max.x, mid_y, ot.bounds.max.z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Bottom-right-back
	ot.nodes[6] = octree_new(
		{{ot.bounds.min.x, mid_y, mid_z}, {mid_x, ot.bounds.max.y, ot.bounds.max.z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Top-left-back
	ot.nodes[7] = octree_new(
		{{mid_x, mid_y, mid_z}, {ot.bounds.max.x, ot.bounds.max.y, ot.bounds.max.z}},
		ot.max_objects,
		ot.max_levels,
		ot.level + 1,
	) // Top-right-back
}

// Check if a point is inside this node
octree_contains :: proc(ot: ^Octree, pos: [3]f32) -> bool {
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
octree_query :: proc(
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
	for node in ot.nodes do octree_query(node, spheres, pos, radius, results)
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

when ODIN_DEBUG {
	// Create a wireframe cube VAO if it doesn't exist
	cube_initialized := false
	cube_vao, cube_vbo, cube_ebo: u32

	// Debug rendering for octree visualization
	octree_render_debug :: proc(ot: ^Octree, shader: Shader, view_proj: ^matrix[4, 4]f32) {

		if !cube_initialized {
			// Cube vertices (positions only)
			vertices := [8][3]f32 {
				{-0.5, -0.5, -0.5},
				{0.5, -0.5, -0.5},
				{0.5, 0.5, -0.5},
				{-0.5, 0.5, -0.5},
				{-0.5, -0.5, 0.5},
				{0.5, -0.5, 0.5},
				{0.5, 0.5, 0.5},
				{-0.5, 0.5, 0.5},
			}

			// Cube edges (line indices)
			indices := [24]u32 {
				0,
				1,
				1,
				2,
				2,
				3,
				3,
				0, // Bottom
				4,
				5,
				5,
				6,
				6,
				7,
				7,
				4, // Top
				0,
				4,
				1,
				5,
				2,
				6,
				3,
				7, // Sides
			}

			gl.GenVertexArrays(1, &cube_vao)
			gl.GenBuffers(1, &cube_vbo)
			gl.GenBuffers(1, &cube_ebo)

			gl.BindVertexArray(cube_vao)

			gl.BindBuffer(gl.ARRAY_BUFFER, cube_vbo)
			gl.BufferData(
				gl.ARRAY_BUFFER,
				size_of(vertices),
				raw_data(vertices[:]),
				gl.STATIC_DRAW,
			)

			gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, cube_ebo)
			gl.BufferData(
				gl.ELEMENT_ARRAY_BUFFER,
				size_of(indices),
				raw_data(indices[:]),
				gl.STATIC_DRAW,
			)

			gl.EnableVertexAttribArray(0)
			gl.VertexAttribPointer(0, 3, gl.FLOAT, false, size_of([3]f32), 0)

			gl.BindVertexArray(0)
			cube_initialized = true
		}

		// Set up shader
		gl.UseProgram(shader.id)
		gl.UniformMatrix4fv(
			gl.GetUniformLocation(shader.id, "view_proj"),
			1,
			false,
			&view_proj[0, 0],
		)
		gl.Uniform3f(gl.GetUniformLocation(shader.id, "color"), 0, 1, 0) // Green

		// Recursively render all octree nodes
		octree_render_debug_nodes(ot, cube_vao, shader)
	}

	octree_render_debug_nodes :: proc(ot: ^Octree, cube_vao: u32, shader: Shader) {
		if ot == nil do return

		// Calculate model matrix for this node's bounds
		center := (ot.bounds.max + ot.bounds.min) * 0.5
		size := ot.bounds.max - ot.bounds.min

		model := linalg.matrix4_scale([3]f32{size.x, size.y, size.z})
		model = linalg.matrix4_translate([3]f32{center.x, center.y, center.z}) * model

		// Draw wireframe cube
		gl.UniformMatrix4fv(gl.GetUniformLocation(shader.id, "model"), 1, false, &model[0, 0])

		gl.BindVertexArray(cube_vao)
		gl.DrawElements(gl.LINES, 24, gl.UNSIGNED_INT, nil)
		gl.BindVertexArray(0)

		// Recursively render child nodes
		for node in ot.nodes {
			octree_render_debug_nodes(node, cube_vao, shader)
		}
	}
}
