package main

import "core:math"
import gl "vendor:OpenGL"

SphereVertex :: struct {
	position: [3]f32,
	normal:   [3]f32,
}

SpherePrimitive :: struct {
	vao:      u32,
	vbo:      u32,
	ebo:      u32,
	shader:   Shader,
	vertices: [dynamic]SphereVertex,
	indices:  [dynamic]u32,
}

create_sphere_primitive :: proc(
	radius: f32,
	sector_count: u32,
	stack_count: u32,
) -> SpherePrimitive {
	vertices: [dynamic]SphereVertex
	indices: [dynamic]u32

	length_inv := 1.0 / f32(radius)
	sector_step := 2 * math.PI / f32(sector_count)
	stack_step := math.PI / f32(stack_count)
	sector_angle, stack_angle: f32

	// Generate vertices and normals
	for i in 0 ..= stack_count {
		stack_angle := math.PI / 2 - f32(i) * stack_step
		xy := radius * math.cos(stack_angle)
		z := radius * math.sin(stack_angle)

		for j in 0 ..= sector_count {
			sector_angle = f32(j) * sector_step

			x := xy * math.cos(sector_angle)
			y := xy * math.sin(sector_angle)

			nx := x * length_inv
			ny := y * length_inv
			nz := z * length_inv

			vertex := SphereVertex {
				position = [3]f32{x, y, z},
				normal   = [3]f32{nx, ny, nz},
			}

			append(&vertices, vertex)
		}
	}

	// Generate indices
	k1, k2: u32
	for i in 0 ..< stack_count {
		k1 = i * (sector_count + 1)
		k2 = k1 + sector_count + 1

		for j in 0 ..< sector_count {
			// 2 triangles per sector excluding first and last stacks
			// k1 => k2 => k1+1
			if i != 0 {
				append(&indices, k1)
				append(&indices, k2)
				append(&indices, k1 + 1)
			}

			// k1+1 => k2 => k2+1
			if i != (stack_count - 1) {
				append(&indices, k1 + 1)
				append(&indices, k2)
				append(&indices, k2 + 1)
			}

			k1 += 1
			k2 += 1
		}
	}

	// Create VAO, VBO, and EBO
	vao: u32
	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)

	vbo: u32
	gl.GenBuffers(1, &vbo)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.BufferData(
		gl.ARRAY_BUFFER,
		len(vertices) * size_of(SphereVertex),
		&vertices[0],
		gl.STATIC_DRAW,
	)

	gl.EnableVertexArrayAttrib(vao, 0)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, size_of(SphereVertex), 0)

	gl.EnableVertexArrayAttrib(vao, 1)
	gl.VertexAttribPointer(
		1,
		3,
		gl.FLOAT,
		false,
		size_of(SphereVertex),
		offset_of(SphereVertex, normal),
	)

	ebo: u32
	gl.GenBuffers(1, &ebo)
	gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, ebo)

	gl.BufferData(
		gl.ELEMENT_ARRAY_BUFFER,
		len(indices) * size_of(u32),
		&indices[0],
		gl.STATIC_DRAW,
	)

	// Create shader program
	shader := load_shader("shaders/sphere_fragment.glsl", "shaders/sphere_vertex.glsl")

	gl.BindVertexArray(0)

	return SpherePrimitive{vao, vbo, ebo, shader, vertices, indices}
}

destroy_sphere_primitive :: proc(primitive: ^SpherePrimitive) {
	if primitive.vao != 0 {
		gl.DeleteVertexArrays(1, &primitive.vao)
	}
	if primitive.vbo != 0 {
		gl.DeleteBuffers(1, &primitive.vbo)
	}
	if primitive.ebo != 0 {
		gl.DeleteBuffers(1, &primitive.ebo)
	}
	if primitive.shader.id != 0 {
		gl.DeleteProgram(primitive.shader.id)
	}
}
