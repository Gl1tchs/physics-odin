package main

import "core:math"
import gl "vendor:OpenGL"

SphereVertex :: struct {
	position: [3]f32,
	normal:   [3]f32,
}

SpherePrimitive :: struct {
	vertex_array:    u32,
	vertex_buffer:   u32,
	index_buffer:    u32,
	instance_buffer: u32,
	shader:          Shader,
	index_count:     i32,
}

sphere_primitive_create :: proc(
	radius: f32,
	sector_count: u32,
	stack_count: u32,
) -> SpherePrimitive {
	vertices: [dynamic]SphereVertex
	defer delete(vertices)

	indices: [dynamic]u32
	defer delete(indices)

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
	vertex_array: u32
	gl.GenVertexArrays(1, &vertex_array)
	gl.BindVertexArray(vertex_array)

	vertex_buffer: u32
	gl.GenBuffers(1, &vertex_buffer)
	gl.BindBuffer(gl.ARRAY_BUFFER, vertex_buffer)
	gl.BufferData(
		gl.ARRAY_BUFFER,
		len(vertices) * size_of(SphereVertex),
		&vertices[0],
		gl.STATIC_DRAW,
	)
	gl.BindBuffer(gl.ARRAY_BUFFER, 0)

	// Create instance buffer
	instance_buffer: u32
	gl.GenBuffers(1, &instance_buffer)

	gl.BindBuffer(gl.ARRAY_BUFFER, instance_buffer)
	gl.BufferData(gl.ARRAY_BUFFER, size_of([3]f32) * 100_000, nil, gl.DYNAMIC_DRAW)
	gl.BindBuffer(gl.ARRAY_BUFFER, 0)

	// Set-Up vertex data
	gl.BindBuffer(gl.ARRAY_BUFFER, vertex_buffer)

	gl.EnableVertexArrayAttrib(vertex_array, 0)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, size_of(SphereVertex), 0)

	gl.EnableVertexArrayAttrib(vertex_array, 1)
	gl.VertexAttribPointer(
		1,
		3,
		gl.FLOAT,
		false,
		size_of(SphereVertex),
		offset_of(SphereVertex, normal),
	)

	gl.BindBuffer(gl.ARRAY_BUFFER, instance_buffer)

	gl.EnableVertexAttribArray(2)
	gl.VertexAttribPointer(2, 3, gl.FLOAT, false, 3 * size_of(f32), 0)
	gl.VertexAttribDivisor(2, 1)

	index_buffer: u32
	gl.GenBuffers(1, &index_buffer)
	gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, index_buffer)

	gl.BufferData(
		gl.ELEMENT_ARRAY_BUFFER,
		len(indices) * size_of(u32),
		&indices[0],
		gl.STATIC_DRAW,
	)

	// Create shader program
	shader := load_shader("shaders/sphere.frag", "shaders/sphere.vert")

	gl.BindVertexArray(0)

	return SpherePrimitive {
		vertex_array,
		vertex_buffer,
		index_buffer,
		instance_buffer,
		shader,
		cast(i32)len(indices),
	}
}

sphere_primitive_destroy :: proc(primitive: ^SpherePrimitive) {
	if primitive.vertex_array != 0 {
		gl.DeleteVertexArrays(1, &primitive.vertex_array)
	}
	if primitive.vertex_buffer != 0 {
		gl.DeleteBuffers(1, &primitive.vertex_buffer)
	}
	if primitive.index_buffer != 0 {
		gl.DeleteBuffers(1, &primitive.index_buffer)
	}
	if primitive.shader.id != 0 {
		gl.DeleteProgram(primitive.shader.id)
	}
}
