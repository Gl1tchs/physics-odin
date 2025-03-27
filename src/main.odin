package main

import "core:fmt"

import "core:math"
import "core:math/linalg"
import gl "vendor:OpenGL"
import "vendor:glfw"

WINDOW_TITLE :: "Odin Physics"

WINDOW_WIDTH :: 800
WINDOW_HEIGHT :: 600

main :: proc() {
	if !glfw.Init() {
		fmt.println("Failed to initialize GLFW")
		return
	}

	defer glfw.Terminate()

	window, ok := create_window(WINDOW_TITLE, WINDOW_WIDTH, WINDOW_HEIGHT).?
	if !ok {
		fmt.println("Failed to create GLFW window")
		return
	}

	defer destroy_window(&window)

	// Enable depth testing
	gl.Enable(gl.DEPTH_TEST)

	sphere_primitive := create_sphere_primitive(1.0, 36, 18)
	defer destroy_sphere_primitive(&sphere_primitive)

	model := linalg.identity(matrix[4, 4]f32)
	angle := 0.0

	view := linalg.matrix4_look_at([3]f32{0, 0, 5}, [3]f32{0, 0, 0}, [3]f32{0, 1, 0})
	projection := linalg.matrix4_perspective(
		math.to_radians(f32(45.0)),
		f32(WINDOW_WIDTH) / f32(WINDOW_HEIGHT),
		0.1,
		100.0,
	)

	// Set uniforms
	gl.UseProgram(sphere_primitive.shader.id)

	gl.UniformMatrix4fv(
		gl.GetUniformLocation(sphere_primitive.shader.id, "view"),
		1,
		false,
		&view[0][0],
	)
	gl.UniformMatrix4fv(
		gl.GetUniformLocation(sphere_primitive.shader.id, "projection"),
		1,
		false,
		&projection[0][0],
	)

	current_time := glfw.GetTime()
	last_time := current_time
	for !window_should_close(window) {
		window_poll_events()

		current_time = glfw.GetTime()
		delta_time := current_time - last_time
		last_time = current_time

		angle += 50.0 * delta_time
		// model = linalg.matrix4_translate(linalg.Vector3f32{math.sin(f32(angle)), 0, 0})
		model = linalg.matrix4_rotate(math.to_radians(f32(angle)), linalg.VECTOR3F32_Y_AXIS)

		gl.ClearColor(0.1, 0.1, 0.1, 1.0)
		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		// Draw sphere
		gl.UseProgram(sphere_primitive.shader.id)

		gl.UniformMatrix4fv(
			gl.GetUniformLocation(sphere_primitive.shader.id, "model"),
			1,
			false,
			&model[0][0],
		)

		gl.BindVertexArray(sphere_primitive.vao)
		gl.DrawElements(gl.TRIANGLES, cast(i32)len(sphere_primitive.indices), gl.UNSIGNED_INT, nil)
		gl.BindVertexArray(0)

		window_swap_buffers(window)
	}
}
