package main

import "core:fmt"

import "core:math"
import "core:math/linalg"
import "core:math/rand"
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

	SPHERE_RADIUS :: 1.0
	sphere_primitive := create_sphere_primitive(SPHERE_RADIUS, 36, 18)
	defer destroy_sphere_primitive(&sphere_primitive)

	view := linalg.matrix4_look_at([3]f32{0, 0, 30}, [3]f32{0, 0, 0}, [3]f32{0, 1, 0})
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

	initial_acc := [3]f32{0, -9.81, 0}

	spheres: [dynamic]PhysicsBody
	defer delete(spheres)

	for i := -12; i <= 12; i += 2 {
		append(&spheres, PhysicsBody{[3]f32{f32(i), 8, 0}, [3]f32{0, 0, 0}, initial_acc, 1.0})
	}
	for i := -12; i <= 12; i += 2 {
		append(&spheres, PhysicsBody{[3]f32{f32(i), 6, 0}, [3]f32{0, 0, 0}, initial_acc, 1.0})
	}

	current_time := f32(glfw.GetTime())
	last_time := current_time
	for !window_should_close(window) {
		window_poll_events()

		current_time = f32(glfw.GetTime())
		delta_time := current_time - last_time
		last_time = current_time

		// Apply force if mouse is pressed
		if glfw.GetMouseButton(window.handle, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS {
			mouse_x, mouse_y := glfw.GetCursorPos(window.handle)
			mouse_x = (mouse_x / f64(WINDOW_WIDTH)) * 2.0 - 1.0
			mouse_y = 1.0 - (mouse_y / f64(WINDOW_HEIGHT)) * 2.0

			for i in 0 ..< len(spheres) {
				sphere := &spheres[i]

				force := linalg.Vector3f32{f32(mouse_x), f32(mouse_y), 0}

				apply_force(force, 1.0, &sphere.acc)
			}
		} else if glfw.GetMouseButton(window.handle, glfw.MOUSE_BUTTON_RIGHT) == glfw.RELEASE {
			for i in 0 ..< len(spheres) {
				spheres[i].acc = initial_acc
			}
		}

		// Add random force to spheres if space is pressed
		if glfw.GetKey(window.handle, glfw.KEY_SPACE) == glfw.PRESS {
			for i in 0 ..< len(spheres) {
				sphere := &spheres[i]

				force := linalg.Vector3f32 {
					rand.float32_range(-10.0, 10.0),
					rand.float32_range(-10.0, 10.0),
					0,
				}
				force *= 50.0

				apply_force(force, 1.0, &sphere.acc)
			}
		}

		// Simulate physics for each sphere
		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]
			velocity_verlet(&sphere.pos, &sphere.vel, sphere.acc, delta_time)
		}

		// Sphere collision detection
		for i in 0 ..< len(spheres) {
			self := &spheres[i]
			for j in 0 ..< len(spheres) {
				other := &spheres[j]
				if i == j {
					continue
				}

				resolve_sphere_collision(self, other, SPHERE_RADIUS)
			}
		}

		// Do not let it go out of bounds
		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]

			if sphere.pos.y <= -8 {
				sphere.pos.y = -8
				sphere.vel.y = -sphere.vel.y * 0.8 // bounce
			}
			if sphere.pos.y >= 8 {
				sphere.pos.y = 8
				sphere.vel.y = -sphere.vel.y * 0.8 // bounce
			}
			if sphere.pos.x <= -14 {
				sphere.pos.x = -14
				sphere.vel.x = -sphere.vel.x * 0.8 // bounce
			}
			if sphere.pos.x >= 14 {
				sphere.pos.x = 14
				sphere.vel.x = -sphere.vel.x * 0.8 // bounce
			}
		}

		gl.ClearColor(0.1, 0.1, 0.1, 1.0)
		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		// Draw sphere
		gl.UseProgram(sphere_primitive.shader.id)

		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]
			model := linalg.matrix4_translate(sphere.pos)

			gl.UniformMatrix4fv(
				gl.GetUniformLocation(sphere_primitive.shader.id, "model"),
				1,
				false,
				&model[0][0],
			)

			gl.BindVertexArray(sphere_primitive.vao)
			gl.DrawElements(
				gl.TRIANGLES,
				cast(i32)len(sphere_primitive.indices),
				gl.UNSIGNED_INT,
				nil,
			)
			gl.BindVertexArray(0)
		}

		window_swap_buffers(window)
	}
}
