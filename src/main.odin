package main

import "core:fmt"

import "core:math"
import "core:math/linalg"
import "core:math/rand"
import "core:strings"
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

	SPHERE_RADIUS :: 0.25
	COLLISION_MARGIN :: SPHERE_RADIUS * 1.05 // Slightly larger for safety
	BOUNDARY_MARGIN :: SPHERE_RADIUS

	sphere_primitive := sphere_primitive_create(SPHERE_RADIUS, 36, 18)
	defer sphere_primitive_destroy(&sphere_primitive)

	when ODIN_DEBUG {
		debug_shader := load_shader("shaders/cube.frag", "shaders/cube.vert")
	}

	camera := Camera {
		pos  = [3]f32{0, 0, 30},
		fov  = 45.0,
		near = 0.1,
		far  = 100.0,
	}

	view_proj := camera_get_view_proj(camera, f32(WINDOW_WIDTH) / f32(WINDOW_HEIGHT))

	// Set uniforms
	gl.UseProgram(sphere_primitive.shader.id)
	gl.UniformMatrix4fv(
		gl.GetUniformLocation(sphere_primitive.shader.id, "view_proj"),
		1,
		false,
		&view_proj[0][0],
	)

	spheres: [dynamic]PhysicsBody
	defer delete(spheres)

	left, right, bottom, top := calculate_world_bounds(
		camera,
		f32(WINDOW_WIDTH) / f32(WINDOW_HEIGHT),
	)

	x_start := left + BOUNDARY_MARGIN
	x_end := right - BOUNDARY_MARGIN
	y_start := top - BOUNDARY_MARGIN
	y_bottom := bottom + BOUNDARY_MARGIN * 3.0 // Leave more space at bottom

	for i: f32 = x_start * 0.5; i <= x_end * 0.5; i += 0.5 {
		for j: f32 = y_start; j >= y_bottom * 0.2; j -= 0.5 {
			append(
				&spheres,
				PhysicsBody{[3]f32{i, j, 0}, [3]f32{0, 0, 0}, [3]f32{0, -9.81, 0}, 1.0},
			)
		}
	}

	// Octree for colision optimization
	ot := new_octree_from_world_bounds(window.handle, camera)
	defer octree_clear(ot)

	current_time := f32(glfw.GetTime())
	last_time := current_time
	for !window_should_close(window) {
		window_poll_events()

		current_time = f32(glfw.GetTime())
		delta_time := current_time - last_time
		last_time = current_time

		builder := strings.builder_make()
		defer strings.builder_destroy(&builder)

		title := fmt.sbprintf(&builder, "%s (FPS: %.2f)", WINDOW_TITLE, 1 / delta_time)

		title_cstr, err := strings.to_cstring(&builder)
		if err == nil {
			glfw.SetWindowTitle(window.handle, title_cstr)
		}

		handle_forces(window.handle, &spheres)

		// Simulate physics for each sphere
		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]
			velocity_verlet(&sphere.pos, &sphere.vel, sphere.acc, delta_time)
		}

		// Sphere collision detection
		octree_clear(ot)
		for sphere, i in spheres {
			octree_insert(ot, spheres[:], i)
		}

		QUERY_RADIUS :: SPHERE_RADIUS * 1.5

		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]
			nearby_indices: [dynamic]int
			defer delete(nearby_indices)

			octree_query(ot, spheres[:], sphere.pos, QUERY_RADIUS, &nearby_indices)

			for j in nearby_indices {
				if i == j do continue // Skip self-collision
				resolve_sphere_collision(sphere, &spheres[j], SPHERE_RADIUS)
			}
		}

		margin: f32 = SPHERE_RADIUS * 1.1 // Slightly larger than sphere radius

		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]

			// Check and resolve each axis independently
			if sphere.pos.y < bottom + margin {
				penetration := (bottom + margin) - sphere.pos.y
				sphere.pos.y += penetration
				sphere.vel.y = -sphere.vel.y * 0.8
			}
			if sphere.pos.y > top - margin {
				penetration := sphere.pos.y - (top - margin)
				sphere.pos.y -= penetration
				sphere.vel.y = -sphere.vel.y * 0.8
			}
			if sphere.pos.x < left + margin {
				penetration := (left + margin) - sphere.pos.x
				sphere.pos.x += penetration
				sphere.vel.x = -sphere.vel.x * 0.8
			}
			if sphere.pos.x > right - margin {
				penetration := sphere.pos.x - (right - margin)
				sphere.pos.x -= penetration
				sphere.vel.x = -sphere.vel.x * 0.8
			}
		}

		gl.ClearColor(0.1, 0.1, 0.1, 1.0)
		gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

		// Draw sphere
		gl.UseProgram(sphere_primitive.shader.id)

		sphere_offsets: [dynamic][3]f32
		defer delete(sphere_offsets)

		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]
			append(&sphere_offsets, sphere.pos)
		}

		gl.BindVertexArray(sphere_primitive.vertex_array)

		gl.BindBuffer(gl.ARRAY_BUFFER, sphere_primitive.instance_buffer)
		gl.BufferSubData(
			gl.ARRAY_BUFFER,
			0,
			size_of([3]f32) * len(sphere_offsets),
			&sphere_offsets[0],
		)
		gl.BindBuffer(gl.ARRAY_BUFFER, 0)

		gl.DrawElementsInstanced(
			gl.TRIANGLES,
			sphere_primitive.index_count,
			gl.UNSIGNED_INT,
			nil,
			i32(len(sphere_offsets)),
		)

		when ODIN_DEBUG {
			octree_render_debug(ot, debug_shader, &view_proj)
		}

		gl.BindVertexArray(0)

		window_swap_buffers(window)
	}
}

new_octree_from_world_bounds :: proc(window: glfw.WindowHandle, camera: Camera) -> ^Octree {
	// Sphere collision detection
	left, right, bottom, top := calculate_world_bounds(
		camera,
		f32(WINDOW_WIDTH) / f32(WINDOW_HEIGHT),
	)
	ot_bounds := struct {
		min, max: [3]f32,
	} {
		min = {left, bottom, -1.0}, // Z bounds can be small since we're in 2D
		max = {right, top, 1.0},
	}
	return octree_new(ot_bounds, max_objects = 4, max_levels = 5)

}

calculate_world_bounds :: proc(
	camera: Camera,
	aspect_ratio: f32,
) -> (
	left, right, bottom, top: f32,
) {
	// Calculate the visible height at the camera's position
	half_height := math.tan(camera.fov * math.PI / 360.0) * math.abs(camera.pos.z)
	half_width := half_height * aspect_ratio

	left = camera.pos.x - half_width
	right = camera.pos.x + half_width
	bottom = camera.pos.y - half_height
	top = camera.pos.y + half_height

	return
}

handle_forces :: proc(window: glfw.WindowHandle, spheres: ^[dynamic]PhysicsBody) {
	// Apply force if mouse is pressed
	if glfw.GetMouseButton(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS {
		mouse_x, mouse_y := glfw.GetCursorPos(window)
		mouse_x = (mouse_x / f64(WINDOW_WIDTH)) * 2.0 - 1.0
		mouse_y = 1.0 - (mouse_y / f64(WINDOW_HEIGHT)) * 2.0

		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]

			force := linalg.Vector3f32{f32(mouse_x), f32(mouse_y), 0}

			apply_force(force, 1.0, &sphere.acc)
		}
	} else if glfw.GetMouseButton(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.RELEASE {
		for i in 0 ..< len(spheres) {
			spheres[i].acc = [3]f32{0, -9.81, 0}
		}
	}

	// Add random force to spheres if space is pressed
	if glfw.GetKey(window, glfw.KEY_SPACE) == glfw.PRESS {
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

}
