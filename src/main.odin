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

// Constants for gas simulation
PARTICLE_COUNT :: 1000 // Number of gas particles
INITIAL_TEMPERATURE: f32 : 300.0 // Kelvin
PARTICLE_MASS: f32 : 15.999 // Molar mass of Oxygen
BOLTZMANN_CONSTANT: f32 : 1.380649e-23

// Replace your sphere initialization with this
init_gas_particles :: proc(spheres: ^[dynamic]PhysicsBody, temperature: f32, bounds: Bounds) {
	clear(spheres)

	avg_speed: f32 = get_avg_speed(temperature)

	// Calculate average velocity based on temperature
	for _ in 0 ..< PARTICLE_COUNT {
		pos := [3]f32 {
			rand.float32_range(bounds.left, bounds.right),
			rand.float32_range(bounds.bottom, bounds.top),
			0,
		}

		// Random direction with magnitude based on temperature
		angle := rand.float32_range(0, 2 * math.PI)
		speed := rand.float32_range(0.5 * avg_speed, 1.5 * avg_speed) // Some variation
		vel := [3]f32{math.cos(angle) * speed, math.sin(angle) * speed, 0}

		append(
			spheres,
			PhysicsBody {
				pos  = pos,
				vel  = vel,
				acc  = [3]f32{0, 0, 0}, // No gravity in ideal gas
				mass = PARTICLE_MASS,
			},
		)
	}
}

get_avg_speed :: proc(temp: f32) -> f32 {
	return math.sqrt(3 * BOLTZMANN_CONSTANT * temp / PARTICLE_MASS)
}

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

	// Octree for colision optimization
	ot := new_octree_from_world_bounds(window.handle, camera)
	defer octree_clear(ot)

	wall_collisions: int
	last_pressure_update := f32(glfw.GetTime())
	pressure: f32
	temperature := INITIAL_TEMPERATURE

	bounds := calculate_world_bounds(camera, f32(WINDOW_WIDTH) / f32(WINDOW_HEIGHT))
	init_gas_particles(&spheres, temperature, bounds)

	current_time := f32(glfw.GetTime())
	last_time := current_time
	for !window_should_close(window) {
		window_poll_events()

		current_time = f32(glfw.GetTime())
		delta_time := current_time - last_time
		last_time = current_time

		// temperature controls
		if glfw.GetKey(window.handle, glfw.KEY_UP) == glfw.PRESS {
			temperature += 10.0
			init_gas_particles(&spheres, temperature, bounds)
			fmt.printfln("temperature: %.2f", temperature)
		}
		if glfw.GetKey(window.handle, glfw.KEY_DOWN) == glfw.PRESS {
			temperature = max(10.0, temperature - 10.0)
			init_gas_particles(&spheres, temperature, bounds)
			fmt.printfln("temperature: %.2f", temperature)
		}

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

		// Perfectly elastic particle collisions
		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]
			nearby_indices: [dynamic]int
			defer delete(nearby_indices)

			octree_query(ot, spheres[:], sphere.pos, QUERY_RADIUS, &nearby_indices)

			for j in nearby_indices {
				if i == j do continue
				resolve_elastic_collision(sphere, &spheres[j])
			}
		}

		margin: f32 = SPHERE_RADIUS * 1.1 // Slightly larger than sphere radius

		// Wall collisions (track for pressure calculation)
		for i in 0 ..< len(spheres) {
			sphere := &spheres[i]

			if sphere.pos.y < bounds.bottom + margin {
				sphere.pos.y = bounds.bottom + margin
				sphere.vel.y = -sphere.vel.y // Perfectly elastic
				wall_collisions += 1
			}
			if sphere.pos.y > bounds.top - margin {
				sphere.pos.y = bounds.top - margin
				sphere.vel.y = -sphere.vel.y
				wall_collisions += 1
			}
			if sphere.pos.x < bounds.left + margin {
				sphere.pos.x = bounds.left + margin
				sphere.vel.x = -sphere.vel.x
				wall_collisions += 1
			}
			if sphere.pos.x > bounds.right - margin {
				sphere.pos.x = bounds.right - margin
				sphere.vel.x = -sphere.vel.x
				wall_collisions += 1
			}
		}

		// Calculate pressure periodically
		if current_time - last_pressure_update > 0.5 {
			// Area of container walls (approximate)
			width := bounds.right - bounds.left
			height := bounds.top - bounds.bottom
			area := 2 * (width + height) // Perimeter in 2D

			// Time since last update
			delta_t := current_time - last_pressure_update

			// Pressure = (impulse per time) / area
			// Each collision contributes 2*m*v (perfectly elastic)
			// We approximate by using average velocity
			avg_velocity := get_avg_speed(temperature)
			pressure =
				f32(wall_collisions) * 2 * PARTICLE_MASS * avg_velocity / (f32(delta_t) * area)

			// Update window title with pressure info
			builder := strings.builder_make()
			defer strings.builder_destroy(&builder)

			title := fmt.sbprintf(
				&builder,
				"%s (FPS: %.0f) | Particles: %d | Temperature: %.2fK | Pressure: %.2e Pa",
				WINDOW_TITLE,
				1 / delta_time,
				PARTICLE_COUNT,
				temperature,
				pressure,
			)
			title_cstr, err := strings.to_cstring(&builder)
			if err == nil {
				glfw.SetWindowTitle(window.handle, title_cstr)
			}

			// Reset counters
			wall_collisions = 0
			last_pressure_update = current_time
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
			// octree_render_debug(ot, debug_shader, &view_proj)
		}

		gl.BindVertexArray(0)

		window_swap_buffers(window)
	}
}

new_octree_from_world_bounds :: proc(window: glfw.WindowHandle, camera: Camera) -> ^Octree {
	// Sphere collision detection
	bounds := calculate_world_bounds(camera, f32(WINDOW_WIDTH) / f32(WINDOW_HEIGHT))
	ot_bounds := struct {
		min, max: [3]f32,
	} {
		min = {bounds.left, bounds.bottom, -1.0}, // Z bounds can be small since we're in 2D
		max = {bounds.right, bounds.top, 1.0},
	}
	return octree_new(ot_bounds, max_objects = 4, max_levels = 5)

}

Bounds :: struct {
	left, right, bottom, top: f32,
}

calculate_world_bounds :: proc(camera: Camera, aspect_ratio: f32) -> Bounds {
	// Calculate the visible height at the camera's position
	half_height := math.tan(camera.fov * math.PI / 360.0) * math.abs(camera.pos.z)
	half_width := half_height * aspect_ratio

	left := camera.pos.x - half_width
	right := camera.pos.x + half_width
	bottom := camera.pos.y - half_height
	top := camera.pos.y + half_height

	return Bounds{left, right, bottom, top}
}
