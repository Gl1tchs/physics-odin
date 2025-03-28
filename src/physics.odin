package main

import "core:math"
import "core:math/linalg"

PhysicsBody :: struct {
	pos:  [3]f32,
	vel:  [3]f32,
	acc:  [3]f32,
	mass: f32,
}

velocity_verlet :: proc(pos: ^[3]f32, vel: ^[3]f32, acc: [3]f32, dt: f32) {
	for i in 0 ..< 3 {
		pos[i] += vel[i] * dt + 0.5 * acc[i] * dt * dt
		vel[i] += acc[i] * dt
	}
}

apply_force :: proc(force: [3]f32, mass: f32, acc: ^[3]f32) {
	for i in 0 ..< 3 {
		acc[i] += force[i] / mass
	}
}

sphere_vs_sphere :: proc(
	sphere1: [3]f32,
	sphere2: [3]f32,
	radius: f32 = 1.0,
) -> (
	bool,
	f32,
	[3]f32,
) {
	delta := sphere1 - sphere2
	distance_squared := linalg.dot(delta, delta)
	min_distance := radius + radius
	min_distance_squared := min_distance * min_distance

	if distance_squared < 1e-6 { 	// Almost same position
		return true, min_distance, {1, 0, 0} // Arbitrary normal (e.g., x-axis)
	}

	if distance_squared >= min_distance_squared {
		return false, 0, {}
	}

	distance := math.sqrt(distance_squared)
	normal := delta / distance // Safe since distance > 0
	penetration := min_distance - distance

	return true, penetration, normal
}

resolve_sphere_collision :: proc(sphere1: ^PhysicsBody, sphere2: ^PhysicsBody, radius: f32 = 1.0) {
	collided, penetration, normal := sphere_vs_sphere(sphere1.pos, sphere2.pos, radius)

	if !collided do return

	// Position correction (with mass consideration)
	total_mass := sphere1.mass + sphere2.mass
	ratio1 := sphere2.mass / total_mass
	ratio2 := sphere1.mass / total_mass

	correction := normal * penetration
	sphere1.pos += correction * ratio1
	sphere2.pos -= correction * ratio2

	// Calculate relative velocity
	relative_velocity := sphere1.vel - sphere2.vel
	velocity_along_normal := linalg.dot(relative_velocity, normal)

	// If objects are moving apart, don't resolve
	if velocity_along_normal > 0 do return

	// Calculate impulse scalar
	e: f32 = 0.8 // Slightly less than 1.0 for more stability
	j := -(1 + e) * velocity_along_normal
	j /= (1 / sphere1.mass + 1 / sphere2.mass)

	// Apply impulse
	impulse := normal * j
	sphere1.vel += impulse / sphere1.mass
	sphere2.vel -= impulse / sphere2.mass

	// Add small separation to prevent sticking
	small_separation := normal * 0.001
	sphere1.pos += small_separation
	sphere2.pos -= small_separation
}
