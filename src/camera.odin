package main

import "core:math"
import "core:math/linalg"

Camera :: struct {
	pos:  [3]f32,
	fov:  f32,
	near: f32,
	far:  f32,
}

camera_get_view_proj :: proc(camera: Camera, aspect_ratio: f32) -> matrix[4, 4]f32 {
	view := linalg.matrix4_look_at(camera.pos, [3]f32{0, 0, 0}, [3]f32{0, 1, 0})
	projection := linalg.matrix4_perspective(
		math.to_radians(camera.fov),
		aspect_ratio,
		camera.near,
		camera.far,
	)

	return projection * view
}
