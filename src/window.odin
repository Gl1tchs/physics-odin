package main

import "core:fmt"

import gl "vendor:OpenGL"
import "vendor:glfw"

Window :: struct {
	handle: glfw.WindowHandle,
}

create_window :: proc(title: string, width: int, height: int) -> Maybe(Window) {
	glfw.WindowHint(glfw.RESIZABLE, 1)
	glfw.WindowHint(glfw.CONTEXT_VERSION_MAJOR, 4)
	glfw.WindowHint(glfw.CONTEXT_VERSION_MINOR, 6)
	glfw.WindowHint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

	handle := glfw.CreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE, nil, nil)
	if handle == nil {
		return nil
	}

	glfw.MakeContextCurrent(handle)

	// Enable VSync
	glfw.SwapInterval(1)

	// Set window callbacks
	glfw.SetKeyCallback(handle, key_callback)
	glfw.SetFramebufferSizeCallback(handle, size_callback)

	// Load OpenGL
	gl.load_up_to(4, 6, glfw.gl_set_proc_address)

	return Window{handle}
}

destroy_window :: proc(window: ^Window) {
	if window.handle == nil {
		return
	}

	glfw.DestroyWindow(window.handle)
	window.handle = nil
}

window_should_close :: proc(window: Window) -> bool {
	return bool(glfw.WindowShouldClose(window.handle))
}

window_poll_events :: proc() {
	glfw.PollEvents()
}

window_swap_buffers :: proc(window: Window) {
	glfw.SwapBuffers(window.handle)
}

key_callback :: proc "c" (window: glfw.WindowHandle, key, scancode, action, mods: i32) {
	// Exit program when escape pressed
	if key == glfw.KEY_ESCAPE {
		glfw.SetWindowShouldClose(window, true)
	}
}

size_callback :: proc "c" (window: glfw.WindowHandle, width, height: i32) {
	// Set the OpenGL viewport size
	gl.Viewport(0, 0, width, height)
}
