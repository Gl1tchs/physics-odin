package main

import "core:fmt"
import "core:mem"
import "core:os"
import "core:strings"

import gl "vendor:OpenGL"

Shader :: struct {
	id: u32,
}

load_shader :: proc(fragment_path: string, vertex_path: string) -> Shader {
	vertex_shader := compile_shader(vertex_path, gl.VERTEX_SHADER)
	fragment_shader := compile_shader(fragment_path, gl.FRAGMENT_SHADER)

	shader_program := gl.CreateProgram()
	gl.AttachShader(shader_program, vertex_shader)
	gl.AttachShader(shader_program, fragment_shader)
	gl.LinkProgram(shader_program)

	gl.DeleteShader(vertex_shader)
	gl.DeleteShader(fragment_shader)

	return Shader{id = shader_program}
}

compile_shader :: proc(path: string, type: u32) -> u32 {
	source, ok := os.read_entire_file(path)
	if !ok {
		return 0
	}
	// TODO! free source

	source_cstr := strings.clone_to_cstring(string(source))
	// TODO! free source_cstr

	shader := gl.CreateShader(type)
	gl.ShaderSource(shader, 1, &source_cstr, nil)
	gl.CompileShader(shader)

	// Check for compilation errors
	var, success: i32
	gl.GetShaderiv(shader, gl.COMPILE_STATUS, &success)
	if success == 0 {
		var, info_log: [512]u8
		gl.GetShaderInfoLog(shader, 512, nil, &info_log[0])

		info_log_str := strings.clone_from_bytes(info_log[:])
		defer free(&info_log_str)

		fmt.printfln("ERROR::SHADER::COMPILATION_FAILED `%s`\n%s", path, info_log_str)
		return 0
	}

	return shader
}
