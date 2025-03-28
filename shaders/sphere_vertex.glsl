#version 460 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aOffsets;

out vec3 Normal;

uniform mat4 view_proj;

void main() {
  gl_Position = view_proj * vec4(aPos + aOffsets, 1.0);

  Normal = normalize(aNormal);
}
