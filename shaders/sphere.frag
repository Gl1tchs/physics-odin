#version 460 core

in vec3 Normal;

out vec4 FragColor;

void main() {
  vec3 color = Normal + vec3(1.0) * 0.5;

  FragColor = vec4(0.0, 0.35, 1.0, 0.8);
  FragColor *= vec4(color, 1.0);
}
