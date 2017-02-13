in vec4 f_color;
in vec3 f_normal;

out vec4 fragColor;

void main(void) {
  fragColor = f_color;
	//gl_FragColor = vec4(f_normal, 1);
	/*gl_FragColor[0] = 0.0;
	gl_FragColor[1] = 0.0;
	gl_FragColor[2] = 1.0;*/
}