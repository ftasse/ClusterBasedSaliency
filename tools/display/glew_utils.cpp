#include "display/glew_utils.h"

#include <stdio.h>
#include <stdlib.h>

#include <GL/glew.h>

char* filetobuf(const char *file);

unsigned int create_shader(const char *shader_file, unsigned int shader_type)
{
 	char *vs_source = filetobuf(shader_file);

 	if (vs_source == NULL) 
 	{
    	fprintf(stderr, "Error opening %s: ", shader_file); perror("");
    	return 0;
 	}

 	// Need the first string to make the code portable for desktop and mobile
 	const GLchar* sources[3] = {
#ifdef GL_ES_VERSION_2_0
    	"#version 100\n"
	    // Note: OpenGL ES automatically defines this:
	    // #define GL_ES
#else
    	"#version 150\n"
#endif
    	,
    	// GLES2 precision specifiers
#ifdef GL_ES_VERSION_2_0
		// Define default float precision for fragment shaders:
		(type == GL_FRAGMENT_SHADER) ?
		"#ifdef GL_FRAGMENT_PRECISION_HIGH\n"
		"precision highp float;           \n"
		"#else                            \n"
		"precision mediump float;         \n"
		"#endif                           \n"
		: ""
		// Note: OpenGL ES automatically defines this:
		// #define GL_ES
#else
	    // Ignore GLES 2 precision specifiers:
	    "#define lowp   \n"
	    "#define mediump\n"
	    "#define highp  \n"
#endif
    ,
    vs_source};

 	GLuint vs = glCreateShader(shader_type);
 	if (vs == 0) {
 		fprintf(stderr, "Error: could not create shader %s of type %d\n", shader_file, shader_type);
 		return 0;
 	}

	glShaderSource(vs, 3, sources, NULL);
	glCompileShader(vs);

	GLint compile_ok = GL_FALSE;
	glGetShaderiv(vs, GL_COMPILE_STATUS, &compile_ok);
	if (0 == compile_ok)
	{
		fprintf(stderr, "Error in shader %d: %s\n", vs, shader_file);
		print_log(vs);
		glDeleteShader(vs);
		return 0;
	} else
		return vs;
}

unsigned int link_shader_program(const std::vector<unsigned int> &shaders,
	const std::vector<char*>& feedback_varyings)
{
	if (shaders.size() == 0)
	{
		fprintf(stderr, "No shaders compiled\n"); perror("");
		return 0;
	}

	unsigned int program = glCreateProgram();
	for (size_t i = 0; i < shaders.size(); ++i)
		if (shaders[i] > 0)
			glAttachShader(program, shaders[i]);

	if (feedback_varyings.size() > 0)
		glTransformFeedbackVaryings(program, feedback_varyings.size(), (const char**) &feedback_varyings[0], GL_INTERLEAVED_ATTRIBS);
	glLinkProgram(program);

	GLint link_ok = GL_FALSE;
	glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
	if (!link_ok) 
	{
		fprintf(stderr, "glLinkProgram: ");
		print_log(program);
		glDeleteProgram(program);
		return 0;
	} 
	else
		return program;
}

unsigned int create_shader_program(const char *vs_file, const std::vector<char*>& feedback_varyings) {
	std::vector<unsigned int> shaders(1, create_shader(vs_file, GL_VERTEX_SHADER));
	return link_shader_program(shaders, feedback_varyings);
}

unsigned int create_shader_program(const char *vs_file, const char *fs_file, const std::vector<char*>& feedback_varyings) {
	std::vector<unsigned int> shaders(2);
	shaders[0] = create_shader(vs_file, GL_VERTEX_SHADER);
	shaders[1] = create_shader(fs_file, GL_FRAGMENT_SHADER);
	return link_shader_program(shaders, feedback_varyings);
}

unsigned int create_shader_program(const char *vs_file, const char *fs_file, const char *gs_file, const std::vector<char*>& feedback_varyings)
{
	std::vector<unsigned int> shaders(3);
	shaders[0] = create_shader(vs_file, GL_VERTEX_SHADER);
	shaders[1] = create_shader(fs_file, GL_FRAGMENT_SHADER);
	shaders[1] = create_shader(gs_file, GL_GEOMETRY_SHADER);
	return link_shader_program(shaders, feedback_varyings);
}

int bindAttributeVariable(unsigned int shader_program, const char* attribute_name)
{
	int attr = glGetAttribLocation(shader_program, attribute_name);
	if (attr == -1) {
	    fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
  	}
  	return attr;
}

int bindUniformVariable(unsigned int shader_program, const char *uniform_name)
{
	int uniform = glGetUniformLocation(shader_program, uniform_name);
	if (uniform == -1) {
		fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
	}
	return uniform;
}

char* filetobuf(const char *file)
{
	FILE* input = fopen(file, "rb");
	if(input == NULL) return NULL;

	if(fseek(input, 0, SEEK_END) == -1) return NULL;
	long size = ftell(input);
	if(size == -1) return NULL;
	if(fseek(input, 0, SEEK_SET) == -1) return NULL;

	/*if using c-compiler: dont cast malloc's return value*/
	char *content = (char*) malloc( (size_t) size +1  ); 
	if(content == NULL) return NULL;

	if(!fread(content, 1, (size_t)size, input) || ferror(input)) {
		free(content);
		return NULL;
	}

	fclose(input);
	content[size] = '\0';
	return content;
}

/**
 * Display compilation errors from the OpenGL shader compiler
 */
void print_log(unsigned int object)
{
	GLint log_length = 0;
	if (glIsShader(object))
		glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
	else if (glIsProgram(object))
		glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
	else {
		fprintf(stderr, "printlog: Not a shader or a program\n");
		return;
	}

	char* log = (char*)malloc(log_length);

	if (glIsShader(object))
		glGetShaderInfoLog(object, log_length, NULL, log);
	else if (glIsProgram(object))
		glGetProgramInfoLog(object, log_length, NULL, log);

	fprintf(stderr, "%s", log);
	free(log);
}
