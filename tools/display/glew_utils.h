#ifndef GLEW_UTILS_H
#define GLEW_UTILS_H

#include <vector>

unsigned int create_shader(const char *shader_file,  unsigned int shader_type);
unsigned int link_shader_program(const std::vector<unsigned int> &shaders,
	const std::vector<char*>& feedbak_varyings = std::vector<char*>());
int bindAttributeVariable(unsigned int shader_program, const char* attribute_name);
int bindUniformVariable(unsigned int shader_program, const char *uniform_name);

unsigned int create_shader_program(
	const char *vs_file, 
	const std::vector<char*>& feedbak_varyings = std::vector<char*>());

unsigned int create_shader_program(
	const char *vs_file, 
	const char *fs_file,
	const std::vector<char*>& feedbak_varyings = std::vector<char*>());

unsigned int create_shader_program(
	const char *vs_file, 
	const char *fs_file,
	const char *gs_file,
	const std::vector<char*>& feedbak_varyings = std::vector<char*>());

void print_log(unsigned int object);


#endif // GLEW_UTILS_H