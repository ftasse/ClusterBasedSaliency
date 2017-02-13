#include "font_renderer.h"

// Ref:  http://en.wikibooks.org/wiki/OpenGL_Programming

#include <stdlib.h>
#include <math.h>
#ifdef NOGLEW
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#else
#include <GL/glew.h>
#endif
/* Using GLM for our transformation matrix */
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

#include "glew_utils.h"

struct point {
	GLfloat x;
	GLfloat y;
	GLfloat s;
	GLfloat t;
};

FontRenderer::FontRenderer(const char* fontfilename) : 
    ft_(0), face_(0), is_valid_(false) {
  fprintf(stderr, "Load font \n");
  // glewExperimental = GL_TRUE;
  GLenum glew_status = glewInit();
	if (glew_status != GLEW_OK) {
		fprintf(stderr, "GLEW Init Error: %s\n", glewGetErrorString(glew_status));
		return;
	}
	
  if (FT_Init_FreeType(&ft_)) {
		fprintf(stderr, "Could not init freetype library\n");
		return;
	}

	/* Load a font */
	if (FT_New_Face(ft_, fontfilename, 0, &face_)) {
		fprintf(stderr, "Could not open font %s\n", fontfilename);
		return;
	}

	program_ = create_shader_program("shaders/text.v.glsl", "shaders/text.f.glsl");
	if(program_ == 0)
		return;

	attribute_coord_ = bindAttributeVariable(program_, "coord");
	uniform_tex_ = bindUniformVariable(program_, "tex");
	uniform_color_ = bindAttributeVariable(program_, "color");

	if(attribute_coord_ == -1 || uniform_tex_ == -1 || uniform_color_ == -1)
		return;

	// Create the vertex buffer object
	glGenBuffers(1, &vbo_);
	
	setPixelSize(20);
	setColor(0, 0, 0, 1);
}

FontRenderer::~FontRenderer() {
  if (face_) FT_Done_Face(face_);
  if (ft_)  FT_Done_FreeType(ft_);
  if (is_valid_)
    glDeleteProgram(program_);
}

void FontRenderer::renderText(const char* text, float x, float y, float sx, float sy) const {
  if (!is_valid_) {
    printf("FontRenderer initialization failed. Unable to render: %s\n", text);
    return;
  }
  GLboolean texture2d_enabled;
  GLboolean blending_enabled;
  glGetBooleanv(GL_BLEND, &texture2d_enabled);
  glGetBooleanv(GL_BLEND, &blending_enabled);
  
  if (!blending_enabled) {
    /* Enable blending, necessary for an alpha texture */
	  glEnable(GL_BLEND);
	  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	if (!texture2d_enabled) glEnable(GL_TEXTURE_2D);
	
  const char *p;
	FT_GlyphSlot g = face_->glyph;

	/* Create a texture that will be used to hold one "glyph" */
	GLuint tex;

	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glUniform1i(uniform_tex_, 0);

	/* We require 1 byte alignment when uploading texture data */
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	/* Clamping to edges is important to prevent artifacts when scaling */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	/* Linear filtering usually looks best for text */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	/* Set up the VBO for our vertex data */
	glEnableVertexAttribArray(attribute_coord_);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_);
	glVertexAttribPointer(attribute_coord_, 4, GL_FLOAT, GL_FALSE, 0, 0);

	/* Loop through all characters */
	for (p = text; *p; p++) {
		/* Try to load and render the character */
		if (FT_Load_Char(face_, *p, FT_LOAD_RENDER))
			continue;

		/* Upload the "bitmap", which contains an 8-bit grayscale image, as an alpha texture */
		glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, g->bitmap.width, g->bitmap.rows, 0, GL_ALPHA, GL_UNSIGNED_BYTE, g->bitmap.buffer);

		/* Calculate the vertex and texture coordinates */
		float x2 = x + g->bitmap_left * sx;
		float y2 = -y - g->bitmap_top * sy;
		float w = g->bitmap.width * sx;
		float h = g->bitmap.rows * sy;

		point box[4] = {
			{x2, -y2, 0, 0},
			{x2 + w, -y2, 1, 0},
			{x2, -y2 - h, 0, 1},
			{x2 + w, -y2 - h, 1, 1},
		};

		/* Draw the character on the screen */
		glBufferData(GL_ARRAY_BUFFER, sizeof box, box, GL_DYNAMIC_DRAW);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		/* Advance the cursor to the start of the next character */
		x += (g->advance.x >> 6) * sx;
		y += (g->advance.y >> 6) * sy;
	}

	glDisableVertexAttribArray(attribute_coord_);
	glDeleteTextures(1, &tex);
	
	if (!blending_enabled) glDisable(GL_BLEND);
	if (!texture2d_enabled) glDisable(GL_TEXTURE_2D);
}

void FontRenderer::setPixelSize(int pixel_size) {
  if (!is_valid_) return;
  pixel_size_ = pixel_size;
  FT_Set_Pixel_Sizes(face_, 0, pixel_size_);
}
  
void FontRenderer::setColor(float r = 0, float g = 0, float b = 0, float a = 1) {
  if (!is_valid_) return;
  color_[0] = r;
  color_[1] = b;
  color_[2] = g;
  color_[3] = a;
  glUniform4fv(uniform_color_, 1, color_);
}

