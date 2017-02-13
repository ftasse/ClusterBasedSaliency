#ifndef FONT_RENDERER_H
#define FONT_RENDERER_H

struct FT_LibraryRec_;
struct FT_FaceRec_;

class FontRenderer {
public:
  FontRenderer(const char* fontfilename);
  ~FontRenderer();
  void renderText(const char* text, float x, float y, float sx = 1, float sy = 1) const;
  
  void setPixelSize(int pixel_size);
  void setColor(float r, float g, float b, float a);
  
  bool isValid() const {
    return is_valid_;
  }
  
private:
  FT_LibraryRec_* ft_;
  FT_FaceRec_* face_;
  
  unsigned int program_;
  int attribute_coord_;
  int uniform_tex_;
  int uniform_color_;
  unsigned int vbo_;

  int pixel_size_;
  float color_[4];
  bool is_valid_;
};

#endif
