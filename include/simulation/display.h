#pragma once

#include <memory>
#include <string>
#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

template <typename... Args>
std::string string_format(const std::string &format, Args... args)
{
    size_t size = snprintf(nullptr, 0, format.c_str(), args...) + 1;
    if (size <= 0)
    {
        throw std::runtime_error("Error during formatting.");
    }
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1);
}

struct Vector2
{
    double x, y;
    Vector2() : x(0.0), y(0.0) {};
    Vector2(double _x, double _y) : x(_x), y(_y) {};
};

std::vector<Vector2> transform_points(const std::vector<Vector2> &points, const Vector2 &position, const double rotation);
std::vector<std::vector<Vector2>> transform_points(const std::vector<std::vector<Vector2>> &dataset, const Vector2 &position, const double rotation);
std::vector<Vector2> offset_points(const std::vector<Vector2> &points, const Vector2 &offset);
std::vector<std::vector<Vector2>> offset_points(const std::vector<std::vector<Vector2>> &dataset, const Vector2 &offset);

class Display
{
public:
    Display();
    ~Display();

    bool create_renderer(std::string title, int screenWidth, int screenHeight);
    void destroy_renderer();

    void show_screen();
    void clear_screen();

    double get_screen_width() const { return m_screen_width; }
    double get_screen_height() const { return m_screen_height; }
    double get_screen_aspect_ratio() const { return get_screen_width() / get_screen_height(); }

    void draw_text_main_font(const std::string text, const Vector2 pos, const double scale = 1, const SDL_Color color = {0, 0, 0}, bool centered = false);

    void set_draw_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha = 0xFF);
    void set_view(double width, double height, double xOffset, double yOffset);
    void set_view(double xOffset, double yOffset);
    void draw_line(const Vector2 &startPos, const Vector2 &endPos);
    void draw_lines(const std::vector<Vector2> &points);
    void draw_lines(const std::vector<std::vector<Vector2>> &points);

private:
    Vector2 transform_point(const Vector2 &point);

    int m_screen_width;
    int m_screen_height;

    double m_view_width;
    double m_view_height;
    double m_view_x_offset;
    double m_view_y_offset;

    SDL_Window *m_window;
    SDL_Renderer *m_renderer;
    TTF_Font *m_main_font;
};