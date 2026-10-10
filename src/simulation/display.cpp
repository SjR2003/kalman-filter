#include <cmath>
#include <string>
#include <iostream>
#include <vector>

#include "simulation/display.h"

Display::Display()
    : m_screen_width(0),
      m_screen_height(0),
      m_window(nullptr),
      m_renderer(nullptr),
      m_main_font(nullptr)
{
}

Display::~Display()
{
    destroy_renderer();
}

bool Display::create_renderer(std::string title, int screenWidth, int screenHeight)
{
    destroy_renderer();

    m_screen_width = screenWidth;
    m_screen_height = screenHeight;

    m_view_width = m_screen_width;
    m_view_height = m_screen_height;
    m_view_x_offset = 0.0;
    m_view_y_offset = 0.0;

    m_window = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, screenWidth, screenHeight, 0);
    if (m_window == nullptr)
    {
        std::cout << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        destroy_renderer();
        return false;
    }

    m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (m_renderer == nullptr)
    {
        std::cout << "Renderer could not be created! SDL Error: " << SDL_GetError() << std::endl;
        destroy_renderer();
        return false;
    }

    m_main_font = TTF_OpenFont("Roboto-Regular.ttf", 18);
    if (m_main_font == nullptr)
    {
        std::cout << "Failed to load font! SDL_ttf Error: " << TTF_GetError() << std::endl;
        destroy_renderer();
        return false;
    }

    return true;
}

void Display::destroy_renderer()
{
    if (m_renderer != nullptr)
    {
        SDL_DestroyRenderer(m_renderer);
        m_renderer = nullptr;
    }

    if (m_window != nullptr)
    {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }

    if (m_main_font != nullptr)
    {
        TTF_CloseFont(m_main_font);
        m_main_font = nullptr;
    }
}

void Display::clear_screen()
{
    if (m_renderer != nullptr)
    {
        SDL_SetRenderDrawColor(m_renderer, 0x00, 0x00, 0x00, 0xFF);
        SDL_RenderClear(m_renderer);
    }
}

void Display::show_screen()
{
    if (m_renderer != nullptr)
    {
        SDL_RenderPresent(m_renderer);
    }
}

void Display::set_view(double width, double height, double xOffset, double yOffset)
{
    m_view_width = std::fabs(width);
    m_view_height = std::fabs(height);
    m_view_x_offset = xOffset - m_view_height / 2.0;
    m_view_y_offset = yOffset - m_view_width / 2.0;
}

void Display::set_view(double xOffset, double yOffset)
{
    m_view_x_offset = xOffset - m_view_height / 2.0;
    m_view_y_offset = yOffset - m_view_width / 2.0;
}

void Display::set_draw_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha) { SDL_SetRenderDrawColor(m_renderer, red, green, blue, alpha); }

void Display::draw_line(const Vector2 &startPos, const Vector2 &endPos)
{
    Vector2 p1 = transform_point(startPos);
    Vector2 p2 = transform_point(endPos);
    SDL_RenderDrawLine(m_renderer, p1.x, p1.y, p2.x, p2.y);
}

void Display::draw_lines(const std::vector<Vector2> &points)
{
    for (unsigned int i = 1; i < points.size(); ++i)
    {
        draw_line(points[i - 1], points[i]);
    }
}
void Display::draw_lines(const std::vector<std::vector<Vector2>> &dataset)
{
    for (const std::vector<Vector2> &points : dataset)
    {
        draw_lines(points);
    }
}

Vector2 Display::transform_point(const Vector2 &point)
{
    double dx = point.x - m_view_x_offset;
    double dy = point.y - m_view_y_offset;
    double y = m_screen_height - (dx / m_view_height) * m_screen_height;
    double x = (dy / m_view_width) * m_screen_width;
    return Vector2(x, y);
}

void Display::draw_text_main_font(const std::string text, const Vector2 pos, const double scale, const SDL_Color color, bool centered)
{
    SDL_Surface *surface = TTF_RenderText_Blended(m_main_font, text.c_str(), color);
    if (surface == nullptr)
    {
        std::cout << "Unable to render text surface! SDL_ttf Error: " << TTF_GetError() << std::endl;
        return;
    }

    SDL_Texture *texture = SDL_CreateTextureFromSurface(m_renderer, surface);
    if (texture == nullptr)
    {
        SDL_FreeSurface(surface);
        std::cout << "Unable to create texture from rendered text! SDL Error: " << SDL_GetError() << std::endl;
        return;
    }

    int width = surface->w * scale;
    int height = surface->h * scale;
    int x = pos.x;
    int y = pos.y;

    if (centered)
    {
        x -= width / 2;
        y -= height / 2;
    }

    SDL_Rect renderQuad = {x, y, width, height};
    SDL_RenderCopy(m_renderer, texture, nullptr, &renderQuad);

    SDL_DestroyTexture(texture);
    SDL_FreeSurface(surface);
}

std::vector<Vector2> transform_points(const std::vector<Vector2> &points, const Vector2 &position, const double rotation)
{
    double ctheta = std::cos(rotation);
    double stheta = std::sin(rotation);
    std::vector<Vector2> transformed_points;
    for (const Vector2 &point : points)
    {
        double x = point.x * ctheta - stheta * point.y + position.x;
        double y = point.x * stheta + ctheta * point.y + position.y;
        transformed_points.push_back(Vector2(x, y));
    }
    return transformed_points;
}
std::vector<std::vector<Vector2>> transform_points(const std::vector<std::vector<Vector2>> &dataset, const Vector2 &position, const double rotation)
{
    std::vector<std::vector<Vector2>> transformed_dataset;
    for (const std::vector<Vector2> points : dataset)
    {
        transformed_dataset.push_back(transform_points(points, position, rotation));
    }
    return transformed_dataset;
}

std::vector<Vector2> offset_points(const std::vector<Vector2> &points, const Vector2 &offset)
{
    std::vector<Vector2> transformed_points;
    for (const Vector2 &point : points)
    {
        transformed_points.push_back(Vector2(point.x + offset.x, point.y + offset.y));
    }
    return transformed_points;
}

std::vector<std::vector<Vector2>> offset_points(const std::vector<std::vector<Vector2>> &dataset, const Vector2 &offset)
{
    std::vector<std::vector<Vector2>> transformed_dataset;
    for (const std::vector<Vector2> points : dataset)
    {
        transformed_dataset.push_back(offset_points(points, offset));
    }
    return transformed_dataset;
}