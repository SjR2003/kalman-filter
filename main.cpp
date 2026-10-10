#include <string>
#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "simulation/simulation.h"
#include "simulation/car.h"
#include "simulation/display.h"
#include "simulation/profiles.h"

const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 770;
const double GRID_SIZE = 500;
const double GRID_SPACEING = 25;

int main(int argc, char *args[])
{
    Display display;
    Simulation sim;

    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        std::cout << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return -1;
    }
    if (TTF_Init() == -1)
    {
        std::cout << "SDL_ttf could not initialize! SDL_ttf Error: " << TTF_GetError() << std::endl;
        return -1;
    }

    if (!display.create_renderer("Kalman sensor fusion simulations", SCREEN_WIDTH, SCREEN_HEIGHT))
    {
        return -1;
    }

    sim.reset(load_simulation_9_parameters());
    bool running = true;
    while (running)
    {
        sim.update();

        display.clear_screen();

        display.set_draw_color(101, 101, 101);
        for (int x = -GRID_SIZE; x <= GRID_SIZE; x += GRID_SPACEING)
        {
            display.draw_line(Vector2(x, -GRID_SIZE), Vector2(x, GRID_SIZE));
        }
        for (int y = -GRID_SIZE; y <= GRID_SIZE; y += GRID_SPACEING)
        {
            display.draw_line(Vector2(-GRID_SIZE, y), Vector2(GRID_SIZE, y));
        }

        sim.render(display);

        display.show_screen();

        SDL_Event event;
        while (SDL_PollEvent(&event) != 0)
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
            else if (event.type == SDL_KEYDOWN)
            {
                switch (event.key.keysym.sym)
                {
                case SDLK_SPACE:
                    sim.toggle_pause_simulation();
                    break;
                case SDLK_ESCAPE:
                    running = false;
                    break;
                case SDLK_KP_PLUS:
                    sim.increase_zoom();
                    break;
                case SDLK_KP_MINUS:
                    sim.decrease_zoom();
                    break;
                case SDLK_RIGHTBRACKET:
                    sim.increase_time_multiplier();
                    break;
                case SDLK_LEFTBRACKET:
                    sim.decrease_time_multiplier();
                    break;
                case SDLK_r:
                    sim.reset();
                    break;
                case SDLK_1:
                    sim.reset(load_simulation_1_parameters());
                    break;
                case SDLK_2:
                    sim.reset(load_simulation_2_parameters());
                    break;
                case SDLK_3:
                    sim.reset(load_simulation_3_parameters());
                    break;
                case SDLK_4:
                    sim.reset(load_simulation_4_parameters());
                    break;
                case SDLK_5:
                    sim.reset(load_simulation_5_parameters());
                    break;
                case SDLK_6:
                    sim.reset(load_simulation_6_parameters());
                    break;
                case SDLK_7:
                    sim.reset(load_simulation_7_parameters());
                    break;
                case SDLK_8:
                    sim.reset(load_simulation_8_parameters());
                    break;
                case SDLK_9:
                    sim.reset(load_simulation_9_parameters());
                    break;
                case SDLK_0:
                    sim.reset(load_simulation_0_parameters());
                    break;
                }
            }
        }
    }
    display.destroy_renderer();

    TTF_Quit();
    SDL_Quit();

    return 0;
}
