#include <iostream>
#include <string>
#include "window.h"
#include <stdexcept>
#include <iostream>

int main()
{
    const size_t screenWidth = 1920;
    const size_t screenHeight = 1080;

    try
    {
        Window win(screenWidth, screenHeight, "Boids Simulation");
        win.run();
    }
    catch (std::runtime_error& err)
    {
        std::cerr << "ERROR: " << err.what() << std::endl;
        return -1;
    }
    return 0;
}