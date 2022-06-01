#include <iostream>
#include "main.hpp"
#include "drone.hpp"

int main()
{
    // values from https://www.researchgate.net/file.PostFileLoader.html?id=576d16ed93553b24b5721a9a&assetKey=AS%3A376462596165634%401466767085787
    DroneParameters drone_params = {
        .g   = 9.81,     // m/s2
        .m   = 0.468,    // kg
        .l   = 0.225,    // m
        .k   = 2.980e-6, //
        .b   = 1.140e-7, //
        .IM  = 3.357e-5, // kg m2
        .Ixx = 4.856e-3, // kg m2
        .Iyy = 4.856e-3, // kg m2
        .Izz = 8.801e-3, // kg m2
        .Ax  = 0.25,     // kg/s
        .Ay  = 0.25,     // kg/s
        .Az  = 0.25,     // kg/s
        .dt  = 0.1,      // s
    };

    Drone drone(drone_params);

    for (int i = 0; i < 10; i++) {
        drone.step();
    }

    return 0;
}
