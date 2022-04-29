#ifndef PARAMETERS
#define PARAMETERS

// robot parameters
namespace parameters
{
    // wheel radius [m]
    constexpr float R = 0.074;

    // sum of wheel positions along x and y (L + W) [m]
    constexpr float L_W = 0.36;

    // gear ratio
    constexpr float T = 5.0;

    // encoder resolution (counts per revolution)
    constexpr float N = 41.5;
}

#endif