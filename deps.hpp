#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <raylib.h>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

constexpr float pi = 3.14159265358979323846f;

inline float rand_uniform() {
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    float noise = dist(rng);
    return noise;
}