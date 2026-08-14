#pragma once
#include "deps.hpp"

class Environment{
public:
    Camera3D camera;
private:

    float floor_z;
    float ceil_z;

    float floor_density;
    float ceil_density;

    float render_dist;

    Eigen::Vector3f update_camera(Eigen::Vector3f car_pos, Eigen::Vector3f car_dir, float car_dist, float cam_tilt);

public:
    Environment(float _floor_z, float _ceil_z, float _floor_density, float _ceil_density, float _render_dist);

    void draw(Eigen::Vector3f car_pos, Eigen::Vector3f car_dir, float car_dist, float cam_tilt);

};