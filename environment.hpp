#pragma once
#include "deps.hpp"

struct tree {
    Vector2 pos;
    float diam;
    float height;
};

class Environment {
  public:
    Camera3D camera;

  private:
    float floor_z;
    float ceil_z;

    float floor_density;
    float ceil_density;

    int num_trees = 100;
    std::vector<tree> trees;

    float render_dist;

    Eigen::Vector3f update_camera(Eigen::Vector3f car_pos, Eigen::Vector3f car_dir, float car_dist,
                                  float cam_tilt);

    Color dist_fade(float x, float y);

    void init_trees();
    void update_trees(Eigen::Vector3f cam_pos);

  public:
    Environment(float _floor_z, float _floor_density, float _render_dist);

    void draw(Eigen::Vector3f car_pos, Eigen::Vector3f car_dir, float car_dist, float cam_tilt);
};
