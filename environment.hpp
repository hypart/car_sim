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

  Eigen::Vector3f update_camera(Eigen::Vector3f car_pos,
                                Eigen::Vector3f car_dir, float car_dist,
                                float cam_tilt);

  Color dist_fade(float x, float y) {
    float dist = std::hypot(x, y);
    float a = 1.0f - dist / render_dist;

    unsigned char r = std::clamp(255 * std::pow(a, 0.6), 0.0, 255.0);
    unsigned char g = std::clamp(255 * std::pow(a, 1.0), 0.0, 255.0);
    unsigned char b = std::clamp(255 * std::pow(a, 0.6), 0.0, 255.0);

    return Color({r, g, b, 255});
  }

  float random() {
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    float noise = dist(rng);
    return noise;
  }
  void init_trees() {
    for (int i = 0; i < num_trees; i++) {
      float diam = 0.1 + 0.9 * std::abs(random());
      float height = 1.0 + 4.0 * std::abs(random());
      float xpos = render_dist * random();
      float ypos = render_dist * random();
      trees.push_back(tree({xpos, ypos, diam, height}));
    }
  }
  void update_trees(Eigen::Vector3f cam_pos) {
    for (tree &tree : trees) {
      float x = tree.pos.x - cam_pos.x();
      float y = tree.pos.y - cam_pos.y();
      if ((std::abs(x) > render_dist))
        tree.pos.x -= 2.0f * render_dist * x / std::abs(x);
      if ((std::abs(y) > render_dist))
        tree.pos.y -= 2.0f * render_dist * y / std::abs(y);
    }
  }

public:
  Environment(float _floor_z, float _floor_density, float _render_dist);

  void draw(Eigen::Vector3f car_pos, Eigen::Vector3f car_dir, float car_dist,
            float cam_tilt);
};
