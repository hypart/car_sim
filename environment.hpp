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
                                float cam_tilt) {
    Eigen::Vector3f tilt_ax =
        (-car_dir).cross(Eigen::Vector3f(0.0f, 0.0f, 1.0f)).normalized();

    Eigen::Vector3f cam_angle =
        (Eigen::AngleAxisf(cam_tilt, tilt_ax) * car_dir).normalized();
    Eigen::Vector3f cam_pos = car_pos - car_dist * cam_angle;

    // raylib has y pointing up
    Eigen::Vector3f look_at = cam_pos + cam_angle;
    camera.position = (Vector3){cam_pos.x(), cam_pos.z(), cam_pos.y()};
    camera.target = (Vector3){look_at.x(), look_at.z(), look_at.y()};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    return cam_pos;
  }

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
  Environment(float _floor_z, float _floor_density, float _render_dist) {
    camera = {0};
    floor_z = _floor_z;
    floor_density = _floor_density;
    render_dist = _render_dist;
    init_trees();
  }

  void draw(Eigen::Vector3f car_pos, Eigen::Vector3f car_dir, float car_dist,
            float cam_tilt) {
    Eigen::Vector3f cam_pos =
        update_camera(car_pos, car_dir, car_dist, cam_tilt);
    update_trees(cam_pos);

    float min_x = std::floor(cam_pos.x() - render_dist);
    float max_x = std::floor(cam_pos.x() + render_dist);

    float min_y = std::floor(cam_pos.y() - render_dist);
    float max_y = std::floor(cam_pos.y() + render_dist);

    float ds = 1 / floor_density;
    for (float x = min_x; x < (max_x - ds); x += ds) {
      for (float y = min_y; y < (max_y - ds); y += ds) {
        Vector3 start = {x, floor_z, y};
        Vector3 endx = {x + ds, floor_z, y};
        Vector3 endy = {x, floor_z, y + ds};
        Color col =
            dist_fade(x + 0.5f * ds - cam_pos.x(), y + 0.5f * ds - cam_pos.y());
        DrawLine3D(start, endx, col);
        DrawLine3D(start, endy, col);
      }
    }

    for (tree tree : trees) {
      Color col = dist_fade(tree.pos.x - cam_pos.x(), tree.pos.y - cam_pos.y());
      Vector3 start = {tree.pos.x, floor_z, tree.pos.y};
      Vector3 end = {tree.pos.x, floor_z + tree.height, tree.pos.y};
      DrawCylinderWiresEx(start, end, tree.diam / 2.0f, tree.diam / 2.0f, 10,
                          col);
    }
  }
};
