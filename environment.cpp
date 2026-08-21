#include "environment.hpp"
#include "deps.hpp"

Eigen::Vector3f Environment::update_camera(Eigen::Vector3f car_pos,
                                           Eigen::Vector3f car_dir,
                                           float car_dist, float cam_tilt) {

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

Environment::Environment(float _floor_z, float _floor_density,
                         float _render_dist) {
  camera = {0};
  floor_z = _floor_z;
  floor_density = _floor_density;
  render_dist = _render_dist;
  init_trees();
}

void Environment::draw(Eigen::Vector3f car_pos, Eigen::Vector3f car_dir,
                       float car_dist, float cam_tilt) {
  Eigen::Vector3f cam_pos = update_camera(car_pos, car_dir, car_dist, cam_tilt);
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
