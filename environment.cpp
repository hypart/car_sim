#include "deps.hpp"
#include "environment.hpp"

namespace{
// grid positions at multiples of d inside [lo, hi], so the lines stay
// anchored to the world instead of sliding along with the camera
std::vector<float> spaced(float lo, float hi, float d) {
    std::vector<float> v;
    if (d <= 0.0f || hi < lo) return v;

    const float first = std::ceil(lo / d) * d;
    if (first > hi) return v;

    const std::size_t n = static_cast<std::size_t>((hi - first) / d) + 1;
    v.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
        v.push_back(first + d * static_cast<float>(i));
    return v;
}
}

Eigen::Vector3f Environment::update_camera(Eigen::Vector3f car_pos, Eigen::Vector3f car_dir, float car_dist, float cam_tilt){

    Eigen::Vector3f tilt_ax = (-car_dir).cross(Eigen::Vector3f(0.0f, 0.0f, 1.0f)).normalized();

    Eigen::Vector3f cam_angle = (Eigen::AngleAxisf(cam_tilt, tilt_ax) * car_dir).normalized();
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

Environment::Environment(float _floor_z, float _ceil_z, float _floor_density, float _ceil_density, float _render_dist){
    camera = { 0 };
    floor_z = _floor_z;
    ceil_z = _ceil_z;
    floor_density = _floor_density;
    ceil_density = _ceil_density;
    render_dist = _render_dist;
}

void Environment::draw(Eigen::Vector3f car_pos, Eigen::Vector3f car_dir, float car_dist, float cam_tilt){
    Eigen::Vector3f cam_pos = update_camera(car_pos, car_dir, car_dist, cam_tilt);

    float min_x = cam_pos.x() - render_dist;
    float max_x = cam_pos.x() + render_dist;

    float min_y = cam_pos.y() - render_dist;
    float max_y = cam_pos.y() + render_dist;

    std::vector<float> linepos_xfloor = spaced(min_x, max_x, 1/floor_density);
    std::vector<float> linepos_yfloor = spaced(min_y, max_y, 1/floor_density);
    std::vector<float> linepos_xceil = spaced(min_x, max_x, 1/ceil_density);
    std::vector<float> linepos_yceil = spaced(min_y, max_y, 1/ceil_density);

    for(float pos: linepos_xfloor){
        Vector3 start = {pos, floor_z, min_y};
        Vector3 end = {pos, floor_z, max_y};
        DrawLine3D(start, end, WHITE);
    }

    for(float pos: linepos_yfloor){
        Vector3 start = {min_x, floor_z, pos};
        Vector3 end = {max_x, floor_z, pos};
        DrawLine3D(start, end, WHITE);
    }

    for(float pos: linepos_xceil){
        Vector3 start = {pos, ceil_z, min_y};
        Vector3 end = {pos, ceil_z, max_y};
        DrawLine3D(start, end, WHITE);
    }

    for(float pos: linepos_yceil){
        Vector3 start = {min_x, ceil_z, pos};
        Vector3 end = {max_x, ceil_z, pos};
        DrawLine3D(start, end, WHITE);
    }
}
