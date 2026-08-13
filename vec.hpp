#include "deps.hpp"

#pragma once

struct Vec3D{

    float x;
    float y;
    float z;

    Vec3D(float _x, float _y, float _z){
        x = _x;
        y = _y;
        z = _z;
    }

    Vec3D(Vector3 v){
        x = v.x;
        y = v.y;
        z = v.z;
    }

    Vec3D(){
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    // operator overloads

    Vec3D operator*(float s) const {
        return Vec3D(x*s, y*s, z*s);
    }

    Vec3D operator/(float s) const {
        return Vec3D(x/s, y/s, z/s);
    }

    Vec3D operator+(Vec3D v) const {
        return Vec3D(x+v.x, y+v.y, z+v.z);
    }

    Vec3D operator-(Vec3D v) const {
        return Vec3D(x-v.x, y-v.y, z-v.z);
    }

    void operator*=(float s){
        x *= s;
        y *= s;
        z *= s;
    }

    void operator/=(float s){
        x /= s;
        y /= s;
        z /= s;
    }

    void operator+=(Vec3D v){
        x += v.x;
        y += v.y;
        z += v.z;
    }

    void operator-=(Vec3D v){
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    // basic functionality
    float len() const {
        return std::sqrtf(x*x + y*y + z*z);
    }

    float dot(const Vec3D& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    Vec3D cross(const Vec3D& v) const {
        return Vec3D(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    }

    void cross_inplace(const Vec3D& v){
        *this = cross(v);
    }

    Vec3D norm() const {
        if (!len()) return Vec3D(0.0, 0.0, 0.0);
        return Vec3D(x, y, z) / len();
    }

    void norm_inplace() {
        *this /= len();
    }

    Vec3D proj(const Vec3D& v) const {
        return v.norm() * dot(v);
    }

    void proj_inplace(const Vec3D& v){
        *this = proj(v);
    }

    Vec3D rotate(const Vec3D& v) const{
        if(v.len() < 1e-8) return *this;
        float angle = v.len();
        float s = std::sin(angle)/angle;
        float c = (1-std::cos(angle))/(angle*angle);
        Vec3D w = v.cross(*this);
        return *this + w*s+ (v.cross(w))*c;
    }

    void rotate_inplace(const Vec3D& v){
        *this = rotate(v);
    }

    Vector3 raylib_vec3(){
        return {x, y, z};
    }
};