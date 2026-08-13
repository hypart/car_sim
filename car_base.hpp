#include "deps.hpp"
#include "vec.hpp"

struct wireMesh{

std::vector<Vec3D> vertices;
std::vector<std::tuple<uint, uint>> connectivity;

wireMesh(std::vector<Vec3D> _vertices, std::vector<std::tuple<uint, uint>> _connectivity){
    vertices = _vertices;
    connectivity = _connectivity;
}

};

float update_rk4(float dt, float x0, float t0, const std::function<float(float, float)>& f){
    float k1 = f(t0, x0);
    float k2 = f(t0 + dt/2, x0 + k1*dt/2);
    float k3 = f(t0 + dt/2, x0 + k2*dt/2);
    float k4 = f(t0 + dt, x0 + k3*dt);

    return x0 + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
}

struct carState{
    float vl; //longitudonal velocity component of chassis CoM [m/s]
    float vt; //transverse velocity component of chassis CoM [m/s]
    float tc; //rotation of chassis with respect to global x axis [rad]
    float tr; //rotation of right tire with respect to longitudonal chassis axis [rad]
    float tl; //rotation of left tire with respect to longitudonal chassis axis [rad]

    float a; //longitudonal acceleration [m/s^2]

    float w; //wheelbase width
    float l; //wheelbase length
    float m; //car mass
    float d; //drag coefficient

    // TODO: have it input the median turning angle of the front wheels instead of the right one
    carState(float _a, float _vl, float _vt, float _tc, float _tr, float _w, float _l, float _m, float _d){
        vl = _vl;
        vt = _vt;
        tc = _tc;
        tr = _tr;
        if (std::abs(tr) < 1e-6) tl = tr;
        else tl = std::atan(w/l + std::tan(PI/2 - tr));

        a = _a;

        w = _w;
        l = _l;
        m = _m;
        d = _d;
    }

    std::tuple<float, float, float> derivatives(){
        if (abs(tr) < 1e-6) return {a/m, 0.0f, 0.0f};
        else{
            float tw;
            if (tr > 0) tw = tr;
            else tw = tl;

            float r = std::sqrt(std::pow(l*std::tan(PI/2 - tr) + w/2, 2) + l*l/4);

            float dvl = a/m - vl*d;
            float dvt = (vl*vl + vt*vt) / r;
            float dtc = std::sqrt(vl*vl + vt*vt) / r;

            return {dvl, dvt, dtc};
        }
    }
};

class carBase{

    wireMesh bodymesh;

    Vec3D com_pos; //position of the center of mass [m]

    float vl; //longitudonal velocity component of chassis CoM [m/s]
    float vt; //transverse velocity component of chassis CoM [m/s]
    float tc; //rotation of chassis with respect to global x axis [rad]
    float tr; //rotation of right tire with respect to longitudonal chassis axis [rad]
    float tl; //rotation of left tire with respect to longitudonal chassis axis [rad]

    float w; //wheelbase width
    float l; //wheelbase length
    float m; //car mass
    float d; //drag coefficient

    float throttle; //throttle command
    float steer; //steering command

    // TODO add power curve
    float accel(){ //longitudonal acceleration [m/s^2]
        return throttle;
    }

public:

    float update_control(float throttle_command, float steer_command){
        
    }

};