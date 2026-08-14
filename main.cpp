#include "deps.hpp"
#include "car_base.hpp"
#include "environment.hpp"

double dispFPS = 60;
uint phys_per_disp_FPS = 10; //physics frames per displayed frame


int main(int argc, const char* argv[]){

    int h = 500;
    int w = 1000;

    carBase car = carBase(2.0, 1.0, 1.0, 0.01, Eigen::Vector3f(1.0f, 1.0f, 0.0f));
    Environment scene(0.0f, 5.0f, 1.0f, 1.0f, 100.0f);

    float throttle = 0.0;
    float steer = 0.0;
    float brake = 0.0;

    InitWindow(w, h, "car");
    SetTargetFPS(dispFPS);

    uint frame_count = 0;
    float dist_scale = 50.0;

    while(!WindowShouldClose()){

        if(IsKeyDown(KEY_W)) throttle = 0.1;
        if(IsKeyDown(KEY_A)) steer = -1.0;
        if(IsKeyDown(KEY_D)) steer = 1.0;
        if(IsKeyDown(KEY_S)) brake = 0.1;

        for(int i = 0; i < phys_per_disp_FPS; i++){
            car.input_control(throttle, brake, steer);
            car.update_state(1/(dispFPS * phys_per_disp_FPS));
        }

        if(!(frame_count % phys_per_disp_FPS)){
            BeginDrawing();
                ClearBackground(BLACK);
                BeginMode3D(scene.camera);
                    Eigen::Vector3f car_dir = {std::cos(car.tc), std::sin(car.tc), 0.0f};
                    scene.draw(car.com_pos, car_dir, 4.0f, pi/16);
                    car.draw();
                    //DrawSphere((Vector3){car.com_pos.x(), car.com_pos.z(), car.com_pos.y()}, 0.5f, WHITE);
                EndMode3D();
            EndDrawing();
        }

        throttle = 0.0;
        steer = 0.0;
        brake = 0.0;
        
        frame_count++;
    }
    CloseWindow();
    return 0;
}