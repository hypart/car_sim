#include "deps.hpp"

double FPS = 10000;


int main(int argc, const char* argv[]){

    int h = 432;
    int w = 768;

    InitWindow(w, h, "car");
    SetTargetFPS(FPS);

    while(!WindowShouldClose()){
        BeginDrawing();
            ClearBackground(BLACK);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}