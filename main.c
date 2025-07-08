#include "include/raylib.h"
#include "include/raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "include/raygui.h"
#include "style_terminal.h"

#define NUM_OF_BOIDS 100
#define SCREEN_MARGIN 50
#define MAX_SPEED 200
#define MIN_SPEED 100
#define SEPARATION_RANGE 20
#define VISUAL_RANGE 40
const int width = 800; const int height = 450;

static float avoidanceFactor = 1;
static float alignmentFactor = 0.05;
static float centringFactor = 0.1;
static float turnFactor = 25;

typedef struct
{
    int index;
    Vector2 position;
    Vector2 velocity;
} Boid;

Boid boids[NUM_OF_BOIDS];

void BoidSeparation(Boid *boid)
{
    Vector2 separationForce = {0, 0};
    for(int i = 0; i < NUM_OF_BOIDS; i++)
    {
        if(Vector2Distance(boid->position, boids[i].position) < SEPARATION_RANGE)
        {
            separationForce.x += boid->position.x - boids[i].position.x;
            separationForce.y += boid->position.y - boids[i].position.y;
        }
    }

    boid->velocity.x += separationForce.x * avoidanceFactor;
    boid->velocity.y += separationForce.y * avoidanceFactor;
}

void BoidAlignment(Boid *boid)
{
    int neighbouringBoids = 0;
    Vector2 percievedVelocity = {0, 0};
    for(int i = 0; i < NUM_OF_BOIDS; i++)
    {
        // If boid is in the visual range and make sure it isn't our current boid
        if(Vector2Distance(boid->position, boids[i].position) < VISUAL_RANGE && i != boid->index)
        {
            percievedVelocity.x += boids[i].velocity.x;
            percievedVelocity.y += boids[i].velocity.y;
            neighbouringBoids++;
        }
    }

    // If it is zero then there're no boids nearby
    if(neighbouringBoids > 0)
    {
        percievedVelocity.x /= neighbouringBoids;
        percievedVelocity.y /= neighbouringBoids;
    }

    boid->velocity.x += (percievedVelocity.x - boid->velocity.x) * alignmentFactor;
    boid->velocity.y += (percievedVelocity.y - boid->velocity.y) * alignmentFactor;
}

void BoidCohesion(Boid *boid)
{
    int neighbouringBoids = 0;
    Vector2 percievedCentre = {0, 0};
    for(int i = 0; i < NUM_OF_BOIDS; i++)
    {
        // If boid is in the visual range and make sure it isn't our current boid
        if(Vector2Distance(boid->position, boids[i].position) < VISUAL_RANGE && i != boid->index)
        {
            percievedCentre.x += boids[i].position.x;
            percievedCentre.y += boids[i].position.y;
            neighbouringBoids++;
        }
    }

     // If it is zero then there're no boids nearby
    if(neighbouringBoids > 0)
    {
        percievedCentre.x /= neighbouringBoids;
        percievedCentre.y /= neighbouringBoids;
    }

    boid->velocity.x += (percievedCentre.x - boid->position.x) * centringFactor;
    boid->velocity.y += (percievedCentre.y - boid->position.y) * centringFactor;
}

void BoidAvoidEdges(Boid *boid)
{
    if(boid->position.x < SCREEN_MARGIN)
    {
        boid->velocity.x += turnFactor;
    }
    if(boid->position.x > width - SCREEN_MARGIN)
    {
        boid->velocity.x -= turnFactor;
    }
    if(boid->position.y < SCREEN_MARGIN)
    {
        boid->velocity.y += turnFactor;
    }
    if(boid->position.y > height - SCREEN_MARGIN)
    {
        boid->velocity.y -= turnFactor;
    }
}

void BoidLimitSpeed(Boid *boid)
{
    float speed = sqrt(boid->velocity.x * boid->velocity.x + boid->velocity.y*boid->velocity.y);
    if(speed > MAX_SPEED)
    {
        boid->velocity.x = (boid->velocity.x/speed)*MAX_SPEED;
        boid->velocity.y = (boid->velocity.y/speed)*MAX_SPEED;
    }
    if(speed < MIN_SPEED)
    {
        boid->velocity.x = (boid->velocity.x/speed)*MIN_SPEED;
        boid->velocity.y = (boid->velocity.y/speed)*MIN_SPEED;
    }
}

void BoidsUpdate()
{
    for(int i = 0; i < NUM_OF_BOIDS; i++)
    {
        BoidSeparation(&boids[i]);
        BoidAlignment(&boids[i]);
        BoidCohesion(&boids[i]);
        BoidAvoidEdges(&boids[i]);
        BoidLimitSpeed(&boids[i]);

        boids[i].position.x += boids[i].velocity.x * GetFrameTime();
        boids[i].position.y += boids[i].velocity.y * GetFrameTime();
    }
}

void BoidsDraw()
{
    for(int i = 0; i < NUM_OF_BOIDS; i++)
    {
        // Triangle rotation stuff
        const int halfHeight = 10;
        const int halfBase = 10;
        float rotation = atan2(boids[i].velocity.y, boids[i].velocity.x) + 1.57;
        Vector2 p1 = {cos(rotation) * (boids[i].position.x - boids[i].position.x) - sin(rotation) * (boids[i].position.y-halfHeight - boids[i].position.y) + boids[i].position.x,
                      sin(rotation) * (boids[i].position.x - boids[i].position.x) + cos(rotation) * (boids[i].position.y-halfHeight - boids[i].position.y) + boids[i].position.y};
        
        Vector2 p2 = {cos(rotation) * (boids[i].position.x-halfBase - boids[i].position.x) - sin(rotation) * (boids[i].position.y+halfHeight - boids[i].position.y) + boids[i].position.x,
                      sin(rotation) * (boids[i].position.x-halfBase - boids[i].position.x) + cos(rotation) * (boids[i].position.y+halfHeight - boids[i].position.y) + boids[i].position.y};

        Vector2 p3 = {cos(rotation) * (boids[i].position.x+halfBase - boids[i].position.x) - sin(rotation) * (boids[i].position.y+halfHeight - boids[i].position.y) + boids[i].position.x,
                      sin(rotation) * (boids[i].position.x+halfBase - boids[i].position.x) + cos(rotation) * (boids[i].position.y+halfHeight - boids[i].position.y) + boids[i].position.y};

        DrawTriangle(p1, p2, p3, GREEN);
    }
}

void BoidsInitalize()
{
    for(int i = 0; i < NUM_OF_BOIDS; i++)
    {
        boids[i].index = i;
        boids[i].position.x = GetRandomValue(0, width);
        boids[i].position.y = GetRandomValue(0, height);
        boids[i].velocity.x = GetRandomValue(-150, 150);
        boids[i].velocity.y = GetRandomValue(-150, 150);
    }
}

int main()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(width, height, "Boids");
    
    BoidsInitalize();
    GuiLoadStyleTerminal();

    SetTargetFPS(60);
    while(!WindowShouldClose())
    {
        BoidsUpdate();

        BeginDrawing();
        ClearBackground(BLACK);
        BoidsDraw();
        
        GuiSliderBar((Rectangle){100, 380, 105, 20}, "0", "full", &avoidanceFactor, 0, 2);
        GuiSliderBar((Rectangle){350, 380, 105, 20}, "0", "full", &alignmentFactor, 0, 1);
        GuiSliderBar((Rectangle){595, 380, 105, 20}, "0", "full", &centringFactor, 0, 1);
        GuiDrawText("Separation", (Rectangle){100, 380, 105, 20}, 1, DARKGREEN);
        GuiDrawText("Alignment", (Rectangle){350, 380, 105, 20}, 1, DARKGREEN);
        GuiDrawText("Cohesion", (Rectangle){595, 380, 105, 20}, 1, DARKGREEN);

        EndDrawing();
    }
    CloseWindow();
    return 0;
}