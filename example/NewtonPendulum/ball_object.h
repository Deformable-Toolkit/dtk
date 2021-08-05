/*******************************************************************
** This code is part of Breakout.
**
** Breakout is free software: you can redistribute it and/or modify
** it under the terms of the CC BY 4.0 license as published by
** Creative Commons, either version 4 of the License, or (at your
** option) any later version.
******************************************************************/
#ifndef BALLOBJECT_H
#define BALLOBJECT_H

#include "glad/glad.h"
#include "glm/glm.hpp"
#include <GLFW/glfw3.h>

#include "game_object.h"
#include "texture.h"

const unsigned int LINE_LEN = 300;
const unsigned int g = 9.8f;


// BallObject holds the state of the Ball object inheriting
// relevant state data from GameObject. Contains some extra
// functionality specific to Breakout's ball object that
// were too specific for within GameObject alone.
class BallObject : public GameObject
{
public:
    // ball state	
    float   Radius;
    bool    Stuck;
    bool    Sticky, PassThrough;

    float Mass = 1.0f;
    glm::vec2 Acce;
    glm::vec2 Force;
    glm::vec2 Impuse;

    float Stiff;
    float angle;
    float angle_speed;

    glm::vec2 FixedPoint;

    GLuint lineVAO,lineVBO;

    bool left;

    // constructor(s)
    BallObject();
    BallObject(glm::vec2 pos, float radius,  Texture2D sprite, glm::vec2 velocity, float mass = 1.0, float stiff = 0.05);
    // moves the ball, keeping it constrained within the window bounds (except bottom edge); returns new position
    glm::vec2 Move(float dt, unsigned int window_width, unsigned int window_height);
    // resets the ball to original state with given position and velocity
    void      Reset(glm::vec2 position, glm::vec2 velocity);

    void Draw(SpriteRenderer &renderer);
};

#endif