/******************************************************************
** This code is part of Breakout.
**
** Breakout is free software: you can redistribute it and/or modify
** it under the terms of the CC BY 4.0 license as published by
** Creative Commons, either version 4 of the License, or (at your
** option) any later version.
******************************************************************/
#include "ball_object.h"
#include <iostream>


BallObject::BallObject() 
    : GameObject(), Radius(12.5f), Stuck(true), Sticky(false), PassThrough(false)  {
        //GLuint VBO,VAO;
        glGenVertexArrays(1, &this->lineVAO);
        glGenBuffers(1,&this->lineVBO);
        glBindVertexArray(this->lineVAO);
        glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1,3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }

BallObject::BallObject(glm::vec2 pos, float radius,  Texture2D sprite, glm::vec2 velocity, float mass, float stiff)
    : GameObject(pos, glm::vec2(radius * 2.0f, radius * 2.0f), sprite, glm::vec3(1.0f), velocity)
    , Radius(radius), Stuck(true), Sticky(false), PassThrough(false), FixedPoint(glm::vec2(400, 0))
    ,Mass(mass), Stiff(stiff), Acce(glm::vec2(0,9.8f)),Force(mass * Acce), Impuse(glm::vec2(0.0f)) { 

        angle = 1.0472; //asin((this->Position.x - this->FixedPoint.x )/LINE_LEN/2);
        angle_speed = 0; 

        glGenVertexArrays(1, &this->lineVAO);
        glGenBuffers(1,&this->lineVBO);
        glBindVertexArray(this->lineVAO);

        float vertexs[]={
            //this->Position.x, this->Position.y, 0,
            //this->FixedPoint.x, this->FixedPoint.y,0
            1.0f,0.0f,0.0f,1.0f, 1.0f, 1.0f,
            0.0f,1.0f,0.0f,1.0f, 1.0f, 1.0f,
            //this->Position.x, this->Position.y, 0,  1.0f, 1.0f, 1.0f,   // 右下
            //this->FixedPoint.x, this->FixedPoint.y,0,  1.0f, 1.0f, 1.0f, 
        };

        glBindBuffer(GL_ARRAY_BUFFER,lineVBO);
        //glBufferSubData(GL_ARRAY_BUFFER,0,sizeof(vertexs),vertexs);
        glBufferData(GL_ARRAY_BUFFER,sizeof(vertexs),vertexs,GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1,3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        left = true;
    }

glm::vec2 BallObject::Move(float dt, unsigned int window_width, unsigned int window_height)
{
    // if not stuck to player board
    //if (!this->Stuck)
    //{
        // move the ball

        //std::cout << dt << std::endl;
        //for(int i = 0; i < 100; ++i){

            /*
            float delta = dt;

            float k1 = angle_speed;
            float l1 = -(g/LINE_LEN)*sin(angle);
            float k2 = angle_speed + delta * l1 / 2.0;
            float l2 = -(g/LINE_LEN)*sin(angle  + delta * k1 / 2.0);
            float k3 = angle_speed + delta * l2 / 2.0;
            float l3 = -(g/LINE_LEN) * sin(angle + delta * k2 / 2.0);
            float k4 = angle_speed + delta * l3;
            float l4 = -(g /LINE_LEN) * sin(angle + delta * k3);
            angle = angle + delta *(k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
            angle_speed = angle_speed + delta * (l1 + 2 * l2 + 2 * l3 + l4) / 6.0;
            std::cout << angle << " " << angle_speed << std::endl;    

        //}
            
            this->Position = glm::vec2(LINE_LEN * sin(angle) + this->FixedPoint.x , LINE_LEN * cos(angle));
            */

            glm::vec2 vec = this->Position + glm::vec2(Radius, Radius) - this->FixedPoint;
            this->Force = glm::vec2(0,9.8 * this->Mass) - glm::normalize(vec) * (glm::length(glm::vec2(0,9.8 * this->Mass))*((this->Position.y - this->FixedPoint.y) / LINE_LEN));
            this->Acce = this->Force / this->Mass;
           // this->Velocity =  this->Velocity * (1.0f - this->Stiff);
            this->Velocity += this->Acce * dt;

            //std::cout << this->Velocity.x << "  " << this->Velocity.y << std::endl;
            
            this->Impuse = this->Mass * this->Velocity;
            this->Position += this->Velocity * dt;
            // then check if outside window bounds and if so, reverse velocity and restore at correct position

            //glm::vec4 pos = glm::vec4(this->Position + glm::vec2(Radius,Radius), 0,0);
            //glm::mat4 model = glm::mat4(1.0f);
            //model = glm::translate(model, glm::vec3(this->Position, 0.0f));
            //pos = model * pos;

            
            

            vec = this->Position + glm::vec2(Radius, Radius) - this->FixedPoint;;
            int len = glm::length(vec);
            if(len > LINE_LEN)
            {
                this->Position = vec * (float)LINE_LEN / (float)len + this->FixedPoint - glm::vec2(Radius, Radius);
               // this->Velocity.y = 0;
                if(glm::length(this->Velocity) < 0.2){
                    //this->Velocity = glm::vec2(0.0f,0.0f);
                    //this->Force = glm::vec2(0.0f,0.0f);
                    //this->Acce = glm::vec2(0.0f,0.0f);
                }
            }
        //}

        if(left)
            angle -= dt * 0.05 * this->Position.y / 5;
        else
            angle += dt * 0.05 * this->Position.y / 5;
        this->Position = glm::vec2(LINE_LEN * sin(angle) + this->FixedPoint.x , LINE_LEN * cos(angle)) - glm::vec2(Radius, Radius);

        
        if (this->Position.x <= 0.0f)
        {
            this->Velocity.x = -this->Velocity.x;
            this->Position.x = 0.0f;
        }
        else if (this->Position.x + this->Size.x >= window_width)
        {
            this->Velocity.x = -this->Velocity.x;
            this->Position.x = window_width - this->Size.x;
        }
        else if(this->Position.y + this->Size.y >= window_height ){
            this->Velocity.y = -this->Velocity.y * 0.9;
            this->Position.y = window_height - this->Size.y;
        }
        if (this->Position.y <= 0.0f)
        {   
            this->Velocity.y = -this->Velocity.y ;
            this->Position.y = 0.0f;
        }

        if(this->Position.y <= Radius)
            left ^= true;
    //}
    return this->Position;
}

// resets the ball to initial Stuck Position (if ball is outside window bounds)
void BallObject::Reset(glm::vec2 position, glm::vec2 velocity)
{
    this->Position = position;
    this->Velocity = velocity;
    this->Stuck = true;
    this->Sticky = false;
    this->PassThrough = false;

}

void BallObject::Draw(SpriteRenderer &renderer){
    
    // line

    
    //glClearColor (0.0, 0.0, 0.0, 0.0);
    //glClear (GL_COLOR_BUFFER_BIT);
    glLineWidth(2.0f);//设置线段宽度

    
    Shader lineshader = ResourceManager::GetShader("line");

    glm::vec4 pos = glm::vec4(this->Position + glm::vec2(Radius,Radius), 0,0);

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(this->Position, 0.0f));
    pos = model * pos;


    float vertexs[]={
        //this->Position.x, this->Position.y, 0,
        //this->FixedPoint.x, this->FixedPoint.y,0
        //1.0f,0.0f,0.0f,1.0f, 0.0f, 0.0f,
        //0.0f,0.0f,0.0f,1.0f, 1.0f, 1.0f,
        pos.x, pos.y , 0,  1.0f, 0.0f, 1.0f,   // 右下
        this->FixedPoint.x, this->FixedPoint.y,0,  1.0f, 0.0f, 1.0f, 
    };

    //GLuint VBO,VAO;
    //glGenVertexArrays(1, &VAO);
    //glGenBuffers(1,&VBO);
    //glBindVertexArray(VAO);

    
    //glBufferData(GL_ARRAY_BUFFER,sizeof(vertexs),vertexs,GL_DYNAMIC_DRAW);

    //lineshader.SetMatrix4("model1", model);

    glBindBuffer(GL_ARRAY_BUFFER,lineVBO);
    glBufferSubData(GL_ARRAY_BUFFER,0,sizeof(vertexs),vertexs);
    
    //glBindBuffer(GL_ARRAY_BUFFER,VBO);
    //glVertexAttribPointer(0,3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    //glEnableVertexAttribArray(0);

    //glVertexAttribPointer(1,3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    //glEnableVertexAttribArray(1);

    //glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    //glClear(GL_COLOR_BUFFER_BIT);
    lineshader.Use();
    glBindVertexArray(lineVAO);
    glDrawArrays(GL_LINES,0,2);
    //glDisableVertexAttribArray(0);
    //glDeleteVertexArrays(2, &VAO);

    renderer.DrawSprite(this->Sprite, this->Position, this->Size, this->Rotation, this->Color);
    
}