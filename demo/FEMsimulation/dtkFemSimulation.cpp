/*
 * @Author: tom: https://github.com/TOMsworkspace 
 * @Date: 2021-09-03 15:32:52 
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 19:17:13
 */

#include <algorithm>
#include <sstream>
#include <iostream>

#include <Eigen\\Dense>
using namespace std;
using namespace Eigen;


//#include <gl/GL.h>
//#include <glad/glad.h>
//#include <GLFW/glfw3.h>


//#include <irrklang/irrKlang.h>
//using namespace irrklang;

#include "dtkFemSimulation.h"

#include "GL/freeglut.h"

//#include "resource_manager.h"
//#include "sprite_renderer.h"


int dim = 2;
int n_node_x = 50;
int n_node_y = 6;
float node_mass = 1.0f;
int n_node = n_node_x * n_node_y;
int n_fem_element = (n_node_x - 1) * (n_node_y - 1) * 2;
float deltat = 3e-4;
float deltax = (1.0 / 32);

float Young_E = 1000.0f; /**< 杨氏模量 */
float Poisson_r = 0.3f; /**< 泊松比 [0 - 0.5] */
float Lame_parameter_1 = Young_E / (2 * (1 + Poisson_r));
float Lame_parameter_2 = Young_E * Poisson_r / ((1 + Poisson_r) * (1 - 2 * Poisson_r));
float element_v = 0.01f; /**< 微元体积 */

float radius = 0.05;

int iterate_time = 3;

inline int mesh(int i, int j) {
    return i * n_node_y + j;
}


dtkFemSimulation::dtkFemSimulation(unsigned int width, unsigned int height)
: dtkScene(width, height),
points(n_node),pre_points(n_node),points_v(n_node),points_force(n_node),B(n_fem_element),
MeshTable(n_fem_element,std::vector<int>(3,0)),sphere(dtk::dtkGraphicsKernel::Point2(0.5,0.25), radius),
total_energy(0),pre_total_energy(0),spherecenter(0.5,0.25)
{ 

}

Matrix2f dtkFemSimulation::compute_D(int i){
    int a = MeshTable[i][0];
    int b = MeshTable[i][1];
    int c = MeshTable[i][2];

    Matrix2f ans;
    ans(0,0) = points[a][0] - points[c][0];
    ans(0,1) = points[b][0] - points[c][0];
    ans(1,0) = points[a][1] - points[c][1];
    ans(1,1) = points[b][1] - points[c][1];
    return ans;
}


void dtkFemSimulation::compute_B(){
    for(int i = 0; i < n_fem_element; ++i){
        this->B[i] = compute_D(i).inverse();
        //cout << setw(6) << B[i] << endl;
    }
}

Matrix2f dtkFemSimulation::compute_P(int i){
    Matrix2f D = compute_D(i);
    Matrix2f F = D * B[i];

    Matrix2f F_T = F.transpose().inverse();

    float J = max(0.5f, F.determinant()); /**< 形变率 */

    return Lame_parameter_1 * (F - F_T) + Lame_parameter_2 * log(J) * F_T ;
    //Matrix2f ans = D * this->B[i];
}

void dtkFemSimulation::compute_total_energy(){

    //this->total_energy = 0.0f;
    for(int i = 0; i < n_fem_element; ++i){
        Matrix2f D = compute_D(i);
        Matrix2f F = D * B[i];

        //NeoHooken
        float I1 = (F * F.transpose()).trace();
        float J = max(0.2f, (float)F.determinant()); /**< 形变率 */

        //cout << J << endl;
        
        float element_energy_density = 0.5 * Lame_parameter_1 * (I1 - dim) - Lame_parameter_1 * log(J) + 0.5 * Lame_parameter_2 * log(J) * log(J);
        this->total_energy += element_energy_density * element_v;
    }
}

dtkFemSimulation::~dtkFemSimulation()
{
}

void dtkFemSimulation::Init()
{
    dtkScene::Init();
    //TODO: load shaders

    //TODO: configure shaders

    //TODO: load textures
    
    //TODO: set render-specific controls

    //TODO: configure Scene objects

    //build mesh
    for(int i = 0; i < n_node_x; ++i){
        for(int j = 0; j < n_node_y; ++j){
            int idx = mesh(i,j);
            //this->points[idx][0] = -14 + i * deltax * 0.5;
            //this->points[idx][1] = 8 + j * deltax * 0.5 + i * deltax * 0.05;

            this->points[idx][0] = 0.1f + i * deltax * 0.5f;
            this->points[idx][1] = 0.5f + j * deltax * 0.5f + i * deltax * 0.1f;
            this->points_v[idx][0] = 0.0f;
            this->points_v[idx][1] = -1.0f; 
        }
    }

    //this->pre_points = points;

    for(int i = 0; i < n_node_x - 1; ++i ){ 
        for(int j = 0; j < n_node_y - 1; ++j){
            //element id
            int eidx = (i * (n_node_y - 1) + j) * 2;
            this->MeshTable[eidx][0] = mesh(i,j);
            this->MeshTable[eidx][1] = mesh(i + 1, j);
            this->MeshTable[eidx][2] = mesh(i,j + 1);

            eidx = (i * (n_node_y - 1) + j) * 2 + 1;
            this->MeshTable[eidx][0] = mesh(i,j + 1);
            this->MeshTable[eidx][1] = mesh(i + 1,j + 1);
            this->MeshTable[eidx][2] = mesh(i + 1,j);
        }
    }

    compute_B();
    
    //TODO: audio
}

void dtkFemSimulation::compute_force(){
    for(int i = 0; i < n_node ; ++i){
        this->points_force[i] = Vector2f(0.0f, - 10.0f * node_mass);
    }

    for(int i = 0; i < n_fem_element; ++i){

        Matrix2f P = compute_P(i);
        Matrix2f H = - element_v * (P * (this->B[i].transpose()));

        Vector2f h1 = Vector2f(H(0,0), H(1,0));
        Vector2f h2 = Vector2f(H(0,1), H(1,1));

        int a = this->MeshTable[i][0];
        int b = this->MeshTable[i][1];
        int c = this->MeshTable[i][2];

        this->points_force[a] += h1;
        this->points_force[b] += h2;
        this->points_force[c] += -(h1 + h2);
    }
}

void dtkFemSimulation::Update(float dt)
{
    //TODO: update objects
    //TODO: check for object collisions
    if(this->State == SCENE_ACTIVE){
        
        // 迭代多轮, 防止穿透
        for(int i = 0; i < iterate_time ; ++i){
            //this->pre_total_energy = total_energy; 
            compute_total_energy();
            DoCollisions();
            compute_force();

            //cout << total_energy << endl;
        
            //float deltaU = this->total_energy - this->pre_total_energy;
            //deltaU = abs(deltaU) < 1e-9 ? 0 : deltaU; 
            for(int i = 0; i < n_node; ++i){
                // update points

                //Vector2f deltaX = this->points[i] - this->pre_points[i];

                //Vector2f diffUtoX = Vector2f(deltaU / deltaX[0], deltaU / deltaX[1]);

            
               // Vector2f diffUtoX = Vector2f(abs(deltaX[0]) > 1e-4 ? deltaU / deltaX[0] : 0.0f, abs(deltaX[1]) > 1e-4 ? deltaU / deltaX[1] : 0.0f);

                //diffUtoX = Vector2f(0.0f,0.0f);

                //cout << diffUtoX << endl;

               // this->points_v[i] = (this->points_v[i] + ((- diffUtoX / node_mass) + Vector2f(0.0f, -10.0f)) * deltat) * exp(deltat * -6);

                this->points_v[i] = (this->points_v[i] + (this->points_force[i] / node_mass) * deltat) * exp(deltat * -3); 
                //this->pre_points[i] = this->points[i];

                this->points[i] += deltat * this->points_v[i];
            }

            
           // this->pre_points = this->points;
           // this->pre_total_energy = this->total_energy;
        }
    }
}


void dtkFemSimulation::ProcessInput(float dt)
{   
    dtkScene::ProcessInput(dt);
    //TODO: process input(keys)
    
}


void dtkFemSimulation::Render()
{
    //if(this->State == SCENE_ACTIVE){
        //TODO: draw circle

        Vector2f center = spherecenter;
        //Vector2f(this->sphere.center()[0], this->sphere.center()[1]);

        glColor3f(0x06 * 1.0 / 0xff, 0x85 * 1.0 / 0xff, 0x87 * 1.0 / 0xff);
        glBegin(GL_POLYGON);

        int n = 100;
        for (int i = 0; i < n; i++)
        {
            glVertex2f(center[0] + radius * cos(2 * dtk::dtkPI / n * i), center[1] + radius * sin(2 * dtk::dtkPI / n * i));		
        }
        glEnd();
        

        //TODO: draw fem element(triangles here)
        glColor3f(0x4f * 1.0 / 0xff, 0xb9 * 1.0 / 0xff, 0x9f * 1.0 / 0xff);
        glBegin(GL_LINES);
        for(int i = 0; i < n_fem_element; ++i){
            for(int j = 0 ; j < 3; ++j){
                int a = this->MeshTable[i][j];
                int b = this->MeshTable[i][(j  + 1) % 3];

                //draw line from a to b;
                glVertex2f(this->points[a][0] , this->points[a][1] );
                glVertex2f(this->points[b][0], this->points[b][1] );
            }
        }
        //glEnd();

        glColor3f(1.0, 1.0, 1.0);
        //glBegin(GL_LINES);
        glVertex2f(0.0f, 0.2f);
        glVertex2f(1.0f, 0.2f);

        glVertex2f(1.0f, 0.85f);
        glVertex2f(1.0f, 0.2f);

        glVertex2f(1.0f, 0.85f);
        glVertex2f(0.0f, 0.85f);

        glVertex2f(0.0f, 0.85f);
        glVertex2f(0.0f, 0.2f);
        glEnd();
   // }
}

// collision detection

void dtkFemSimulation::DoCollisions()
{
    if(this->State == SCENE_ACTIVE){
        Vector2f center = spherecenter;
        //Vector2f(this->sphere.center()[0], this->sphere.center()[1]);
        float radius = this->sphere.squared_radius();
        
        for(int i = 0; i < n_node; ++i){
            //# Collide with sphere
            
            Vector2f dis = this->points[i] - center;
            if((float)(dis.dot(dis)) < radius * radius)
            {
                Vector2f normal = dis.normalized();
                
                this->points[i] = center + radius * normal;
                this->points_v[i] -=  (this->points_v[i].dot(normal)) *  normal;
            }
            
            
            // Collide with ground

            if(this->points[i][1] < 0.2f) {
                this->points[i][1] = 0.2f;
                this->points_v[i][1] = 0.0f;
            }

            if(this->points[i][1] > 0.9f) {
                this->points[i][1] = 0.9f;
                this->points_v[i][1] = 0.0f;
            }

            if(this->points[i][0] < 0.0f) {
                this->points[i][0] = 0.0f;
                this->points_v[i][0] = 0.0f;
            }

            if(this->points[i][0] > 1.0f) {
                this->points[i][0] = 1.0f;
                this->points_v[i][0] = 0.0f;
            }
        }
        
    }
}

void dtkFemSimulation::moveBall(int x, int y){

    //this->spherecenter = Vector2f((x) * 1.0 / 800 , (600 - y) * 1.0 / 600 );
}

float dtkFemSimulation::getEnergy(){
    return this->total_energy;
}