/**
 * @file SPHsolver.h
 * @author TOMsworkspace (2683322180@qq.com)
 * @brief 
 * @version 1.0
 * @date 2021-09-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef DTKSPHSOLVER_H
#define DTKSPHSOLVER_H

#include <vector>
#include <unordered_map>
#include "dtkScene.h"
#include "../../src/dtk.h"
#include "../../src/dtkGraphicsKernel.h"
#include <iostream>

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using namespace dtk;

/**
 * @brief SPH方法
 * 
 */
enum SPHMethod{
    WCSPH = 0,
    PCISPH = 1,
    DFSPH = 2
};

/**
 * @brief 材质，区分边界和流体。
 * 
 */
enum SPH_Material{
    Material_Fiuld = 0,
    Material_Bound = 1
}


class dtkSPHSolver
{

private:
    /* data */
    /**
     * @brief SPH 方法
     */
    enum SPHMethod method;
    bool adaptive_time_step ; /** */
    int dim; /**< 维度 */
    vector<int> res;

    float screen_to_world_ratio;

    bool dynamic_allocate; /**< dynamic allocate memory */

    bool padding;  /**< exernel of paticale with bound */
    int max_num_particles;

    float g = -9.80f; /**< gravity */
    float alpha; /**< material viscosity */
    float rho_0 = 1000.0 ; /**< reference density */
    float CFL_v = 0.25; /**< CFL coefficient for velocity */
    float CFL_a = 0.25; /**< CFL coefficient for acceleration */

    float df_fac = 1.3;
    float dx; /**< Particle radius */
    float dh; /**< Smooth length */
    float dt; /** delta_t */

    // paticle parameters

    float m; /**< 质量 */
    float grid_size;
    Vector2i grid_pos;

    float top_bound;
    float bottom_bound;
    float left_bound;
    float right_bound;

    // ----- WCSPH parameters-----------
    // Pressure state function parameters
    float gamma = 7.0;
    float c_0 = 200.0;

    //------PCISPH parameters-----------
    // Scaling factor for PCISPH
    float s_f;
    int it = 0;
    int max_it = 0;
    int sub_max_iteration = 3;
    vector<float> rho_err;
    vector<float> max_rho_err;

    //------DFSPH parameters-----------
    // Summing up the rho for all particles to compute the average rho
    float sum_rho_err;
    float sum_drho;

    //dynamic fill particles use
    vector<vector<float>> source_bound;
    vector<vector<float>> source_velocity;
    vector<vector<float>> source_pressure;
    vector<vector<float>> source_density;

    int particle_num; /**< num of particles **/
    vector<VectorXf> particle_position;
    vector<VectorXf> particle_velocity;
    vector<VectorXf> particle_position_new; /**< Prediction position values for PCISPH */
    vector<VectorXf> particle_velocity_new; /**< Prediction velocity values for PCISPH */
    
    vector<float> particle_pressure;
    vector<VectorXf> particle_pressure_acc; /**< pressure force for PCISPH */
    vector<float> particle_density;
    vector<float> particle_desity_new; /**< Prediction desity values for PCSPH */
    vector<float> particle_alpha; /**< for DFSPH */
    vector<float> particle_stiff;

    vector<int> color;
    vector<enum SPH_Material> material;

    vector<VectorXf> d_velocity; /**< dv */
    vector<float> d_density; /**< d density */

    unordered_map<int,int> grid_num_particles; /**< particles num of every grid */ 
    unordered_map<int,vector<int>> grid2particles; /**<every grid cotains particle index */ 
    vector<int> particle_num_neigbors; /** neighbor particles num of every paritcle */
    vector<vector<int>> particle_neighbors; /**< every particles's neighbor particle index */

    int max_num_particles_per_cell = 100; 
    int max_num_neighbors = 100;

    float max_a = 0.0;
    float max_a = 0.0;
    float max_rho = 0.0;
    float max_pressuer = 0.0;

    inline int compute_grid_index(VectorXf pos);

    void allocate_particles();

    bool is_in_grid(int cell){
        bool res = true;
    }


public:

    dtkSPHSolver(const vector<int>& res, float screen_to_world_ratio, const vector<int>& bound, float alpha, float dx,
    int max_num_paticles, bool dynamic_allocate, bool adaptive_time_step, enmu SPHMethod method);

    ~dtkSPHSolver(/* args */);



}


#endif