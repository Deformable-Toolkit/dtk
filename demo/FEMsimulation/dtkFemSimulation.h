/*
 * @Author: tom: https://github.com/TOMsworkspace 
 * @Date: 2021-09-03 16:12:05 
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 17:22:45
 */


#ifndef DTKFEMSIMULATION_H
#define DTKFEMSIMULATION_H

#include "dtkScene.h"
#include "../../src/dtk.h"
#include "../../src/dtkGraphicsKernel.h"
#include <iostream>
#include <vector>

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using namespace dtk;

class dtkFemSimulation  : public dtkScene
{

private:
    /* data */
    vector<Eigen::Vector2f> points;//(n_node);  /**< 点的位置 */

    vector<Vector2f> pre_points;//(n_node);  /**< 点之前的位置 */

    vector<Eigen::Vector2f> points_force;
    
    vector<Vector2f> points_v;//(n_node); /**< 点的速度 */

    vector<Eigen::Matrix2f> B;//(n_fem_element); /**< 微元本身长度的逆 */

    float total_energy; /** 总势能 */
    float pre_total_energy; /** 之前的总势能 */

    std::vector<std::vector<int> > MeshTable;//(n_fem_element, std::vector<int>(3,0)); /**< 三角微元下标 */

    dtk::dtkGraphicsKernel::Circle2 sphere;//(dtk::dtkGraphicsKernel::Point2(0.5,0.2), 0.1);

    Vector2f spherecenter;
    
    Matrix2f compute_D(int i);

    Matrix2f compute_P(int i);
    
    void compute_B();

    void compute_force();

    void compute_total_energy();

public:

    dtkFemSimulation(unsigned int width, unsigned int height);
    ~dtkFemSimulation();
    void Init();
    
    float getEnergy();

    //loop

    void moveBall(int x, int y);
    void ProcessInput(float dt) ;
    void Update(float dt) ;
    void Render() ;
    void DoCollisions() ;
};

#endif
