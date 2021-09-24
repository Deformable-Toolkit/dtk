# include "dtkSPHsolver.h"

dtkSPHSolver::dtkSPHSolver(const vector<int>& res, float screen_to_world_ratio, const vector<int>& bound, float alpha = 0.5, float dx = 0.2
    int max_num_paticles = (1 << 20), bool dynamic_allocate = false, bool adaptive_time_step = true, enmu SPHMethod method = SPHMethod::WCSPH)
    :res(res), screen_to_world_ratio(screen_to_world_ratio),alpha(alpha), dx(dx), max_num_paticles(max_num_paticles),dynamic_allocate(dynamic_allocate),
    adaptive_time_step(adaptive_time_step),method(method)
    {
        dim = res.size();
        padding = 2 * dx;
        g = -9.80;
        rho_0 = 1000.0;
        CFL_v = 0.25;
        CFL_a = 0.05;

        df_fac = 1.3;
        dh = dx * df_fac;
        m = dx * dim * rho_0;

        grid_size = 2 * dh;
        grid_pos = Vector2i(ceil(res[0] / screen_to_world_ratio / grid_size), ceil(res[1] / screen_to_world_ratio / grid_size));

        top_bound = bound[0] / screen_to_world_ratio;
        bottom_bound = bound[1] / screen_to_world_ratio;
        left_bound = bound[2] / screen_to_world_ratio;
        right_bound = bound[3] screen_to_world_ratio ;

        rho_err(max_num_particles);
        max_rho_err(max_num_particles);

        //source_bound()

        particle_num = 0;

        if(dim == 2){
            particle_position(max_num_particles,Vector2f(0.0,0.0));
            particle_velocity(max_num_particles,Vector2f(0.0,0.0));
            particle_position_new(max_num_particles,Vector2f(0.0,0.0));
            particle_velocity_new(max_num_particles,Vector2f(0.0,0.0));
            particle_pressure_acc(max_num_particles, Vector2f(0.0,0.0));

            d_velocity(max_num_particles, Vector2f(0.0,0.0));
        }
        else{
            particle_position(max_num_particles,Vector3f(0.0,0.0,0.0));
            particle_velocity(max_num_particles,Vector3f(0.0,0.0,0.0));
            particle_position_new(max_num_particles,Vector3f(0.0,0.0,0.0));
            particle_velocity_new(max_num_particles,Vector3f(0.0,0.0,0.0));
            particle_pressure_acc(max_num_particles, Vector3f(0.0,0.0,0.0));
            d_velocity(max_num_particles, Vector3f(0.0,0.0));
        }
        
        particle_pressure(max_num_particles, 0.0);
        particle_density(max_num_particles, 0.0);
        particle_density_new(max_num_particles, 0.0);
        particle_alpha(max_num_particles, 0.0);
        particle_stiff(max_num_particles, 0.0);

        d_density(max_num_particles, 0.0);

        color(max_num_particles, 0XFFFFFF);
        material(max_num_particles, SPH_Material::Material_Fiuld);

        //grid_num_particles;
        // grid2particles

        particle_num_neigbors(max_num_particles, 0);
        particle_neighbors(max_num_particles);

        //initizlize dt
        if(method == WCSPH){
            dt = 0.1 * dh / c_0;
            CFL_v = 0.20;
            CFL_a = 0.20;
        }
        else if(method == PCISPH){
            s_f = 1.0;
            if(adaptive_time_step){
                dt = 0.0015;
            }
            else{
                dt = 0.00015;
            }
        }
        else{
            dt = 0.1 * dh / c_0;
            CFL_v = 0.30;
            CFL_a = 0.05;
        }

    }


dtkSPHSolver::~dtkSPHSolver(/* args */) {}

inline int dtkSPHSolver::compute_grid_index(VectorXf pos){
    int res = 0;
    for(int i = 0; i < dim; ++i){
        if(i > 0)
            res *= grid_pos[i - 1];
        res += (int)(pos[i] / 2 * dh);
    }

    return res;
}

void SPHSolver::allocate_particles(){
    // Ref to pbf2d example from by Ye Kuang (k-ye)
    // https://github.com/taichi-dev/taichi/blob/master/examples/pbf2d.py
    // allocate particles to grid
    for(int i = 0; i < particle_num; ++i){
        int cell = compute_grid_index(particle_position[i]);
        int offset = ++grid_num_particles[cell];
        grid2particles[cell][offset] = i;
    }
}
