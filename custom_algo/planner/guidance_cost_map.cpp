#include "guidance_cost_map.h"

namespace CustomAlgo {
    void gcm_update(SharedEnvironment* env ,int loc, int orient, int curr_timestep){
        float wait_rate = (float)env->planner_state.wait_map[loc][orient] / (float)env->planner_state.gcm_freq;
        
        float w =(1-env->alpha) * env->planner_state.gcm[loc][orient] + env->alpha * wait_rate;
        env->planner_state.gcm[loc][orient] = w;
        
        if (w > env->planner_state.w_peak[loc].val) {
            env->planner_state.w_peak[loc].val = w;
            env->planner_state.w_peak[loc].timestep = curr_timestep;
        }
    }

    void gcm_cooldown(SharedEnvironment* env, int curr_timestep){
        auto& ps = env->planner_state;

        for (int loc = 0 ; loc < env->map.size(); loc++) {
            int deg = env->degree_map[loc];

            float k_v = ps.k_base * (float)deg / (float)ps.max_degree;

            float t_since = (float)curr_timestep - (float)ps.w_peak[loc].timestep;

            float cooled = ps.w_baseline + ((float)ps.w_peak[loc].val - ps.w_baseline) * exp(-k_v * t_since);
            
            cooled = max(ps.w_baseline, cooled);
            
            for (int orient = 0; orient < 4; orient++) {
                float curr = ps.gcm[loc][orient];

                ps.gcm[loc][orient] = max(ps.w_baseline,
                    curr > cooled ? cooled : curr);
            }
        }
    }
}