#include "task_reassignment.h"
#include "scheduler.h"
#include "heuristics.h"
#include "congestion.h"

namespace CustomAlgo{

    void schedule_tasks(int time_limit, std::vector<int>& agt_dtr, SharedEnvironment* env, std::vector<int>& proposed_schedule, double gamma, std::unordered_set<int>& reserved_set, std::unordered_set<int>& free_tasks) {
        auto start = std::chrono::steady_clock::now();
        auto elapsed = [&](TimePoint from) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - from).count();
        };

        fprintf(stderr, "Time limit DTR:     %ldms\n", time_limit);
        
        std::unordered_map<int,int> task_to_agt;
        for (int i = 0 ; i < proposed_schedule.size() ; i++) {
            if (proposed_schedule[i] != -1)  {
                task_to_agt[proposed_schedule[i]] = i;
            }
            
        }
        
        
        std::unordered_set<int> removed_task;
        removed_task.reserve(agt_dtr.size());
        
        // int MAX_TRIES = 0.25 * env->task_pool.size();
        // int MAX_TRIES = 300;
        int original_size = agt_dtr.size();
        
        for (int dtr_i = 0 ; dtr_i < original_size && elapsed(start) < time_limit ; dtr_i++) {
            int agt_id = agt_dtr[dtr_i];
            
            int best_task = -1;
            int best_makespan = INTERVAL_MAX;
            
            State agt_state = env->curr_states[agt_id];
            
            // int tries = -1;
            for (int f_t_id : free_tasks) {
                // fprintf(stderr, "[t=%d] FT1\n", env->curr_timestep);
                
                // Kalau ga ada di task_to_agent, pastiin ga ada di makespan juga
                if(task_to_agt.find(f_t_id) == task_to_agt.end()) env->makespan.erase(f_t_id);
                
                // fprintf(stderr, "[t=%d] FT1-2\n", env->curr_timestep);
                
                auto task_it = env->task_pool.find(f_t_id);
                
                if (task_it == env->task_pool.end()) continue;
                auto t = task_it->second;
                
                // fprintf(stderr, "[t=%d] FT1-4\n", env->curr_timestep);
                int t_loc = t.locations[0]; // First location of the task
                int c_dest = env->hpa_h.voronoi_map[t_loc];
                
                // fprintf(stderr, "[t=%d] FT1-5 , Gates Cluster %d \n", env->curr_timestep, env->hpa_h.Gates[c_dest][0]);
                // Check if the task can be reached from its own gate (Phase 3 of HPA*)
                
                int first_gate = env->hpa_h.Gates[c_dest][0];
                // fprintf(stderr, "[t=%d] FT1-5-1\n", env->curr_timestep);

                int g_local = env->hpa_h.global_to_local[first_gate];  // gate = KEY
                // fprintf(stderr, "[t=%d] FT1-5-2\n", env->curr_timestep);

                int task_local = env->hpa_h.global_to_local[t_loc];     // task loc = INDEX into the vector
                
                // fprintf(stderr, "[t=%d] FT1-6\n", env->curr_timestep);
                    // if (env->map[t_loc] == 1 || c_dest == -1) continue;
                    // if (env->hpa_h.Gates[c_dest].empty()) continue;
                    
                // fprintf(stderr, "[t=%d] FT2\n", env->curr_timestep);

                int cost = env->hpa_h.IntraHT[c_dest][g_local][task_local][0];  // can task be reached?
                if (cost >= INTERVAL_MAX) continue;


                // Skip kalau udah masuk ke reserved. Awalnya loop tapi nanti jadi O(A)
                if (reserved_set.count(t.task_id))continue;
    
                // Skip kalau udah di opened
                if (t.idx_next_loc > 0 )  continue; 

                int curr_makespan = 0;
                int c_loc = agt_state.location;
                int c_orient = agt_state.orientation;
                double delta = 0;
                // fprintf(stderr, "[t=%d] FT3\n", env->curr_timestep);
                
                curr_makespan += query_heuristic(env, c_loc, c_orient, t.locations[0]);
                if (curr_makespan >= INTERVAL_MAX)  continue; 
                
                for (int i = t.idx_next_loc; i < (int)t.locations.size(); i++) {
                    int errand_loc = t.locations[i];

                    curr_makespan += manhattanDistance(c_loc, errand_loc, env);
                    // curr_makespan += query_heuristic(env, c_loc, c_orient, errand_loc);

                    c_loc = errand_loc;
                    c_orient = 0; // orientation unknown after first hop, use any

                    delta = gamma * task_square_density(errand_loc, curr_makespan, env);
                }

                // fprintf(stderr, "[t=%d] FT4\n", env->curr_timestep);
                
                int estimated_task_makespan = curr_makespan + (int) delta;
                
                //Cek sama best makespan sekarang
                if (estimated_task_makespan >= best_makespan )  continue; 
                
                //Cek sama makespan yang udah disimpan
                auto ms_it = env->makespan.find(t.task_id);
       

                if (ms_it != env->makespan.end() && estimated_task_makespan >= ms_it->second) continue; 

                // fprintf(stderr, "[t=%d] FT5\n", env->curr_timestep);
                
                best_task = t.task_id;
                best_makespan = estimated_task_makespan;
                
            }
            //  fprintf(stderr, "[t=%d] After LOOP\n", env->curr_timestep);
            
            if (best_task == -1) {
                proposed_schedule[agt_id] = -1;
                continue;
            }
            
            // Cek best_task udah diambil sama agen lain atau belum. Ambil dan masukan agen B ke agt_dtr untuk reassignment
            //  fprintf(stderr, "[t=%d] BEFORE REPLACING\n", env->curr_timestep);
        
            auto it = task_to_agt.find(best_task);
            // auto it = env->task_pool.find(best_task);
            if (it != task_to_agt.end()) {
    
                int old_agent = it->second;
                
                // Hanya replace kalau beda agen 
                if (old_agent != agt_id) {
           
                    proposed_schedule[old_agent] = -1;
                    agt_dtr.push_back(old_agent);

                    // Hapus, jangan di replace -1
                    env->makespan.erase(best_task);
                }
            }   
            
            //  fprintf(stderr, "[t=%d] After REPLACING\n", env->curr_timestep);

            removed_task.insert(best_task);
            
            proposed_schedule[agt_id] = best_task;
            env->makespan[best_task] = best_makespan;
            task_to_agt[best_task] = agt_id;
            
        }
         
        for (int t_id : removed_task) {
            free_tasks.erase(t_id);
        }
        fprintf(stderr, "Elapsed Time DTR :     %ldms\n", elapsed(start));
        
    }
}