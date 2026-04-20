#include "chaining.h"
#include "scheduler.h"
#include "congestion.h"

#include "heuristics.h"

namespace CustomAlgo {
    void chaining_task(std::vector<int>& opened_agt ,SharedEnvironment* env , std::vector<int>& proposed_schedule, double gamma, std::unordered_set<int>& reserved_set, std::unordered_set<int>& free_tasks) {
        auto t_start = std::chrono::steady_clock::now();

        std::unordered_map<int,int> loc_to_task;        
        
        std::unordered_set<int> removed_task;
        removed_task.reserve(opened_agt.size());

        int chained_count = 0;
        int skipped_no_task = 0;
        int skipped_no_improvement = 0;

        // Daftarin task yang belum diopened dan belum dikasihin ke agen lain 
        for(int t_id: free_tasks) {
            auto task_it = env->task_pool.find(t_id);
            if (task_it == env->task_pool.end()) continue;

            Task& t = task_it->second;

            //Kalau udah opened atau udah di assigned ke agen lain, skip
            if (t.idx_next_loc > 0 || t.agent_assigned != -1) continue;

            
            loc_to_task[t.locations.front()] = t.task_id;
    
        }
            
        for (int agt_id : opened_agt) {
            State a_state = env->curr_states[agt_id];

            int curr_task_id = env->curr_task_schedule[agt_id];
            if (curr_task_id == -1) continue;

            
            auto curr_task_it = env->task_pool.find(curr_task_id);
            if (curr_task_it == env->task_pool.end()) {
                skipped_no_task++;
                continue;
            }

            //Ambil goal location (second errand) dari task sekarang 
            int curr_task_goal = curr_task_it->second.locations.back();

            
            //Ambil makespan dari env, ga perlu hitung query lagi karena udah pasti ada task dan disimpan dari DTR
            // auto ms_it = env->makespan.find(curr_task_id);
            // if (ms_it == env->makespan.end()) continue;

            
            // int curr_makespan = ms_it->second;

            int best_chained_task = -1;
            int best_chain_cost = INTERVAL_MAX;

            for (int offset : env->offsets) {
                int candidate_loc = curr_task_goal + offset;

                if (candidate_loc < 0 || candidate_loc >= env->map.size() || env->map[candidate_loc] == 1) continue;

                // Kalau akses via key / index bisa cek pakai " == 0" tapi asumsi kemungkinan ada map yang loc 0 bisa dipakai, jadi ganti via iterator
                auto loc_it = loc_to_task.find(candidate_loc);
                if (loc_it == loc_to_task.end()) continue;

                
                //Kalo task udah direserved sama agen lain duluan
                int t_id = loc_it->second;
                if (reserved_set.count(t_id)) continue;

                
                auto chain_task_it = env->task_pool.find(t_id);
                if (chain_task_it == env->task_pool.end()) continue;

                auto& t = chain_task_it->second;
                if (t.agent_assigned != -1 && t.agent_assigned != agt_id) continue; //Seharusnya 2nd condition tidak perlu karena agen harus dihapus dari opened_agt
                
                /*
                Hitung lagi si makespannya, jangan pakai dari env-> makespan, karena simpannya untuk sebelum opened task -> first errand -> second errand (task goal)
                
                Butuhnya curr location (yang sebenernya bisa first errand juga) ke task goal
                */

                int curr_makespan = query_heuristic(env, a_state.location, a_state.orientation, curr_task_goal);


                //Orientasi belum tentu optimal
                int chain_makespan = CustomAlgo::query_heuristic(env, curr_task_goal, a_state.orientation, candidate_loc);

                double delta = gamma * CustomAlgo::task_square_density(candidate_loc, chain_makespan, env);

                int total_chaining_cost = curr_makespan + chain_makespan + delta;

                if (total_chaining_cost >= best_chain_cost) continue;

                auto t_ms_it = env->makespan.find(t_id);
                if (t_ms_it != env->makespan.end() && total_chaining_cost >= t_ms_it->second) continue; 
                    
                
                best_chained_task = t_id;
                best_chain_cost = total_chaining_cost;
            }

            if (best_chained_task != -1 && !env->dbc_reserved[agt_id]) {

                
                removed_task.insert(best_chained_task);
                reserved_set.insert(best_chained_task);

                env->reserved_task_schedule[agt_id] = best_chained_task;
                env->dbc_reserved[agt_id] = true;
                // env->makespan[best_chained_task] = best_chain_cost;

                chained_count++;
                // proposed_schedule[agt_id] = best_chained_task;
            } else {
                skipped_no_improvement++;
            }
         }

        for (int t_id : removed_task) {
            free_tasks.erase(t_id);
        }

        double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();
        std::cout << "[SCHEDULE][DBC] t=" << env->curr_timestep
              << " opened_agents=" << opened_agt.size()
              << " chained=" << chained_count
              << " skipped_no_task=" << skipped_no_task
              << " skipped_no_improvement=" << skipped_no_improvement
              << " time=" << ms << "ms" << std::endl;

    }
}