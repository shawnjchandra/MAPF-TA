#include "chaining.h"
#include "scheduler.h"
#include "congestion.h"

#include "heuristics.h"

namespace CustomAlgo {
    void chaining_task(std::vector<int>& opened_agt ,SharedEnvironment* env , std::vector<int>& proposed_schedule, double gamma, std::unordered_set<int>& reserved_set) {

        std::unordered_map<int,int> loc_to_task;
        for(auto& pair: env->task_pool) {
            Task& t = pair.second;
            if (t.agent_assigned == -1)
                loc_to_task[t.locations.front()] = t.task_id;
        }

        
        for (int agt_id : opened_agt) {
            State a_state = env->curr_states[agt_id];

            int curr_task_id = proposed_schedule[agt_id];
            if (curr_task_id == -1) continue;
            
            int curr_task_goal = env->task_pool[curr_task_id].locations.back();

            //Ambil makespan dari env, ga perlu hitung query lagi karena udah pasti ada task dan disimpan dari DTR
            int curr_makespan = env->makespan[curr_task_id];

            int best_chained_task = -1;
            int best_chain_cost = INTERVAL_MAX;

            for (int offset : env->offsets) {
                int candidate_loc = curr_task_goal + offset;

                if (candidate_loc < 0 || candidate_loc >= env->map.size() || env->map[candidate_loc] == 1) continue;

                // Kalau akses via key / index bisa cek pakai " == 0" tapi asumsi kemungkinan ada map yang loc 0 bisa dipakai, jadi ganti via iterator
                auto loc_it = loc_to_task.find(candidate_loc);
                if (loc_it == loc_to_task.end()) continue;

                int t_id = loc_it->second;

                //Kalo task udah direserved sama agen lain duluan
                if (reserved_set.count(t_id)) continue;

                //Orientasi belum tentu optimal
                int chain_makespan = CustomAlgo::query_heuristic(env, curr_task_goal, a_state.orientation, candidate_loc);

                double delta = gamma * CustomAlgo::task_square_density(candidate_loc, chain_makespan, env);

                int total_chaining_cost = curr_makespan + chain_makespan + delta;

                if (total_chaining_cost > best_chain_cost) continue;

                if (env->makespan.count(t_id) && total_chaining_cost > env->makespan[t_id]) continue; 
                    
                
                best_chained_task = t_id;
                best_chain_cost = total_chaining_cost;
            }

            if (best_chained_task != -1) {
                env->reserved_task_schedule[agt_id] = best_chained_task;
                env->makespan[best_chained_task] = best_chain_cost;
                reserved_set.insert(best_chained_task);
                env->dbc_reserved[agt_id] = true;
            }
         }
    }
}