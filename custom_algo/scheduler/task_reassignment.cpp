#include "task_reassignment.h"
#include "scheduler.h"
#include "../heuristics.h"
#include "congestion.h"

namespace CustomAlgo{
    
    void schedule_tasks(std::vector<int>& agt_dtr, SharedEnvironment* env, std::vector<int> proposed_schedule, int gamma, int square_density) {
        
        for (int agt_id: agt_dtr) {
            Task best_task = {};
            int best_makespan = INTERVAL_MAX;

            State agt_state = env->curr_states[agt_id];
        
            for (auto& pair : env->task_pool) {
                Task& t = pair.second;
                bool is_reserved = false;

                if (t.agent_assigned != -1 && t.agent_assigned != agt_id) continue;

                if (!env->reserved_task_schedule.empty())
                    for (int r_id : env->reserved_task_schedule) {
                        if (r_id == t.task_id){
                            is_reserved = true;
                            break;
                        }
                    }
                    
                if (is_reserved) continue;
                
                if ( t.agent_assigned != -1 && t.agent_assigned != agt_id) continue;

                int task_loc = t.locations.front();

                int curr_makespan = CustomAlgo::query_heuristic(env, agt_state.location, agt_state.orientation, task_loc);

                double delta = gamma * CustomAlgo::task_square_density(task_loc, curr_makespan, env);

                int estimated_task_makespan = curr_makespan + (int) delta;

                if (estimated_task_makespan >= best_makespan ) continue;

                if (env->makespan.count(t.task_id) && estimated_task_makespan > env->makespan[t.task_id])         
                    continue;

                best_task = t;
                best_makespan = estimated_task_makespan;

            }

            // if (std::find(proposed_schedule.begin(), proposed_schedule.end(), best_task.task_id) !=  proposed_schedule.end()) {
            //     auto rtrv_agt = std::find(proposed_schedule.begin(), proposed_schedule.end(), best_task.task_id);
            //     agt_dtr.push_back(rtrv_agt);
            // }
            
            if (best_task.is_assigned()) {
                for (int i = 0 ; i < proposed_schedule.size() ; i ++) {
                    if (proposed_schedule[i] == best_task.task_id && i != agt_id) {
                        proposed_schedule[i ] = -1;
                        break;
                    }
                }
            }

            proposed_schedule[agt_id] = best_task.task_id;
            env->makespan[best_task.task_id] = best_makespan;

        }
        
    }
}