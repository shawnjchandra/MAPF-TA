#include "task_reassignment.h"
#include "scheduler.h"
#include "heuristics.h"
#include "congestion.h"

namespace CustomAlgo{
    
    void schedule_tasks(std::vector<int>& agt_dtr, SharedEnvironment* env, std::vector<int>& proposed_schedule, double gamma, std::unordered_set<int>& reserved_set) {
        
        std::unordered_map<int,int> task_to_agt;
        for (int i = 0 ; i < proposed_schedule.size() ; i++) 
            if (proposed_schedule[i] != -1) 
                task_to_agt[proposed_schedule[i]] = i;


        for (int agt_id: agt_dtr) {

            int best_task = -1;
            int best_makespan = INTERVAL_MAX;

            State agt_state = env->curr_states[agt_id];
        
            // Isinya [task id, task] 
            for (auto& pair : env->task_pool) {
                Task& t = pair.second;
                bool is_reserved = false;

                // Skip kalau task udah di opened sama agen lain
                if (t.agent_assigned != -1 && t.agent_assigned != agt_id) continue;

                // Skip kalau udah masuk ke reserved. Awalnya loop tapi nanti jadi O(A)
                if (reserved_set.count(t.task_id)) continue;
          
                //Lokasi selanjutnya (dalam Task, loc pertama diinit = 0, baru ke lokasi yang terdaftar)
                int task_loc = t.locations[t.idx_next_loc];

                int curr_makespan = CustomAlgo::query_heuristic(env, agt_state.location, agt_state.orientation, task_loc);

                double delta = gamma * CustomAlgo::task_square_density(task_loc, curr_makespan, env);

                int estimated_task_makespan = curr_makespan + (int) delta;

                //Cek sama best makespan sekarang
                if (estimated_task_makespan >= best_makespan ) continue;

                //Cek sama makespan yang udah disimpan
                if (env->makespan.count(t.task_id) && estimated_task_makespan > env->makespan[t.task_id])         
                    continue;

                best_task = t.task_id;
                best_makespan = estimated_task_makespan;

            }

            // if (std::find(proposed_schedule.begin(), proposed_schedule.end(), best_task.task_id) !=  proposed_schedule.end()) {
            //     auto rtrv_agt = std::find(proposed_schedule.begin(), proposed_schedule.end(), best_task.task_id);
            //     agt_dtr.push_back(rtrv_agt);
            // }
            if (best_task == -1) continue;
            
            // Cek best_task udah diambil sama agen lain atau belum. Ambil dan masukan agen B ke agt_dtr untuk reassignment
            int tta_task_old_agent = task_to_agt[best_task]; 
            if (tta_task_old_agent != -1 && tta_task_old_agent != agt_id) {
                proposed_schedule[tta_task_old_agent] = -1;
                agt_dtr.push_back(tta_task_old_agent);
            }

            proposed_schedule[agt_id] = best_task;
            env->makespan[best_task] = best_makespan;
            task_to_agt[best_task] = agt_id;
        }
        
    }
}