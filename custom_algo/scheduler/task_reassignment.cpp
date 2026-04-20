#include "task_reassignment.h"
#include "scheduler.h"
#include "heuristics.h"
#include "congestion.h"

namespace CustomAlgo{

    void schedule_tasks(std::vector<int>& agt_dtr, SharedEnvironment* env, std::vector<int>& proposed_schedule, double gamma, std::unordered_set<int>& reserved_set, std::unordered_set<int>& free_tasks) {
    
        auto t_start = std::chrono::steady_clock::now();
        cout << "[t=" << env->curr_timestep << "] "
     << "free_agents=" << env->new_freeagents.size()
     << " free_tasks=" << env->new_tasks.size()
     << " agt_dtr(after insert)=" << agt_dtr.size()  // after the loop
     << " total_agents=" << env->num_of_agents
     << endl;
    
        std::unordered_map<int,int> task_to_agt;
        for (int i = 0 ; i < proposed_schedule.size() ; i++) {
            if (proposed_schedule[i] != -1)  {
                task_to_agt[proposed_schedule[i]] = i;
            }
            
        }


        std::unordered_set<int> removed_task;
        removed_task.reserve(agt_dtr.size());
    
        int assigned_count = 0;
        int unassigned_count = 0;
        int displaced_count = 0;

        // --- DIAGNOSTIC COUNTERS ---
        int reject_reserved = 0;
        int reject_opened = 0;
        int reject_overflow = 0;
        int reject_worse_than_best = 0;
        int reject_beaten_by_owner = 0;
        int reject_missing_task = 0;

        std::cout << "\n--- TASK AUTOPSY (Checking the remaining " << free_tasks.size() << " tasks) ---\n";
        int trapped_tasks = 0;
        int missing_ht = 0;
        int check_limit = 10; // Limit prints to avoid console spam

        // int MAX_TRIES = 0.25 * env->task_pool.size();
        // int MAX_TRIES = 300;
        for (int dtr_i = 0 ; dtr_i < agt_dtr.size() ; dtr_i++) {
            int agt_id = agt_dtr[dtr_i];
            
            int best_task = -1;
            int best_makespan = INTERVAL_MAX;
            
            State agt_state = env->curr_states[agt_id];
            
            // int tries = -1;
            for (int f_t_id : free_tasks) {
         
                // Kalau ga ada di task_to_agent, pastiin ga ada di makespan juga
                if(task_to_agt.find(f_t_id) == task_to_agt.end()) env->makespan.erase(f_t_id);


                auto task_it = env->task_pool.find(f_t_id);
                
                if (task_it == env->task_pool.end()) { reject_missing_task++; continue; }
                auto t = task_it->second;

                int t_loc = t.locations[0]; // First location of the task
                int c_dest = env->hpa_h.voronoi_map[t_loc];
                
                if (env->map[t_loc] == 1 || c_dest == -1) {
                    if (check_limit-- > 0) std::cout << "[TASK DEAD] Task " << f_t_id << " at Loc " << t_loc << " is inside a wall (-1)!\n";
                    trapped_tasks++;
                } else if (env->hpa_h.Gates[c_dest].empty()) {
                    if (check_limit-- > 0) std::cout << "[TASK TRAPPED] Task " << f_t_id << " is in Cluster " << c_dest << " with ZERO gates!\n";
                    trapped_tasks++;
                } else {
                    // Check if the task can be reached from its own gate (Phase 3 of HPA*)
                    int first_gate = env->hpa_h.Gates[c_dest][0];
                    int g_local = env->hpa_h.global_to_local[first_gate];  // gate = KEY
                    int task_local = env->hpa_h.global_to_local[t_loc];     // task loc = INDEX into the vector

                    if (env->hpa_h.IntraHT[c_dest].find(g_local) != env->hpa_h.IntraHT[c_dest].end()) {
                        int cost = env->hpa_h.IntraHT[c_dest][g_local][task_local][0];  // can task be reached?
                        if (cost >= INTERVAL_MAX) trapped_tasks++;
                    } else {
                        missing_ht++;
                    }
                }

                // Skip kalau udah masuk ke reserved. Awalnya loop tapi nanti jadi O(A)
                if (reserved_set.count(t.task_id)) {reject_reserved++;continue;}
    
                // Skip kalau udah di opened
                if (t.idx_next_loc > 0 ) { reject_opened++; continue; }

                int curr_makespan = 0;
                int c_loc = agt_state.location;
                int c_orient = agt_state.orientation;
                double delta = 0;

                // Skip kalau task udah di opened sama agen lain
                // if (t.agent_assigned != -1 && t.agent_assigned != agt_id) continue;
                // if (env->task_pool[t.task_id].agent_assigned != -1) continue;
    
          
                //Lokasi selanjutnya (dalam Task, loc pertama diinit = 0, baru ke lokasi yang terdaftar)
                // if (t.idx_next_loc >= (int)t.locations.size()) continue;
    
                curr_makespan += query_heuristic(env, c_loc, c_orient, t.locations[0]);
                if (curr_makespan >= INTERVAL_MAX) { reject_overflow++; continue; } // overflow guard
                
                for (int i = t.idx_next_loc; i < (int)t.locations.size(); i++) {
                    int errand_loc = t.locations[i];

                    curr_makespan += manhattanDistance(c_loc, errand_loc, env);
                    // curr_makespan += query_heuristic(env, c_loc, c_orient, errand_loc);

                    c_loc = errand_loc;
                    c_orient = 0; // orientation unknown after first hop, use any

                    delta = gamma * task_square_density(errand_loc, curr_makespan, env);
                }

                
                int estimated_task_makespan = curr_makespan + (int) delta;
                
                //Cek sama best makespan sekarang
                if (estimated_task_makespan >= best_makespan ) { reject_worse_than_best++; continue; }
                
                //Cek sama makespan yang udah disimpan
                auto ms_it = env->makespan.find(t.task_id);
       

                if (ms_it != env->makespan.end() && estimated_task_makespan >= ms_it->second) { reject_beaten_by_owner++; continue; }

                
                best_task = t.task_id;
                best_makespan = estimated_task_makespan;
                
            }
            
            if (best_task == -1) {
                proposed_schedule[agt_id] = -1;
                unassigned_count++;
                std::cout << "[DEBUG] Agent " << agt_id << " at Loc " << agt_state.location 
                        << " FAILED. Reasons for rejecting " << free_tasks.size() << " tasks -> "
                        << " Reserved:" << reject_reserved
                        << " Opened:" << reject_opened
                        << " Overflow(HPA*):" << reject_overflow
                        << " BeatenByOwner:" << reject_beaten_by_owner << std::endl;
                continue;
            }
            
            // Cek best_task udah diambil sama agen lain atau belum. Ambil dan masukan agen B ke agt_dtr untuk reassignment
        
            auto it = task_to_agt.find(best_task);
            // auto it = env->task_pool.find(best_task);
            if (it != task_to_agt.end()) {
    
                int old_agent = it->second;
                
                // Hanya replace kalau beda agen 
                if (old_agent != agt_id) {
           
                    proposed_schedule[old_agent] = -1;
                    agt_dtr.push_back(old_agent);
                    // task_to_agt[best_task] = agt_id;

                    // Hapus, jangan di replace -1
                    env->makespan.erase(best_task);
                    displaced_count++;
                }
            }   
            
            removed_task.insert(best_task);
            
            proposed_schedule[agt_id] = best_task;
            env->makespan[best_task] = best_makespan;
            task_to_agt[best_task] = agt_id;
            
            assigned_count++;
        }
            std::cout << "Results: " << trapped_tasks << " Tasks physically trapped | " << missing_ht << " Tasks missing HT data.\n\n";
         
        for (int t_id : removed_task) {
            free_tasks.erase(t_id);
        }
        
        double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_start).count();
        std::cout << "[SCHEDULE][DTR] t=" << env->curr_timestep
              << " agents=" << agt_dtr.size()
              << " assigned=" << assigned_count
              << " unassigned=" << unassigned_count
              << " displaced=" << displaced_count
              << " free_tasks_remaining=" << free_tasks.size()
              << " time=" << ms << "ms" << std::endl;
    }
}