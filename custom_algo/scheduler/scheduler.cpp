#include "scheduler.h"
#include "congestion.h"
#include "task_reassignment.h"
#include "chaining.h"
#include "heuristics.h"

namespace CustomAlgo {

    std::unordered_set<int> free_agents;
    std::unordered_set<int> not_opened_tasks;

    /**
     * 
     * Agent:
     * env->new_freeagents hanya diupdate kalau task mereka selesai. Dan sebelumnya udah di clear oleh TaskManager, 
     * 
     * Schedule:
     * Proposed_schedule ga pernah diclear atau diganti sama system, hanya sama user atau kode algo
     * 
     * Tasks:
     * new_freetask itu di clear.. dan ditambahin kalo udah ada task yang selesai
     *  
     * 
     */

    void get_newly_completed_tasks(SharedEnvironment* env, std::vector<int>& proposed_schedule, std::unordered_set<int>& reserved_set) {

            if (env->curr_timestep == 0) return;
        
            for (int agt_id : env->new_freeagents) {
                int finished_task = proposed_schedule[agt_id];
                if (finished_task != -1) {
                    env->newly_completed_tasks.push_back(finished_task);

                    if (reserved_set.count(finished_task)) {
                        reserved_set.erase(finished_task);
                    }
            }
        }
    }

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env){
        env->reserved_task_schedule.resize(env->num_of_agents,-1);
        env->square_density.resize(env->k,0);
        env->dbc_reserved.resize(env->num_of_agents, false);
        return;
    }

    void schedule_plan(int preprocess_time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env) {
        std::unordered_set<int> reserved_set;
        reserved_set.insert(env->reserved_task_schedule.begin(), env->reserved_task_schedule.end());
        reserved_set.erase(-1);

        free_agents.clear();
        not_opened_tasks.clear();

        // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
        free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
        
        //Selalu diisi sama task baru (task yang lama ga ada...)
        not_opened_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

        
        //free tasks isinya gabungan task yang belum diassign dan sudah tapi belum diopen (selama next loc = 0)
        for (auto& [t_id, t] : env->task_pool) {
            // if (t.agent_assigned == -1 && t.t_completed < 0) {
            if (t.idx_next_loc == 0) { // Diisi sama assigned but not opened task.. jadi lengkap
                not_opened_tasks.insert(t_id);
                
            }
        }
            
        for (int t_id : not_opened_tasks) {
            env->makespan.erase(t_id);
        }

        get_newly_completed_tasks(env, proposed_schedule, reserved_set);

        for (int t_id : env->newly_completed_tasks) {
            Task& t = env->task_pool[t_id];
            env->total_actual_duration+= t.t_completed - t.t_revealed;
            env->total_min_duration+= env->makespan[t_id];
            
            env->num_task_finished+=1;

            // Kalau sebelumnya DBC dan baru selesai, kembaliin ke false reserved
            if (env->curr_task_schedule[t.agent_assigned] == -1 && env->dbc_reserved[t.agent_assigned]) {

                env->dbc_reserved[t.agent_assigned] = false;
                env->reserved_task_schedule[t.agent_assigned] = -1;

            }
        }
        env->newly_completed_tasks.clear();
        env->newly_completed_tasks.reserve(env->task_pool.size());

        double gamma = 0.0;
        if (env->num_task_finished > 0) {
            gamma = (double)(env->total_actual_duration - env->total_min_duration) / (double)env->num_task_finished;

            if (gamma < 0.0) gamma = 0;
        }

        /**
         * @brief Bagian utama, ambil agen sesuai kondisi masing-masing
         * 
         */
        std::vector<int> agt_dtr;
        std::vector<int> opened_agt;

        // Reserve kapasistas sesuai jumlah agen
        agt_dtr.reserve(env->num_of_agents);
        opened_agt.reserve(env->num_of_agents);
        
        for (int agt_id =0 ; agt_id < env->num_of_agents; agt_id++) {
            //Agen -> task | idx -> value
            
            if (free_agents.find(agt_id) != free_agents.end()) {

                //Kalo agen punya reserved dan lagi ga ngerjain task
                if (env->reserved_task_schedule[agt_id] != -1) {
                    int reserved_t_id = env->reserved_task_schedule[agt_id];
                    auto it = env->task_pool.find(reserved_t_id);

                    if (it != env->task_pool.end() && it->second.agent_assigned  == -1 && it->second.idx_next_loc == 0) {
                        proposed_schedule[agt_id] = reserved_t_id;

                        // Harus ada penanda kalau schedule ini dari reserved dan ga bisa diambil DTR...
                        reserved_set.insert(it->second.task_id);

                    } else {
                        // Reserved task became invalid — treat agent as free
                        agt_dtr.push_back(agt_id);
                        env->dbc_reserved[agt_id] = false;
                    }
                    env->reserved_task_schedule[agt_id] = -1;
                } else {
                    agt_dtr.push_back(agt_id);
                }

                //Kalu ga punya reserved (artinya bebas dari awal)
            } else {
                int curr_task_id = env->curr_task_schedule[agt_id];
                if (curr_task_id != -1) {
                    auto& t = env->task_pool[curr_task_id];
                    
                    if (t.idx_next_loc > 0 && !env->dbc_reserved[agt_id]) {
                        opened_agt.push_back(agt_id);
                    }
                } else {
                    // Agent has no task and isn't newly free — re-add to DTR pool
                    agt_dtr.push_back(agt_id);
                }
            }

        }


        //Update square density masing-masing cluster
        calc_square_density(env);

        // DTR
        schedule_tasks(agt_dtr, env, proposed_schedule, gamma, reserved_set, not_opened_tasks);

    
        //Chaining
        chaining_task(opened_agt, env, proposed_schedule, gamma, reserved_set, not_opened_tasks);
     
        }
}