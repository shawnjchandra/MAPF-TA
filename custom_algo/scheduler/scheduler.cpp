#include "scheduler.h"
#include "congestion.h"
#include "task_reassignment.h"
#include "chaining.h"

namespace CustomAlgo {

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env){
        env->reserved_task_schedule.resize(env->num_of_agents,-1);
        env->square_density.resize(env->k,0);
        env->dbc_reserved.resize(env->num_of_agents);
        return;
    }

    void schedule_plan(int preprocess_time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env) {
        
        for (int t_id : env->newly_completed_tasks) {
            Task& t = env->task_pool[t_id];
            env->total_actual_duration+= t.t_completed - t.t_revealed;
            env->total_min_duration+= env->makespan[t_id];
            env->num_task_finished+=1;
        }
        env->newly_completed_tasks.clear();

        double gamma = 0.0;
        if (env->num_task_finished > 0) {
            gamma = (double)(env->total_actual_duration - env->total_min_duration) / (double)env->num_task_finished;

            if (gamma < 0.0) gamma = 0;
        }

        std::vector<int> agt_dtr;
        std::vector<int> opened_agt;

        // Reserve kapasistas sesuai jumlah agen
        agt_dtr.reserve(env->num_of_agents);
        opened_agt.reserve(env->num_of_agents);

        for (int i =0 ; i <env->num_of_agents; i++) {
            //Agen -> task | idx -> value
            int curr_task_id = proposed_schedule[i];

            /*Catatan Penting.
                TaskManager akan langsung ngubah task menjadi = -1 ketika selesai dan langsung sync ke env. Jadi setiap kali sampai di scheduler, env akan selalu yang terbaru (hasil eksekusi).
             */
            // Kalau agen belum diassigned tugas. 
            if (curr_task_id == -1) {

                //Kalo agen punya reserved 
                if (env->reserved_task_schedule[i] != -1) {
                    proposed_schedule[i] = env->reserved_task_schedule[i];
                    env->reserved_task_schedule[i] = -1;                    

                    //Kalau ga ada reserved, masukin ke dtr
                } else {
                    //Kalau pertama kali atau udah selesein reserved tasks, boleh masuk DTR 
                    env->dbc_reserved[i] = false;
                    agt_dtr.push_back(i);
                }

                // Sudah ada tugas
            } else {

                Task& t = env->task_pool[curr_task_id];

                //Masukin 
                if(t.idx_next_loc > 0 && !env->dbc_reserved[i]) {
                    opened_agt.push_back(i);

                } else if (!env->dbc_reserved[i]) {
                    agt_dtr.push_back(i);
                }
            }
        }


        CustomAlgo::calc_square_density(env);

        std::unordered_set<int> reserved_set;
        reserved_set.insert(env->reserved_task_schedule.begin(), env->reserved_task_schedule.end());
        reserved_set.erase(-1);


        // DTR
        CustomAlgo::schedule_tasks(agt_dtr,env,proposed_schedule,gamma, reserved_set);

        //Chaining
        CustomAlgo::chaining_task(opened_agt,env, proposed_schedule, gamma, reserved_set);
    }
}