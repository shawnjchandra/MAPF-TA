#include "scheduler.h"
#include "congestion.h"
#include "task_reassignment.h"
#include "chaining.h"

namespace CustomAlgo {

    void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env){
        env->reserved_task_schedule.resize(env->num_of_agents, -1);
        env->square_density.resize(env->k, 0);
        env->dbc_reserved.resize(env->num_of_agents);

    }

    void schedule_plan(int preprocess_time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env) {
        auto t0 = std::chrono::steady_clock::now();
        auto ms_since = [&](std::chrono::steady_clock::time_point from) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - from).count();
        };

        // // Update completed tasks
        // for (int t_id : env->newly_completed_tasks) {
        //     Task& t = env->task_pool[t_id];
        //     env->total_actual_duration += t.t_completed - t.t_revealed;
        //     env->total_min_duration += env->makespan[t_id];
        //     env->num_task_finished += 1;
        // }
        // env->newly_completed_tasks.clear();

        double gamma = 0.0;
        if (env->num_task_finished > 0) {
            gamma = (double)(env->total_actual_duration - env->total_min_duration)
                  / (double)env->num_task_finished;
            if (gamma < 0.0) gamma = 0;
        }

        env->logger->log_info(
            "gamma=" + std::to_string(gamma) +
            " tasks_finished=" + std::to_string(env->num_task_finished) +
            " | " + std::to_string(ms_since(t0)) + "ms",
            env->curr_timestep);

        std::vector<int> agt_dtr;
        std::vector<int> opened_agt;
        agt_dtr.reserve(env->num_of_agents);
        opened_agt.reserve(env->num_of_agents);

        for (int i = 0; i < env->num_of_agents; i++) {
            int curr_task_id = proposed_schedule[i];

            if (curr_task_id == -1) {
                if (env->reserved_task_schedule[i] != -1) {
                    proposed_schedule[i] = env->reserved_task_schedule[i];
                    env->reserved_task_schedule[i] = -1;
                } else {
                    env->dbc_reserved[i] = false;
                    agt_dtr.push_back(i);
                }
            } else {
                Task& t = env->task_pool[curr_task_id];
                if (t.idx_next_loc > 0 && !env->dbc_reserved[i])
                    opened_agt.push_back(i);
                else if (!env->dbc_reserved[i])
                    agt_dtr.push_back(i);
            }
        }

        env->logger->log_info(
            "agt_dtr=" + std::to_string(agt_dtr.size()) +
            " opened_agt=" + std::to_string(opened_agt.size()) +
            " | " + std::to_string(ms_since(t0)) + "ms",
            env->curr_timestep);

        CustomAlgo::calc_square_density(env);

        env->logger->log_info(
            "density done | " + std::to_string(ms_since(t0)) + "ms",
            env->curr_timestep);

        std::unordered_set<int> reserved_set;
        reserved_set.insert(env->reserved_task_schedule.begin(), env->reserved_task_schedule.end());
        reserved_set.erase(-1);

        CustomAlgo::schedule_tasks(agt_dtr, env, proposed_schedule, gamma, reserved_set);

        for (int task_id : proposed_schedule) {
            if (task_id != -1) {
                reserved_set.insert(task_id);
            }
        }

        env->logger->log_info(
            "dtr done | " + std::to_string(ms_since(t0)) + "ms",
            env->curr_timestep);

        CustomAlgo::chaining_task(opened_agt, env, proposed_schedule, gamma, reserved_set);

        env->logger->log_info(
            "chaining done | total=" + std::to_string(ms_since(t0)) + "ms",
            env->curr_timestep);
        env->logger->flush();
    }
}