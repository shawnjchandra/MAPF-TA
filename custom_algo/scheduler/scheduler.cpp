#include "scheduler.h"

void Scheduler::schedule_initialize(int preprocess_time_limit){
    env->reserved_task_schedule.resize(env->num_of_agents);
    
    return;
}

void Scheduler::schedule_plan(int preprocess_time_limit, std::vector<int> & proposed_schedule, std::vector<int> & reserved_schedule) {

}