#pragma once

#include "SharedEnv.h"

class Scheduler{
    public:
    std::unordered_set<int> free_agents;
    std::unordered_set<int> free_tasks;
    SharedEnvironment* env;

    Scheduler(SharedEnvironment* env): env(env){};
    Scheduler(){env = new SharedEnvironment();};
    virtual void schedule_initialize(int preprocess_time_limit);
    virtual void schedule_plan(int preprocess_time_limit, std::vector<int> & proposed_schedule, std::vector<int> & reserved_schedule);
};
