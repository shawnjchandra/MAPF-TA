#pragma once

#include "Types.h"
#include <random>
#include "SharedEnv.h"


namespace CustomAlgo{

    
    void planner_initialize(int preprocess_time_limit, SharedEnvironment* env);

    void planner_plan(int time_limit,vector<Action> & actions,  SharedEnvironment* env);

    std::unordered_set<int> find_deadend_agents(SharedEnvironment* env);
}