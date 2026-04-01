#pragma once
#include <vector>
#include "SharedEnv.h"

namespace CustomAlgo {
    void schedule_tasks(std::vector<int>& agt_dtr, SharedEnvironment* env, std::vector<int>& proposed_schedule, double gamma, std::unordered_set<int>& reserved_set);
}