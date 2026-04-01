#pragma once
#include "SharedEnv.h"

namespace CustomAlgo{
    void chaining_task(std::vector<int>& opened_agt ,SharedEnvironment* env , std::vector<int>& proposed_schedule, double gamma, std::unordered_set<int>& reserved_set);
}