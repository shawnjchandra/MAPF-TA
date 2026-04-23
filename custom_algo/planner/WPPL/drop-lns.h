#pragma once 

#include "wppl.h"
#include <vector>
#include <chrono>

namespace CustomAlgo {
    using TimePoint = std::chrono::steady_clock::time_point;

    int compute_soc(
        std::vector<std::vector<State>>& planned,
        int w,
        std::vector<int>& goal_per_agent,
        SharedEnvironment* env
    );

    std::vector<std::vector<State>> repair(
        std::vector<std::vector<State>>& current_plan,
        std::vector<int>& destroyed_agents,
        std::vector<int>& ids,
        std::vector<int>& goal_per_agent,
        const std::vector<double>& p,
        SharedEnvironment* env
    );

    void run_lns (
        WPPLState& wppl,
        SharedEnvironment* env,
        std::vector<int>& ids,
        std::vector<int>& goal_per_agent,
        const std::vector<double>& p,
        TimePoint deadline
    );
}