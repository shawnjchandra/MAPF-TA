#pragma once

#include "SharedEnv.h"

namespace CustomAlgo {

    struct WPPLState {

        // [t][agent] => ukuran w+1 (windowed planning)
        std::vector<std::vector<State>> wppl_plan;
        int steps_since_plan = 0;
    };

    void wppl_init (WPPLState& wppl, SharedEnvironment* env);
    void run_windowed_pibt(WPPLState& wppl,
                       SharedEnvironment* env,
                       const std::vector<int>& ids,
                       const std::vector<int>& goal_per_agent,
                       std::vector<double>& p,
                       const std::vector<double>& p_copy,
                       const std::vector<DCR>& decided);
}