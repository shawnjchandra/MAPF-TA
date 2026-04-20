#pragma once

#include "SharedEnv.h"
#include "States.h"
#include <assert.h>

namespace CustomAlgo {

    bool pibt(
      int curr_id,
        int higher_id,
        std::vector<State>& prev_states,
        std::vector<State>& next_states,
        std::vector<int>& prev_decision,
        std::vector<int>& decision,
        std::vector<bool>& occupied,
        const std::vector<int> goal_per_agents,
        SharedEnvironment* env,
        std::vector<double> p
      );

    bool moveCheck (int id, std::vector<bool>& checked, std::vector<DCR>& decided, std::vector<Action>& actions, std::vector<int>& prev_decision);
}