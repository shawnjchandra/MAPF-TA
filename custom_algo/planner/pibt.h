#pragma once

#include "SharedEnv.h"
#include "States.h"
#include <assert.h>

namespace CustomAlgo {

  std::vector<std::vector<int>> init_pibt_window (
      std::vector<int> non_disabled_agents,
      SharedEnvironment* env
  );

  bool pibt(int curr_id,
            int higher_id,
            std::vector<State>& prev_states,
            std::vector<State>& next_states,
            std::vector<int>& prev_dcs,
            std::vector<int>& next_dcs,
            std::vector<bool>& occupied,
            const std::vector<int> goal_per_agents,

            // const WindowedInstance& instance,
            SharedEnvironment* env
          );

}