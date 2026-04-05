#pragma once

#include "SharedEnv.h"
#include "mapf_utils.h"
#include "Types.h"

/*
destroy_and_repair

snapshot current best plan Pmin
pick destroy heuristic by roulette wheel
select N agents by that heuristic
measure their old partial cost
re-run run_pibt_window on just those N agents
measure new partial cost
if improved:
  merge into Pmin
  increase heuristic weight
else:
  decrease heuristic weight
*/

namespace CustomAlgo {
  
  void run_lns(
    LNS& lns,
    const std::vector<int> non_disabled_agents,
    SharedEnvironment* env,
    int time_limit_ms
  );

  void destroy_and_repair( 
      LNS& lns,
      const std::vector<int> non_disabled_agents,
      SharedEnvironment* env
    );

  DestroyHeuristic select_heuristic_method(
    const std::array<float, (int)DestroyHeuristic::COUNT>& weights
  );

  std::vector<int> select_neighbourhood( 
    DestroyHeuristic h, 
    int N, 
    const std::vector<std::vector<int>>& plans,
    const std::vector<int> non_disabled_agents,
    SharedEnvironment* env
  );


  float compute_soc(
    const std::vector<std::vector<int>>& plans,
    const std::vector<int> non_disabled_agents,
    SharedEnvironment* env
  );



  }