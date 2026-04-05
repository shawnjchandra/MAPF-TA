#pragma once

#include "SharedEnv.h"
#include "mapf_utils.h"
#include "Types.h"

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