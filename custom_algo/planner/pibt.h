#pragma once

#include "SharedEnv.h"
#include "States.h"
#include <assert.h>


/*
hpa_pibt

get neighbours of curr location (4-connected + wait)
for each neighbour:
  h = query_heuristic(hpa, neighbour, orient, goal)
  weight h by gcm[neighbour][orient]
sort by weighted h ascending
for each candidate in sorted order:
  skip if occupied / already decided / swap conflict
  reserve it
  if another agent is there and hasn't moved:
    recurse hpa_pibt on that agent
    if recursion fails: backtrack, try next candidate
  return true
fall through: stay in place, return false

*/
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