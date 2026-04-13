#pragma once

#include "Types.h"
#include <random>
#include "SharedEnv.h"
#include "pibt.h"
#include "instance.h"
#include "graph.h"

namespace CustomAlgo{

// struct Planner {
    


    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env);

    Instance* build_instance(const SharedEnvironment* env, std::vector<Path>* precomputed_paths = nullptr);
    void get_step_actions(SharedEnvironment* env, std::vector<Action>& actions);
    void update_start_locations();
    void update_goal_locations(const SharedEnvironment* env);
    State compute_next_state(const State& curr, Agent* agent, SharedEnvironment* env);
    bool is_valid_actions(const std::vector<State>& curr_states, const std::vector<Action>& actions, SharedEnvironment* env);

    Action get_action_from_states(const State & state, const State & next_state, bool consider_rotation = true);
    int get_next_location(const State& s, Action a, SharedEnvironment* env);
// };

}