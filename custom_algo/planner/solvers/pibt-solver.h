#pragma once

#include "SharedEnv.h"
#include "States.h"
#include <assert.h>
#include "solver.h"

#include "pibt.h"
#include "const.h"
#include "guidance_cost_map.h"
#include "mapf_utils.h"

namespace CustomAlgo {

    class PIBTSolver : public Solver {

        public:
            void initialize(int preprocess_time_limit, SharedEnvironment* env) ;
            void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env ) ;

        private:
            std::vector<int> decision;
            std::vector<int> prev_decision;
            std::vector<double> p;
            std::vector<double> p_copy;
            std::vector<State> next_states;
            std::vector<State> prev_states;
            std::vector<bool> occupied;
            std::vector<DCR> decided;
            std::vector<bool> checked;
            std::vector<int> ids;
            std::vector<int> goal_per_agent;
            std::mt19937 mt1;

    };
}