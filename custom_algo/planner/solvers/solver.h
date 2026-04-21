#pragma once

#include "SharedEnv.h"
#include <vector>

namespace CustomAlgo {
    class Solver {
        public:
        virtual ~Solver();
        virtual void initialize(int preprocess_time_limit, SharedEnvironment* env) = 0;
        virtual void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env) = 0;
    };
}