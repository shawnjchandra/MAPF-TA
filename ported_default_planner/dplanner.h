#pragma once

#include "dTypes.h"
#include "TrajLNS.h"
#include <random>

#include "solver.h"


namespace CustomAlgo{

    class PIBTTrajSolver: public Solver {

        void initialize(int preprocess_time_limit, SharedEnvironment* env);
    
        void plan(int time_limit,vector<Action> & actions,  SharedEnvironment* env);
    };


}