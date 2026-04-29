#include "planner.h"
#include "solver.h"
#include "pibt-solver.h"
#include "wppl.h"

#include "dplanner.h"

namespace CustomAlgo {

    static std::unique_ptr<Solver> solver;

    void planner_initialize(int preprocess_time_limit, SharedEnvironment* env) {
        const string& mode = env->mode;

        if (mode == "wppl") {
            solver = std::make_unique<WPPLSolver>();
        } else if (mode == "pibt") {    
            solver = std::make_unique<PIBTSolver>();

        } else if (mode == "traj") {//TRAJLNS kalau jadi
            solver = std::make_unique<PIBTTrajSolver>();
            cout << "Solver ported traj " << std::endl;
        } else  { 
            
            //Sementara fallback ke pibt
            solver = std::make_unique<PIBTSolver>();
        }

        cout << "Mode : " << mode << std::endl;
        solver->initialize(preprocess_time_limit, env);
    }


    void planner_plan(int time_limit, vector<Action>& actions, SharedEnvironment* env) {
        solver->plan(time_limit, actions, env);
    }


    std::unordered_set<int> find_deadend_agents(SharedEnvironment* env) {
        std::unordered_set<int> disabled;
        for (int a = 0; a < env->num_of_agents; a++) {
            if (env->degree_map[env->curr_states[a].location] == 1)
                disabled.insert(a);
        }
        return disabled;
    }

} 