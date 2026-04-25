#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"
#include "solver.h"

#include <vector>
#include <random>
#include <chrono>

namespace CustomAlgo {

    using TimePoint = std::chrono::steady_clock::time_point;

    enum class DESTROYHEURISTIC { RANDOM, AGENT_BASED, MAP_BASED, COUNT };

    /*
        Pindahin dari SharedEnv ke sini

        Simpen plan sebanyak w+1
        plan[timestep][agent]
    */
    struct LNSPlan {
        vector<vector<State>> plan;
    };

    struct LNSState {
        // LNSPlan
        LNSPlan lns_plan;
        bool has_cached_plan = false;
        LNSPlan cached_plan;

        int best_soc = INT_MAX;
        std::array<float , (int)DESTROYHEURISTIC::COUNT> weights;
    
        float GAMMA = 0.01f;
        float W_MIN = 0.01f;

        void init_weights() {
            weights.fill(1.0f);
        }

        DESTROYHEURISTIC select_heuristic(std::mt19937& rng) {
            float total = 0.0f;
            for (float w : weights) total += w;

            //Roulette wheel selection
            std::uniform_real_distribution<float> dist(0.0f, total);
            float pick = dist(rng);

            float acc = 0.0f;
            for ( int i = 0 ; i < (int)DESTROYHEURISTIC::COUNT ; i++) {
                acc += weights[i];
                if (pick <= acc) return static_cast<DESTROYHEURISTIC>(i);
            }

            return DESTROYHEURISTIC::RANDOM;
        }

        void update_weight(DESTROYHEURISTIC h, int increase_val ) {
            float& w = weights[(int)h];
            if (increase_val > 0 ) w = GAMMA * increase_val + (1.0f - GAMMA) * w;
            else w = (1.0f - GAMMA)* w;

            w = max(w, W_MIN);
        }
        
        LNSState () {
            init_weights();
        };
    };

    class WPPLSolver : public Solver {
        public:
            void initialize(int preprocess_time_limit, SharedEnvironment* env);
            void plan (int time_limit, vector<Action>& actions, SharedEnvironment* env);

        //Copy dari PIBT
        private:
            vector<double> p;
            vector<double> p_copy;
            vector<int> ids;
            vector<int> goal_per_agent;
            std::mt19937 mt1;
            
            
            // Simulasi buat Windowed PIBT
            std::vector<int> sim_prev_dec;
            std::vector<int> sim_dec;
            std::vector<bool> sim_occupied;
            
            // Ekstrak action
            std::vector<DCR> decided;
            std::vector<bool> checked;
            std::vector<int> prev_decision;

            LNSState lns_state;

            LNSState run_pibt_window(SharedEnvironment* env, int w);
            void run_iterations(LNSState& lns_state, SharedEnvironment* env, TimePoint deadline);

            std::vector<int> destroy_agents(DESTROYHEURISTIC h, LNSState& lns_state, int N, std::mt19937& rng, SharedEnvironment* env);

            //Method Destroy Heuristic (DROP-LNS)
            std::vector<int> destroy_random(int N, std::mt19937& rng);
            std::vector<int> destroy_agent_based(LNSState& lns_state, int N, SharedEnvironment* env);
            std::vector<int> destroy_map_based(LNSState& lns_state,int N, SharedEnvironment* env, std::mt19937& rng);
            

            // Repair (Bagian D&R)
            void repair(LNSState& lns_state, std::vector<int>& destroyed_agents, SharedEnvironment* env);
            
            // Destroyed agents path (untuk simpen dan restore)
            std::vector<std::vector<State>> save_paths(LNSState& lns_state,std::vector<int>& destroyed_agents, int w);
            void restore_paths(LNSState& lns_state, std::vector<int>& destroyed_agents, const std::vector<std::vector<State>>& saved, int w);

            //Utilnya
            int compute_soc(LNSState& lns_state, int w, SharedEnvironment* env);
            void shift_plan(LNSState& lns_state, int h, SharedEnvironment* env);


            //Nge sync antara warmup sama kondisi saat ini
            void sync_condition(LNSState& lns_state, SharedEnvironment* env);
    };
}