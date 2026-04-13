#pragma once

#include "Types.h"
#include "graph.h"

namespace CustomAlgo {
    struct AgentInfo {
        int goal_location;
        float elapsed;
        float tie_breaker;
        int id;
        int stuck_order;
        bool disabled;

        AgentInfo():id(-1),goal_location(-1),elapsed(-1),tie_breaker(-1), stuck_order(0), disabled(false) {};
    };

 

    struct Instance {
        const Graph & G;
        Config starts;
        Config goals;
        uint N;
        std::vector<AgentInfo>& agent_infos;  // std:: prefix
        int planning_window = -1;
        std::vector<::Path>* precomputed_paths;

        Instance(
            const Graph & G,
            const std::vector<std::pair<uint,int>>& start_indexes,  // const ref
            const std::vector<std::pair<uint,int>>& goal_indexes,   // const ref
            std::vector<AgentInfo>& agent_infos,
            int planning_window = -1,
            std::vector<::Path>* precomputed_paths = nullptr
        );
        ~Instance() {}
        
        void set_starts_and_goals(std::vector<::State>* starts, std::vector<::State>* goals);
        bool is_valid(const int verbose = 0) const;
        
    };
}