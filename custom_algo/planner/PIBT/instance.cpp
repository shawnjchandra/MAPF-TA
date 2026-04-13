#include "instance.h"
#include "SharedEnv.h"
#include "mapf_utils.h"

namespace CustomAlgo {

    Instance::Instance(
        const Graph & G,
        const std::vector<std::pair<uint,int> >& start_indexes,
        const std::vector<std::pair<uint,int> >& goal_indexes,
        std::vector<AgentInfo>& agent_infos,
        int planning_window,
        std::vector<::Path> * _precomputed_paths
        ):
        G(G),
        starts(agent_infos.size()),
        goals(agent_infos.size()),
        N(agent_infos.size()),
        agent_infos(agent_infos),
        planning_window(planning_window), 
        precomputed_paths(_precomputed_paths) {
    // for (auto k : start_indexes) starts.push_back(G.U[k]);
    // for (auto k : goal_indexes) goals.push_back(G.U[k]);

    for (int i=0;i<N;++i) {
        starts.locs[i]=G.U[start_indexes[i].first];
        starts.orients[i]=start_indexes[i].second;
        goals.locs[i]=G.U[goal_indexes[i].first];
        goals.orients[i]=goal_indexes[i].second;
    }

    }

    void Instance::set_starts_and_goals(
        std::vector<::State>* start_states,
        std::vector<::State>* goal_states
    ) {
        // no-op for locs since we don't have Graph
        // orients come from State
        for (uint i = 0; i < N; ++i) {
            starts.orients[i] = (*start_states)[i].orientation;
            goals.orients[i]  = (*goal_states)[i].orientation;
        }
    }

    bool Instance::is_valid(const int verbose) const {
        if (starts.size() != N || goals.size() != N) {
            if (verbose) std::cerr << "size mismatch" << std::endl;
            return false;
        }
        return true;
    }

    
}