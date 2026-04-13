#pragma once

#include "SharedEnv.h"
#include "mapf_utils.h"
#include "Types.h"
#include "heuristics.h"
#include "instance.h"
#include "graph.h"

//Ambil PIBT dari MAPFLRR2023 - Team Pikachu,
namespace CustomAlgo {

    
    struct Agent
    {
        const int id;
        Vertex* v_now;
        Vertex* v_next;
        int o_next;
        
        Agent(int _id) : id(_id) , v_now(nullptr), v_next(nullptr), o_next(-1) {}
    };
    
    using Agents = std::vector<Agent*>;
    
    struct PIBT {
        
        std::mt19937* MT;
        SharedEnvironment* env;
        Instance* ins;

        std::vector<std::array<Vertex*, 5>> C_next; // Next_locs
        std::vector<float> tie_breakers;    //Random vals

        // std::vector<Vertex> vertex_pool; // owns all vertices
        // std::vector<Vertex*> vertex_map; // index -> Vertex*
        int invalid_count = 0;
        int total_count = 0;

        bool use_swap;

        Agents A;
        Agents occupied_now;
        Agents occupied_next;

        PIBT(std::mt19937* _MT, SharedEnvironment* _env, Instance* ins, bool use_swap = false);

        void initialize(Graph* G);
        int get_o_dist(int o1, int o2);
        float get_cost_move(int pst, int ped);
        bool funcPIBT( Agent* ai, const Config& ctx);
        bool get_new_config(const Config& ctx);
    };
}