#include "pibt.h"

namespace CustomAlgo {



    PIBT::PIBT(std::mt19937* _MT, SharedEnvironment* _env, Instance* ins, bool use_swap):
        MT(_MT), env(_env), ins(ins), use_swap(use_swap) {};

    void PIBT::initialize(Graph* G) {
        int num_agents   = env->num_of_agents;

        occupied_now.resize(G->V.size(), nullptr);   // size by id range
        occupied_next.resize(G->V.size(), nullptr);

        A.resize(num_agents);
        for (int i = 0; i < num_agents; ++i)
            A[i] = new Agent(i);

        C_next.resize(num_agents);
        tie_breakers.resize(G->V.size(), 0.0f);
    }

    int PIBT::get_o_dist(int o1, int o2) {
        return std::min((o2-o1+4)%4,(o1-o2+4)%4);
    }

    float PIBT::get_cost_move(int pst, int ped) {
        int diff = ped - pst;
        int orient;
        
        if      (diff ==  1)           orient = 0; // east
        else if (diff ==  env->cols) orient = 1; // south
        else if (diff == -1)           orient = 2; // west
        else if (diff == -env->cols) orient = 3; // north
        else if (diff ==  0)           orient = 4; // stay
        else {
            std::cerr << "invalid move: " << pst << " " << ped << std::endl;
            exit(-1);
        }
        
        return env->planner_state.gcm[pst][orient];
    }

    bool PIBT::funcPIBT(Agent* ai, const Config& ctx) {
        const auto i = ai->id;
        // cout << "funcPIBT agent " << i << endl;
        
        if (ai->v_now == nullptr) {
            std::cerr << "v_now is null for agent " << i << endl;
            return false;
        }
        
        const auto K = ai->v_now->neighbor.size();
        // cout << "K=" << K << endl;

        for (auto k = 0; k < K; ++k) {
            auto u = ai->v_now->neighbor[k];
            if (u == nullptr) {
                std::cerr << "null neighbor " << k << " for agent " << i << endl;
                continue;
            }
            C_next[i][k] = u;
            if (MT != nullptr) tie_breakers[u->id] = get_random_float(MT);
        }
        // cout << "neighbors filled" << endl;

        C_next[i][K] = ai->v_now;
        tie_breakers[ai->v_now->id] = get_random_float(MT);
        // cout << "tie breakers set" << endl;

        int o0 = ctx.orients[i];
        // cout << "o0=" << o0 << endl;

        auto& ps = env->planner_state;
        // cout << "planner_state accessed" << endl;

        float cost_rot = ps.gcm[ai->v_now->index][4];
        // cout << "cost_rot=" << cost_rot << endl;

    //     cout << "gcm[328]: E=" << ps.gcm[328][0] 
    //  << " S=" << ps.gcm[328][1]
    //  << " W=" << ps.gcm[328][2] 
    //  << " N=" << ps.gcm[328][3]
    //  << " stay=" << ps.gcm[328][4] << endl;

        std::vector<std::tuple<int,float,float,float,Vertex*>> scores;
        // cout << "scores vector created" << endl;

        for (int k = 0; k <= K; ++k) {
            auto& v = C_next[i][k];
            // cout << "scoring candidate k=" << k << " v=" << (v == nullptr ? -1 : (int)v->index) << endl;

            int o1 = get_neighbor_orientation(ai->v_now->index, v->index, env->cols, o0);
            // cout << "o1=" << o1 << endl;

            int o1_dist = get_o_dist(o0, o1);
            // cout << "o1_dist=" << o1_dist << endl;

            float cost1 = (float)o1_dist * cost_rot + get_cost_move(ai->v_now->index, v->index);
            // cout << "cost1=" << cost1 << endl;

            float d1 = query_heuristic(env, v->index, o1, env->goal_locations[i].front().first) + cost1;
            // cout << "d1=" << d1 << endl;
            if (ai->id == 0) {
            int raw_h = query_heuristic(env, v->index, o1, env->goal_locations[i].front().first);
            
            cout << "  k=" << k << " v=" << v->index 
                << " o1=" << o1 << " o1_dist=" << o1_dist
                << " cost1=" << cost1 << " raw_h=" << raw_h << " d1=" << d1 << endl;
        }

            int pre_d1 = 1;
            if (ins->precomputed_paths != nullptr) {
                auto& path = (*ins->precomputed_paths)[i];
                int j = ctx.timestep;

                if (j < (int)path.size() - 1
                    && path[j].location == ai->v_now->index
                    && path[j].orientation == o0) {

                    if (path[j+1].orientation == o1) {
                        pre_d1 = 0;
                    }
                }
            }
            // cout << "pre_d1=" << pre_d1 << endl;

            scores.emplace_back(pre_d1, d1, (float)o1_dist, tie_breakers[v->id], v);
            // cout << "score emplaced k=" << k << endl;
        }
        // cout << "all scores computed" << endl;

        std::sort(scores.begin(), scores.end(), [&](const auto& a, const auto& b) {

            if (std::get<0>(a) != std::get<0>(b)) return std::get<0>(a) < std::get<0>(b);
            // cout << "if 1 bablas" << std::endl;            
            if (std::get<1>(a) != std::get<1>(b)) return std::get<1>(a) < std::get<1>(b);
            
            // cout << "if 2 bablas" << std::endl;            
            if (std::get<2>(a) != std::get<2>(b)) return std::get<2>(a) < std::get<2>(b);
            // cout << "if 3 bablas" << std::endl;            
            return std::get<3>(a) < std::get<3>(b); // tie_breaker
        });
        // cout << "scores sorted" << endl;

        for (int k = 0; k <= K; ++k) {
            C_next[i][k] = std::get<4>(scores[k]);
        }
        // cout << "C_next reordered" << endl;
        if (ai->id == 0) {
            cout << "agent 0 scores:" << endl;
            for (auto& s : scores) {
                cout << "  v=" << std::get<4>(s)->index 
                    << " pre_d1=" << std::get<0>(s)
                    << " d1=" << std::get<1>(s)
                    << " o_dist=" << std::get<2>(s) 
                    << " tie_breaker=" << std::get<3>(s) << endl;
            }
            cout << "agent 0 loc=" << env->curr_states[0].location 
                << " goal=" << env->goal_locations[0][0].first
                << " heuristic=" << query_heuristic(env, 361, 0, env->goal_locations[0][0].first)
                << endl;
        }

        for (int k = 0; k < K + 1; ++k) {
            auto u = C_next[i][k];
            // cout << "trying candidate k=" << k << " u=" << (u == nullptr ? -1 : (int)u->index) << endl;

            if (occupied_next[u->id] != nullptr) {
                // cout << "vertex conflict at k=" << k << endl;
                continue;
            }

            auto& ak = occupied_now[u->id];
            // cout << "ak=" << (ak == nullptr ? "null" : std::to_string(ak->id)) << endl;

            if (ak != nullptr && ak->v_next == ai->v_now) {
                // cout << "swap conflict at k=" << k << endl;
                continue;
            }

            occupied_next[u->id] = ai;
            ai->v_next = u;
            if (u->index == ai->v_now->index) {
                ai->o_next = ctx.orients[i];
            } else {
                int diff = u->index - ai->v_now->index;
                if      (diff ==  1)           ai->o_next = 0;
                else if (diff ==  env->cols)   ai->o_next = 1;
                else if (diff == -1)           ai->o_next = 2;
                else if (diff == -env->cols)   ai->o_next = 3;
            }
            // cout << "v_next set to " << u->index << " o_next=" << ai->o_next << endl;

            if (ak != nullptr && ak != ai && ak->v_next == nullptr && !funcPIBT(ak, ctx)) {
                // cout << "priority inheritance failed at k=" << k << endl;
                continue;
            }

            // cout << "funcPIBT agent " << i << " success" << endl;
            return true;
        }

        // fallback
        occupied_next[ai->v_now->id] = ai;
        ai->v_next = ai->v_now;
        ai->o_next = ctx.orients[i];
        // cout << "agent " << i << " forced to wait" << endl;
        return false;
    }

    bool PIBT::get_new_config(const Config& ctx)
    {
        // setup cache
        for (auto a : A) {
            // clear previous cache
            if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
                occupied_now[a->v_now->id] = nullptr;
            }
            if (a->v_next != nullptr) {
                occupied_next[a->v_next->id] = nullptr;
                a->v_next = nullptr;
            }

            // set occupied now — was H->C.locs[a->id]
            a->v_now = ctx.locs[a->id];
            occupied_now[a->v_now->id] = a;
        }

        std::vector<float> h_vals(A.size());
        for (uint i = 0; i < A.size(); ++i) {
            h_vals[i] = (float)query_heuristic(env, ctx.locs[i]->index, ctx.orients[i], env->goal_locations[i].front().first);
            total_count++;
            if (h_vals[i] >= INTERVAL_MAX) invalid_count++;
        }

        std::vector<uint> order(A.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](uint i, uint j){
            return h_vals[i] < h_vals[j];
        });
         cout << "heuristic: " << invalid_count << "/" << total_count << " unreachable" << endl;

        // LNode constraint block is completely dropped —
        // no LaCAM2 low-level constraints needed, PIBT handles conflicts itself

        // perform PIBT — was H->order
        for (auto k : order) {
            auto a = A[k];
            if (a->v_next == nullptr && !funcPIBT(a, ctx)) {
                return false; // planning failure
            }
        }
        return true;
    }

}