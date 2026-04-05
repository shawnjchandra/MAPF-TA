#include "pibt.h"
#include "mapf_utils.h"
#include "heuristics.h"
namespace CustomAlgo {

    /*@notes
        INFO:
        - prev_decision = lokasi SEKARANG dari agen. Di set sekali untuk setiap timestep . Untuk cek swap conflict dan rekursif ke siapa
        - next_decision = lokasi SELANJUTNYA dari agen. Di set pas diawal timestep, dan dipakai / dimodifikasi selama rekursif
        - planner_state.decided = lokasi TUJUAN dan STATE (sudah sampai atau belum) dari agen. Updatenya setiap commit / step. Bertujuan untuk agen yang lagi rotasi, ga dikenain plan.
    */

    /**
     * @brief Inisialisasi dan running pibt (causal) sepanjang window horizon (w). Mengikuti implementasi pseudocode dengan pencatatan state, dan decision (lihat info ditas) berdasarkan kebutuhan dan untuk mempermudah implementasi.
     * 
     * @param non_disabled_agents 
     * @param env 
     * @return std::vector<std::vector<int>> 
     */
    std::vector<std::vector<int>> init_pibt_window (
        std::vector<int> non_disabled_agents,
        SharedEnvironment* env
    ){

        //Init plans
        std::vector<std::vector<int>> plans(env->num_of_agents);

        // State (loc, timestep, orient) sekarang dan selanjutnya
        std::vector<State> prev_states(env->num_of_agents);
        std::vector<State> next_states(env->num_of_agents, State(-1,-1,-1));

        // Untuk nandain ,agen mana yang ngambil lokasi ini untuk langkah selanjutnya (dan dipakai sekarang)
        std::vector<int> prev_decision(env->map.size(), -1);
        std::vector<int> next_decision(env->map.size(), -1);

        std::vector<bool> occupied(env->map.size());

        // Biar ga terlalu panjang 
        auto& ps = env->planner_state;

        for (int a : non_disabled_agents ) {
            plans[a].resize(env->horizon+1); //+1 karena posisi 0 langsung diisi sama lokasi terkini
            plans[a][0] = env->curr_states[a].location;
        
            prev_states[a] = env->curr_states[a];
            prev_decision[prev_states[a].location] = a;

            //Cek / Simpan tujuan 
            if (ps.decided[a].loc == -1) ps.decided[a].loc = prev_states[a].location;

            //Cek sampai tujuan, dan ubah state
            if (prev_states[a].location == ps.decided[a].loc) ps.decided[a].state = CustomAlgo::DONE::DONE;
        

            if (ps.decided[a].state == CustomAlgo::DONE::NOT_DONE) {
                next_decision[ps.decided[a].loc] = a;
                next_states[a] = State(ps.decided[a].loc ,-1 ,-1);
            }
        }

        std::vector<int>order (non_disabled_agents.begin(), non_disabled_agents.end());

        // 
        std::vector<int> goal_per_agents;
        for (int step = 1 ; step <= env->horizon ; step++ ) {

            //Reset per timestep
            prev_decision.assign(env->map.size(), -1);
            next_decision.assign(env->map.size(), -1);

            goal_per_agents.assign(env->num_of_agents, -1);
            for (int a : non_disabled_agents) {

                //Update goalnya
                goal_per_agents[a] = env->goal_locations[a].empty() ? prev_states[a].location : env->goal_locations[a].front().first;

                //Update priorities
                bool is_at_goal = (prev_states[a].location == goal_per_agents[a]);

                int loc = prev_states[a].location;
                int orient = prev_states[a].orientation;
                float gcm_weight = ps.gcm[loc][orient];
            
                //Cek sampai atau belum dan tambahin priority
                if (is_at_goal) ps.priorities[a] = ps.priorities_base[a];
                else ps.priorities[a] += 1; 

                //Lokasi deadend
                if(!env->goal_locations[a].empty() && env->degree_map[prev_states[a].location] == 1) {
                    ps.priorities[a] += 10;
                }

                next_states[a] = State(-1,-1,-1);
                prev_decision[prev_states[a].location] = a;
                if (ps.decided[a].state == CustomAlgo::DONE::NOT_DONE) {
                    next_decision[ps.decided[a].loc] = a;
                    next_states[a] = State(ps.decided[a].loc ,-1,-1);
                }
            }

            //Sort descending berdasarkan priorities
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                return ps.priorities[a] > ps.priorities[b]; 
            });

            // Jalanin PIBT nya
            for (int a : order) {
                // Skip kalau masih rotasi
                if (ps.decided[a].state == CustomAlgo::DONE::NOT_DONE) continue;

                // Kalau belum punya lokasi selanjutnya
                if (next_states[a].location == -1) {
                    pibt(a, -1, prev_states, next_states, prev_decision, next_decision, occupied, goal_per_agents, env);
                }

            }

            for (int a : order) {
                
                if (next_states[a].location >= 0) {
                    
                    // Kalo agen a jalan, reset lokasi a di decision
                    next_decision[next_states[a].location] = -1;

                    // Update decide untuk agen a (status jalan)
                    ps.decided[a] = {
                        next_states[a].location,
                        CustomAlgo::DONE::NOT_DONE
                    };

                    plans[a][step] = next_states[a].location;
                    
                    prev_states[a] = next_states[a];
                }else {
                     plans[a][step] = prev_states[a].location;
                 }

            }


        }
        return plans;
    }

    /**
     * @brief Fungsi Rekursif PIBT. Mengikuti implementasi pada pseudocode
     * 
     * @param curr_id 
     * @param higher_id 
     * @param prev_states 
     * @param next_states 
     * @param prev_decision 
     * @param next_decision 
     * @param occupied 
     * @param goal_per_agents 
     * @param env 
     * @return true 
     * @return false 
     */
    bool pibt(
            int curr_id,
            int higher_id,
            std::vector<State>& prev_states,
            std::vector<State>& next_states,
            std::vector<int>& prev_decision,
            std::vector<int>& next_decision,
            std::vector<bool>& occupied,
            const std::vector<int> goal_per_agents,
            // const WindowedInstance& instance,
            SharedEnvironment* env
          ) {
            //Cancel PIBT kalau sebelumnya udah dapat rencana gerak
            assert(next_states[curr_id].location == -1);

            int prev_loc = prev_states[curr_id].location;
            int prev_orient = prev_states[curr_id].orientation;
            int goal_agent = goal_per_agents[curr_id];

            std::vector<int> neighbors;
            // neighbors = CustomAlgo::getNeighborLocs(env->)
            CustomAlgo::getNeighborLocs(&(env->ns), neighbors, prev_loc);

            std::vector<PIBT_C> candidates;
            int weighted_h;
            for (int nb = 0 ; nb < 4 ; nb++) {
                int cand_loc = neighbors[nb];

                
                if (cand_loc < 0 || cand_loc > env->map.size() || env->map[cand_loc] == 1) continue;

                if (occupied[cand_loc]) continue;  //  Vertex conflict
                if (next_decision[cand_loc] != -1) continue; // Swap conflict
                if (higher_id != -1 && prev_decision[cand_loc] == higher_id) continue; // Conflict dengan agen yang melakukan priority inheritance 

                int nb_orientation = getOrientationBetween(prev_loc, cand_loc);

                int h = CustomAlgo::query_heuristic(env, cand_loc, nb_orientation, goal_agent);

                float gcm_w = env->planner_state.gcm[cand_loc][nb_orientation];

                weighted_h = (int) (gcm_w * h);

                candidates.push_back(PIBT_C(cand_loc, weighted_h, nb_orientation, rand()));
                
            }
            
            //Wait
            int h = CustomAlgo::query_heuristic(env, prev_loc, prev_orient, goal_agent);
            float gcm_w = env->planner_state.gcm[prev_loc][prev_orient];
            weighted_h = (int) (gcm_w * h);
            candidates.push_back(PIBT_C(prev_loc, weighted_h, prev_orient, rand()));
            
            // TODO: harus cek lagi kalau neighbors beneran sequential / increasing order
            std::sort(candidates.begin(), candidates.end(), [&](PIBT_C a, PIBT_C b){
                // Opsi 1 (Utama) : utamain yang heuristik lebih rendah
                if (a.heuristic != b.heuristic) return a.heuristic < b.heuristic;
                
                // Opsi 2 : Utamain yang satu orientasi
                bool is_a_forward = (a.location == neighbors[prev_orient]);
                bool is_b_forward = (b.location == neighbors[prev_orient]);
                if (is_a_forward != is_b_forward) return is_a_forward > is_b_forward;

                // Opsi 3 : random tie breaker (yang lebih kecil)
                return a.tie_breaker < b.tie_breaker;
            }); 
        
            for (auto& cand : candidates) {
                next_states[curr_id] = State(cand.location, -1, cand.orientation);
                next_decision[cand.location] = curr_id;

                // Kalau ada agen pada lokasi, dan belum pindah (rencanin)
                if (prev_decision[cand.location] != -1 && next_states[prev_decision[cand.location]].location != -1) {
                    int lower_id = prev_decision[cand.location];
                    if (!pibt(lower_id, curr_id, prev_states, next_states, prev_decision, next_decision, occupied, goal_per_agents, env)) {

                        //Backtrack kalau gagal 
                        next_states[curr_id] = State(-1,-1,-1); // Untuk validasi assert
                        next_decision[cand.location] = -1;
                        continue;
                    }
                }
                return true;
            }

            next_states[curr_id] = State(prev_loc, -1, prev_orient);
            next_decision[prev_loc] = curr_id;
            return false;
        }

}
