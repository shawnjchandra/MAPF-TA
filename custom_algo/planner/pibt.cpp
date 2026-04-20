#include "pibt.h"
#include "mapf_utils.h"
#include "heuristics.h"

namespace CustomAlgo {

    /*@notes
        INFO:
        - prev_decision = lokasi SEKARANG dari agen. Di set sekali untuk setiap timestep.
          Untuk cek swap conflict dan rekursif ke siapa.
        - decision = lokasi SELANJUTNYA dari agen. Di set pas diawal timestep, dan
          dipakai / dimodifikasi selama rekursif.
        - planner_state.decided = lokasi TUJUAN dan STATE (sudah sampai atau belum)
          dari agen. Updatenya setiap commit / step. Bertujuan untuk agen yang lagi
          rotasi, ga dikenain plan.

        PORTING NOTES (dari causalPIBT original framework):
        - TrajLNS dihapus; heuristik sekarang via query_heuristic() + GCM weight
          dari env->planner_state.gcm (sama seperti sebelumnya di versi kamu).
        - get_gp_h() diganti langsung dengan query_heuristic() + GCM weighting.
        - occupied[] sekarang di-set untuk curr_id kalau higher_id == -1 (agen pertama
          di chain), persis seperti original causalPIBT.
        - Backtrack sekarang eksplisit: reset next_states[curr_id] dan decision[],
          sama persis dengan original (kamu sebelumnya comment-out bagian ini).
        - Bug fix: `decision[cand.location != -1]` → `decision[cand.location] != -1`
        - validateMove di-skip untuk wait candidate (prev_loc == cand.location),
          sama seperti original yang tidak assert validateMove untuk wait.
    */

    /**
     * @brief Fungsi rekursif causal PIBT. Di-port dari causalPIBT() original framework,
     *        tanpa TrajLNS. Menggunakan query_heuristic() + GCM dari planner_state.
     *
     * @param curr_id       agen yang sedang diproses
     * @param higher_id     agen yang memanggil (priority lebih tinggi), -1 jika root
     * @param prev_states   state agen di timestep sekarang
     * @param next_states   state agen di timestep selanjutnya (diisi oleh PIBT)
     * @param prev_decision peta lokasi → agen yang ada di sana sekarang
     * @param decision      peta lokasi → agen yang akan menuju ke sana
     * @param occupied      lokasi yang dikunci (anti-cycle untuk agen root)
     * @param goal_per_agents goal location per agen
     * @param env           shared environment
     * @return true  jika berhasil menemukan move
     * @return false jika gagal (caller harus backtrack)
     */
    bool pibt(
        int curr_id,
        int higher_id,
        std::vector<State>& prev_states,
        std::vector<State>& next_states,
        std::vector<int>& prev_decision,
        std::vector<int>& decision,
        std::vector<bool>& occupied,
        const std::vector<int> goal_per_agents,
        SharedEnvironment* env,
        std::vector<double> p
    ) {
        assert(next_states[curr_id].location == -1);

        int prev_loc    = prev_states[curr_id].location;
        int prev_orient = prev_states[curr_id].orientation;
        int goal_agent  = goal_per_agents[curr_id];

        // Arah maju dari orient (0,3)
        int forward_locs[4] = {
            prev_loc + 1,
            prev_loc + env->cols,
            prev_loc - 1,
            prev_loc - env->cols
        };
        int orien_next_v = forward_locs[prev_orient];

        assert(prev_loc >= 0 && prev_loc < (int)env->map.size());

        // kunci lokasi awalnya supaya tidak ada cycle.
        if (higher_id == -1) {
            occupied[prev_loc] = true;
        }

        // Generate candidate nya
        std::vector<int> neighbors;
        std::vector<PIBT_C> candidates;
        CustomAlgo::getNeighborLocs(&(env->ns), neighbors, prev_loc);

        for (auto& nb : neighbors) {
            assert(validateMove(prev_loc, nb, env));

            int nb_orient = getOrientationBetween(prev_loc, nb);
            int h         = query_heuristic(env, nb, nb_orient, goal_agent);
            int wait_penalty = env->planner_state.wait_map[nb][nb_orient];
            float gcm_w   = env->planner_state.gcm[nb][nb_orient];
            
            int hw_penalty = 0;
            if (env->hpa_h.hw.r_e_hw.count({prev_loc, nb_orient})) hw_penalty = env->c_penalty;
            
            int weighted_h = (int)(gcm_w * h) + wait_penalty + hw_penalty;

            candidates.push_back(PIBT_C(nb, weighted_h, nb_orient, rand()));
        }

        // Wait candidate
        
        int h          = query_heuristic(env, prev_loc, prev_orient, goal_agent);
        float gcm_w    = env->planner_state.gcm[prev_loc][prev_orient];
        int wait_penalty = env->planner_state.wait_map[prev_loc][prev_orient];
        int weighted_h = (int)(gcm_w * h) + wait_penalty;

        if (prev_loc == goal_agent) weighted_h += 1;
        if (p[curr_id] > 5.0) weighted_h += (int)(p[curr_id]);
        candidates.push_back(PIBT_C(prev_loc, weighted_h, prev_orient, rand()));
        

        // Sort heuristik
        std::sort(candidates.begin(), candidates.end(), [&](const PIBT_C& a, const PIBT_C& b) {
            if (a.heuristic == b.heuristic) {
                if (a.location == orien_next_v && b.location != orien_next_v) return true;
                if (a.location != orien_next_v && b.location == orien_next_v) return false;
                return a.tie_breaker < b.tie_breaker;
            }
            return a.heuristic < b.heuristic;
        });

        //Iterasi candidates 
        for (auto& cand : candidates) {

            // Skip kalau lokasi dikunci, kecuali agen root boleh wait di tempatnya sendiri
            if (occupied[cand.location] &&
                !(higher_id == -1 && cand.location == prev_loc))
                continue;

            // Wait tidak perlu validateMove (sudah pasti valid)
            if (cand.location != prev_loc) {
                assert(validateMove(prev_loc, cand.location, env));
            }

            // Kalau seandaikan cand ga valid
            if (cand.location == -1) continue;

            // Vertex conflict
            if (decision[cand.location] != -1) continue;

            // Swap conflict
            if (higher_id != -1 && prev_decision[cand.location] == higher_id) continue;

            // Ambil lokasi cand
            next_states[curr_id] = State(cand.location, -1, cand.orientation);
            decision[cand.location] = curr_id;

            // Kalau ada agen lain di cand.location dan belum punya rencana,  inheritance
            if (prev_decision[cand.location] != -1 &&
                next_states[prev_decision[cand.location]].location == -1) {
                int lower_id = prev_decision[cand.location];
                if (!pibt(lower_id, curr_id,
                          prev_states, next_states,
                          prev_decision, decision,
                          occupied, goal_per_agents, env, p)) {
                    // Backtrack
                    next_states[curr_id] = State(-1, -1, -1);
                    decision[cand.location] = -1;
                    continue;
                }
            }

            return true;
        }

        // Semua kandidat gagal, wait paksa di tempat
        next_states[curr_id]  = State(prev_loc, -1, prev_orient);
        decision[prev_loc]    = curr_id;

        for (int orient = 0 ; orient < 4 ; orient++) {
            env->planner_state.wait_map[prev_loc][orient] +=1;
        }
        return false;
    }

    //Ambil dari framework
    bool moveCheck(
        int id,
        std::vector<bool>& checked,
        std::vector<DCR>& decided,
        std::vector<Action>& actions,
        std::vector<int>& prev_decision)
    {
        if (checked.at(id) && actions.at(id) == Action::FW)
            return true;
        checked.at(id) = true;

        if (actions.at(id) != Action::FW)
            return false;

        int target = decided.at(id).loc;
        assert(target != -1);

        int na = prev_decision[target];
        if (na == -1)
            return true;

        if (moveCheck(na, checked, decided, actions, prev_decision))
            return true;

        actions.at(id) = Action::W;
        return false;
    }

}