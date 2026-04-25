#include "wppl.h"
#include "pibt.h"
#include "mapf_utils.h"
#include "heuristics.h"
#include "guidance_cost_map.h"
#include "const.h"

namespace CustomAlgo {
    
    // Copy yang pibt 
    void WPPLSolver::initialize(int preprocess_time_limit, SharedEnvironment* env) {
         int map_size   = env->map.size();
        int agent_size = env->num_of_agents;
 
        assert(agent_size != 0);
 
        auto& ps = env->planner_state;
        ps.w = std::max(3, ps.w);
        ps.h = std::max(1, ps.h);
        env->horizon = ps.h;
 
        // GCM init (base penalty dari highway)
        ps.gcm.assign(map_size, {1.0f, 1.0f, 1.0f, 1.0f});
        for (auto& [key, val] : env->hpa_h.hw.r_e_hw) {
            int loc    = key.first;
            int orient = key.second;
            ps.gcm[loc][orient] = (float)env->c_penalty;
        }
 
        ps.wait_map.assign(map_size, {0, 0, 0, 0});
        ps.w_peak.resize(map_size);
        for (auto& wp : ps.w_peak) {
            wp.val      = 1.0f;
            wp.timestep = 0;
        }
 
        ps.gcm_freq   = std::max(ps.gcm_freq, 5);
        ps.w_baseline = std::max(ps.w_baseline, 1.0f);
        ps.k_base     = std::max(ps.k_base, 0.1f);
        ps.max_degree = 4;
 
        ps.current_plans.assign(agent_size, {});
 
        // PIBT state
        p.resize(agent_size);
        p_copy.resize(agent_size);
        ids.resize(agent_size);
        goal_per_agent.assign(agent_size, -1);
 
        // Simulation buffers
        int ms = map_size;
        sim_prev_dec.resize(ms, -1);
        sim_dec.resize(ms, -1);
        sim_occupied.resize(ms, false);
 
        // Rotation / action extraction
        decided.resize(agent_size, DCR({-1, DONE::DONE}));
        checked.resize(agent_size, false);
        prev_decision.resize(ms, -1);
 
        // Initialize priorities (randomized)
        std::iota(ids.begin(), ids.end(), 0);
        mt1.seed(0);
        srand(0);
        std::shuffle(ids.begin(), ids.end(), mt1);
        for (int i = 0; i < (int)ids.size(); i++) {
            p[ids[i]] = ((double)(ids.size() - i)) / ((double)(ids.size() + 1));
        }
        p_copy = p;

        LNSState lns_state = LNSState();
        lns_state.has_cached_plan = false;
    }

    //Utilnya

    /**
     * @brief Hitung SOC dari plan agen. Dari current location ke goal dengan query_heuristic
     * 
     * @param lns_state 
     * @param w 
     * @param env 
     * @return int 
     */
    int WPPLSolver::compute_soc(LNSState& lns_state, int w, SharedEnvironment* env) {
        int soc = 0;

        LNSPlan& lns_plan = lns_state.lns_plan;

        for (int a : ids) {
            auto& curr_state = lns_plan.plan[w][a];

            if ( curr_state.location >= 0 && curr_state.location < env->map.size() && curr_state.orientation >= 0 ) 
                soc += query_heuristic(env, curr_state.location, curr_state.orientation, goal_per_agent[a]);
        }

        return soc;
    }

    /**
     * @brief Majuin windowed plan kedepan sebanyak h (replanning period). Tapi, untuk implementasi ini, h nya fixed di 1.
     * 
     * @param lns_state 
     * @param h 
     * @param env 
     */
    void WPPLSolver::shift_plan(LNSState& lns_state, int h, SharedEnvironment* env) {
        int w = env->planner_state.w;
        int n = env->num_of_agents;

        if (lns_state.lns_plan.plan.size() <= h) {
            lns_state.has_cached_plan = false;
            return;
        }
 
        // Shift, plan[0] = plan[h], plan[1] = plan[h+1]
        // Sisa (w-h) steps disimpen, bagian yang kosong diisi dengan state terakhir
        std::vector<std::vector<State>> new_plan (w+1, std::vector<State>(n, State()));
        
        int remaining = lns_state.lns_plan.plan.size() - h;

        // t = 0 ->  (w-h)
        for (int t = 0 ; t < remaining && t <= w; t++) {
            new_plan[t] = lns_state.lns_plan.plan[t+h];
        }

        for (int t = remaining ; t <= w ; t++) {
            new_plan[t] = new_plan[remaining - 1];
        }

        lns_state.lns_plan.plan = new_plan;
    }
    
    /**
     * @brief Select metode destroy (Agen ,Map, atau RANDOM) dan kembalikan agen yang dipilih
     * 
     * @param h 
     * @param lns_state 
     * @param N 
     * @param rng 
     * @param env 
     * @return std::vector<int> 
     */
    std::vector<int> WPPLSolver::destroy_agents(DESTROYHEURISTIC h, LNSState& lns_state, int N, std::mt19937& rng, SharedEnvironment* env) {
        switch (h)
        {
        case DESTROYHEURISTIC::AGENT_BASED:
            return destroy_agent_based(lns_state, N, env);
        case DESTROYHEURISTIC::MAP_BASED:
            return destroy_map_based(lns_state,N, env, rng);
        case DESTROYHEURISTIC::RANDOM:
        default:  
            return destroy_random(N,rng);
        }
    }

    //Method Destroy Heuristic (DROP-LNS)
    /**
     * @brief Pilih N agen secara random
     * 
     * @param N 
     * @param rng 
     * @return std::vector<int> 
     */
    std::vector<int> WPPLSolver::destroy_random(int N, std::mt19937& rng) {
        int n = ids.size();

        std::vector<int> agents(n);
        
        std::iota(agents.begin(), agents.end(), 0 );
        std::shuffle(agents.begin(), agents.end(), rng);
    

        agents.resize(N);
        return agents;
    }

    /**
     * @brief Pilih N agen sesuai jumlah banyak wait dari setiap agen. Urutin secara menurun dan pilih top N
     * 
     * @param lns_state 
     * @param N 
     * @param env 
     * @return std::vector<int> 
     */
    std::vector<int> WPPLSolver::destroy_agent_based(LNSState& lns_state, int N, SharedEnvironment* env) {
        int n = env->num_of_agents;
        int w = env->planner_state.w;

        std::vector<std::pair<int,int>> wait_counts(n);

        for (int a = 0 ; a < n ; a++) {
            int wait = 0 ;
            for (int t = 0 ; t < w ; t++) {
                int loc1 = lns_state.lns_plan.plan[t][a].location;
                int loc2 = lns_state.lns_plan.plan[t+1][a].location;
            
                if (loc1 >= 0 && loc1 < env->map.size() && loc1 == loc2) wait++;
            }
            wait_counts[a] = {wait, a};

        }
        // Sort secara menurun sesuai jumlah wait
        //Pakai no capture
        sort(wait_counts.begin(), wait_counts.end(), [](auto& a, auto& b){
            return a.first > b.first;
        });

        std::vector<int> result;
        result.reserve(N);
        for (int i = 0 ; i < N && i < n ; i++) result.push_back(wait_counts[i].second);

        return result;
    }
    
    /**
     * @brief Pilih lokasi random, terus pilih N agen yang paling dekat 
     * 
     * @param N 
     * @param env 
     * @param rng 
     * @return std::vector<int> 
     */
    std::vector<int> WPPLSolver::destroy_map_based(LNSState& lns_state,int N, SharedEnvironment* env, std::mt19937& rng) {
        int n = env->num_of_agents;
        int map_size = env->map.size();

        // Ambil point random yang bukan wall
        std::uniform_int_distribution<int> locs_dist(0, map_size - 1);
        int focal = locs_dist(rng);
        for (int i = 0 ; i < 10 && env->map[focal] == 1 ; i++) focal = locs_dist(rng);
        
        // Ambil lokasi sekarang
        std::vector<std::pair<int,int>> dists(n);
        for (int a = 0 ; a < n ; a++ ){
            int curr_loc = env->curr_states[a].location;

            if ( lns_state.lns_plan.plan[0][a].location >= 0 )
                curr_loc = lns_state.lns_plan.plan[0][a].location;

            dists[a] = {
                manhattanDistance(curr_loc, focal, env),
                a
            };
        }

        //Sort berdasarkan yang terdekat
        std::sort(dists.begin(), dists.end(), [&](auto& a, auto& b ){
            return a.first < b.first;
        });

        std::vector<int> result;
        result.reserve(N);
        for (int i = 0 ; i < N && i < n ; i++) result.push_back(dists[i].second);

        return result;
    }

    /**
     * @brief Simpan plan original dari agen yang di destroy (sebagai fallback plan)
     * 
     * @param lns_state 
     * @param destroyed_agents 
     * @param w 
     * @return std::vector<std::vector<State>> 
     */
    std::vector<std::vector<State>> WPPLSolver::save_paths(LNSState& lns_state,std::vector<int>& destroyed_agents, int w) {
        std::vector<std::vector<State>> saved(destroyed_agents.size());

        for (int i = 0 ; i < destroyed_agents.size() ; i++) {
            int a = destroyed_agents[i];
            saved[i].resize(w+1);

            for (int t = 0 ; t <= w ; t++) {
                saved[i][t] = lns_state.lns_plan.plan[t][a];
            }
        }

        return saved;
    }

    /**
     * @brief Kembalikan plan original dari agen yang dipilih untuk di destroy
     * 
     * @param lns_state 
     * @param destroyed_agents 
     * @param saved 
     * @param w 
     */
    void WPPLSolver::restore_paths(LNSState& lns_state, std::vector<int>& destroyed_agents, const std::vector<std::vector<State>>& saved, int w){
        for (int i = 0 ; i < destroyed_agents.size(); i ++) {
            int a = destroyed_agents[i];

            for (int t = 0 ; t <= w ; t++) {
                lns_state.lns_plan.plan[t][a] = saved[i][t] ;
            }
        }
    }

    /**
     * @brief Method repair dari D&R. Jalanin simulasi PIBT untuk agen-agen yang dipilih untuk di destroy sebelumnya. Ditambah dengan fxd_dst untuk cek swap conflict antara agen obstacle dan yang di destroy
     * 
     * @param lns_state 
     * @param destroyed_agents 
     * @param env 
     * @return * void 
     */
    void WPPLSolver::repair(LNSState& lns_state, std::vector<int>& destroyed_agents, SharedEnvironment* env) {
        auto& ps = env->planner_state;
        int w = ps.w;
        int n = env->num_of_agents;
        int map_size = env->map.size();

        auto& lns_plan = lns_state.lns_plan;

        std::unordered_set<int> destroyed(destroyed_agents.begin(), destroyed_agents.end());

        // Tracking swap conflict
        std::vector<int> fxd_dst;
        fxd_dst.assign(map_size, -1);

        // Prep untuk windowed PIBT lagi
        for (int t = 0 ; t < w; t++) {
            std::vector<State>& prev_state = lns_plan.plan[t];
            std::vector<State>& next_state = lns_plan.plan[t+1];

            std::fill(sim_prev_dec.begin(), sim_prev_dec.end(), -1);
            std::fill(sim_dec.begin(), sim_dec.end(), -1);
            std::fill(sim_occupied.begin(), sim_occupied.end(), false);
            std::fill(fxd_dst.begin(), fxd_dst.end(), -1);


            for (int a = 0 ; a < n ; a++) {
                if (prev_state[a].location >= 0) sim_prev_dec[prev_state[a].location] = a;
            }

            /*  Reset state untuk yang destroyed_agents, karena hanya perlu replan mereka.
            *   Agen lainnya dibiarkan.
            */
            for (int a = 0 ; a < n ; a++) {
                if (destroyed.count(a)) {
                    next_state[a] = State();
                    continue;
                }

                // Agen yang tidak di destroy, di save lagi lokasi dan claim posisi t+1 nya
                int next_loc = next_state[a].location;
                if (next_loc >= 0) sim_dec[next_loc] = a;

                int curr_loc = prev_state[a].location;
                if (curr_loc >= 0 && next_loc >= 0 && curr_loc != next_loc) fxd_dst[curr_loc] = next_loc;
            }

            //Jalanin PIBT untuk destroy agent sesuai priority
            for (int id : ids) {
                if (!destroyed.count(id)) continue;
                if (next_state[id].location != -1) continue;

                pibt(id, -1 , prev_state, next_state, sim_prev_dec, sim_dec, sim_occupied, goal_per_agent, env, p);
            }

            for (int a : destroyed_agents) {
                int from = prev_state[a].location;
                int to = next_state[a].location;

                //Cek kalau ada swap conflict (tukeran posisi antara agen di from dan to)
                if (from >= 0 && to >= 0 && from != to && fxd_dst[to] == from) {
                    sim_dec[from] = a;
                    sim_dec[to] = -1;
                    next_state[a] = prev_state[a];
                }
            }
        }

    }

    /**
     * @brief Jalankan iterasi LNS. Awalnya coba untuk single thread (mastiin jalan) dan dilimitasi dengan hardcoded max_iter + deadline. Untuk multi-threaded, limitasi dibatasiin dengan deadline saja
     * 
     * @param lns_state 
     * @param env 
     * @param deadline 
     */
    void WPPLSolver::run_iterations(LNSState& lns_state, SharedEnvironment* env, TimePoint deadline) {

        // Single threaded

        // int w = env->planner_state.w;
        // int n = env->num_of_agents;
        // int N = n / 10; //Neighbourhood fixed di 10% dulu

        // int best_soc = compute_soc(lns_state, w, env);
        // std::mt19937 rng(0);

        // int max_iter = 200;
        // for (int iter = 0 ; iter < max_iter; iter++) {
        //     if (std::chrono::steady_clock::now() >= deadline) break;

        //     DESTROYHEURISTIC h = lns_state.select_heuristic(rng);
        //     auto destroyed_agents = destroy_agents(h, lns_state, N, rng, env);
        
        //     auto saved_paths = save_paths(lns_state,destroyed_agents, w);

        //     repair(lns_state, destroyed_agents, env);

        //     int new_soc = compute_soc(lns_state, w, env);
        //     int dif_soc = best_soc - new_soc;

        //     lns_state.update_weight(h, dif_soc);

        //     //Kalau lebih baik replace SOC, kalau tidak, kembalikan path yang lama
        //     if (dif_soc > 0) best_soc = new_soc;
        //     else restore_paths(lns_state,destroyed_agents, saved_paths, w);

        // }

        //Multi threaded
        int n = env->num_of_agents;
        int w = env->planner_state.w;
        int N = n / 10;
        
        std::mutex mtx;

        std::vector<std::thread> threads;
        for (int i = 0 ; i < env->m ; i++) {
            threads.emplace_back([&](){
                
                // arbitrary number (angka random), mastiin setiap thread beda
                std::mt19937 rng(i * 99);

                while (std::chrono::steady_clock::now() < deadline ){ 
                    LNSState local_copy;
                    
                    //copy lns
                    {
                        std::lock_guard<std::mutex> lk(mtx);
                        local_copy = lns_state;
                    }

                    DESTROYHEURISTIC h = local_copy.select_heuristic(rng);
                    auto destroyed_agents = destroy_agents(h, local_copy, N, rng, env);
                    repair(local_copy, destroyed_agents, env);

                    
                    int new_soc = compute_soc(lns_state, w, env);
                    int dif_soc = local_copy.best_soc - new_soc;
                    
                    //
                    {
                        std::lock_guard<std::mutex> lk(mtx);
                        lns_state.update_weight(h, dif_soc);

                        if (new_soc  < lns_state.best_soc) {
                            lns_state.best_soc = new_soc;
                            lns_state.lns_plan = local_copy.lns_plan;
                        }
                    }
                }
            });
        }

        for (auto& t : threads) t.join();
        
    }

    /**
     * @brief Sync atau resimulate step 0 ke 1 dari warmup. Karena bisa aja kerja planner dengan yang direncanakan beda. Bisa conflict jika tidak.
     * 
     * @param lns_state 
     * @param env 
     */
    void WPPLSolver::sync_condition(LNSState& lns_state, SharedEnvironment* env){
        int n = env->num_of_agents;

        LNSPlan& lns_plan = lns_state.lns_plan;

        std::vector<State>& prev_state = lns_plan.plan[0];
        std::vector<State>& next_state = lns_plan.plan[1];
    
        std::fill(sim_prev_dec.begin(), sim_prev_dec.end(), -1);
        std::fill(sim_dec.begin(), sim_dec.end(), -1);
        std::fill(sim_occupied.begin(), sim_occupied.end(), false);

        for (int a = 0; a < n ; a++) {
            if (prev_state[a].location >= 0) sim_prev_dec[prev_state[a].location] = a;

            next_state[a] = State(); //Plan ulang untuk step 1 
        }

        for (int id :ids) {
            if (next_state[id].location != -1) continue;

            pibt(id, -1, prev_state, next_state, sim_prev_dec, sim_dec, sim_occupied, goal_per_agent, env, p);
        }
    }
    
    /**
     * @brief Jalanin pibt untuk window plan sebesar w (+1, untuk loop dan tracking) 
     * 
     * @param env 
     * @param w 
     * @return LNSState 
     */
    LNSState WPPLSolver::run_pibt_window(SharedEnvironment* env, int w) {
        int n = env-> num_of_agents;
        int map_size = env->map.size();

        LNSState lns_state;
        auto& lns_plan = lns_state.lns_plan;
        lns_plan.plan.resize(w+1 , std::vector<State>(n, State()));

        //Set plan[0] ke current state
        for (int a = 0 ; a < n ; a++) {
            lns_plan.plan[0][a] = env->curr_states[a];
        }

        //Simulate w timesteps untuk bikin plan
        for (int t = 0 ; t < w ; t++) {
            std::vector<State>& prev_state = lns_plan.plan[t];
            std::vector<State>& next_state = lns_plan.plan[t+1];
        
            std::fill(sim_prev_dec.begin(), sim_prev_dec.end(), -1);
            std::fill(sim_dec.begin(), sim_dec.end(), -1);
            std::fill(sim_occupied.begin(), sim_occupied.end(), false);

            for (int a = 0; a < n ; a++) {
                if (prev_state[a].location >= 0) sim_prev_dec[prev_state[a].location] = a;
            }

            for (int id :ids) {
                if (next_state[id] != -1) continue;

                pibt(id, -1, prev_state, next_state, sim_prev_dec, sim_dec, sim_occupied, goal_per_agent, env, p);
            }

        }
        return lns_state;
    }
    
    
    /**
     * @brief Bagian utama planning. Mirip dengan PIBT Solver yang standar, hanya saja ditambahkan Fase 1 : windowed planning , dan Fase 2 : LNS
     * 
     * @param time_limit 
     * @param actions 
     * @param env 
     */
    void WPPLSolver::plan(int time_limit, vector<Action>& actions, SharedEnvironment* env) {
        TimePoint start_time = std::chrono::steady_clock::now();

        auto& ps = env->planner_state;
        int n = env->num_of_agents;
        int w = ps.w;

        //Sebagian copy PIBT
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents / 100;
        if (pibt_time <= 0) pibt_time = 1;

        TimePoint deadline = start_time + std::chrono::milliseconds(time_limit - pibt_time);

        // Reset tiap timestep state 
        prev_decision.assign(env->map.size(), -1);

        for (int a = 0; a < env->num_of_agents; a++) {

            assert(env->curr_states[a].location >= 0);
            // Inisialisasi decided pertama kali
            if (decided[a].loc == -1) {
                decided[a].loc   = env->curr_states[a].location;
                assert(decided[a].state == DONE::DONE);
            }

            // Cek apakah agen sudah sampai ke decided.loc
            if (env->curr_states[a].location == decided[a].loc) {
                decided[a].state = DONE::DONE;
            }

            prev_decision[env->curr_states[a].location] = a;

            // Update goal
            goal_per_agent[a] = env->goal_locations[a].empty() ? env->curr_states[a].location : env->goal_locations[a].front().first;

            // Update priorities (pakai dari original framework)
            bool is_at_goal = (env->curr_states[a].location == goal_per_agent[a]);

            if (env->goal_locations[a].empty()) {
                // Tidak ada goal = reset ke base priority (sama dengan original)
                p[a] = p_copy[a];
            } else if (is_at_goal) {
                // Sudah di goal = reset ke base priority
                p[a] = p_copy[a];
            } else {
                // Belum sampai = naikkan priority 
                p[a] += 1.0;
            }

            // Bonus priority untuk dead-end (sama dengan original framework)
            if (!env->goal_locations[a].empty() &&
                env->degree_map[env->curr_states[a].location] == 1) {
                p[a] += 10.0;
            }
        }

        // sort descending, utamain yang priority tinggi
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
            return p[a] > p[b];  
        });

        // Fase 1 : w timestep plan (simulasi pertama)
        LNSPlan& lns_plan = lns_state.lns_plan;

        if (lns_state.has_cached_plan) {
            lns_plan = lns_state.cached_plan;

            // Masukkan cached plan lalu resync pada step 0 dengan kondisi pada saat ini
            for (int a = 0 ; a < n ; a++) {
                lns_plan.plan[0][a] = env->curr_states[a];
            }

            sync_condition(lns_state, env);
        } else {
            lns_state = run_pibt_window(env, w);
        }
        //Fase 2 : LNS
        if (std::chrono::steady_clock::now() < deadline) {
            run_iterations(lns_state, env, deadline);
        }

        //Fase 3 : Extract actions
        actions.resize(n);

        //Tolak agen dengan tujuan yang udah di claim, atau obstacle, bukan posisi bersebelahan atau lokasi invalid lainnya. Nge utamain dari current_states

        int cols = env-> cols;
        std::vector<int> claimed(env->map.size(), -1);
        
        for (int a : ids) {
            int curr_loc = lns_plan.plan[0][a].location;  
            int next_loc = lns_plan.plan[1][a].location;
            
            bool valid = true;

            // next_loc ga ada
            if (next_loc < 0 || next_loc >= env->map.size()) valid = false;

            // next_loc obstacle
            if (valid && env->map[next_loc] == 1) valid = false;

            // next_Loc ga bersebelahan atau
            if (valid && next_loc != curr_loc) {
                int dr = abs(next_loc / cols - curr_loc / cols);
                int dc = abs(next_loc % cols - curr_loc % cols);
                bool adj = ((dr + dc) == 1);

                if (!adj) valid = false;
            }

            // Vertex conflict
            if (valid && claimed[next_loc] != -1) valid = false;

            // Cek valid, reserve kalau ya, wait kalau tidak
            if (valid) {
                claimed[next_loc] = a;
                decided[a] = DCR({
                    next_loc,
                    DONE::NOT_DONE
                });    
            }
            else {
                // lns_plan.plan[1][a] = lns_plan.plan[0][a];
                // claimed[curr_loc] = a;
            
                if (claimed[curr_loc] == -1) claimed[curr_loc] = a;
                
                decided[a] = DCR({
                    curr_loc, DONE::DONE
                });
            }

            actions[a] = getAction(env->curr_states[a] , decided[a].loc, env);
            checked[a] = false;
        }
        

        // resolve untuk agent yang maju kalaui bertabrakan 
        for (int a = 0 ; a < n ; a++) {
            if (!checked[a] && actions[a] == Action::FW) 
                moveCheck(a, checked, decided, actions, prev_decision);
        }

        // Fase 4 : cache sisa plan sebagai warm up
        shift_plan(lns_state, 1, env);
        lns_state.cached_plan = lns_plan;
        lns_state.has_cached_plan = true;

        //Fase 5 GCM update & cooldownyna (sama kayak pibt)
        gcm_cooldown(env, env->curr_timestep);

        for (int a : ids) {
            gcm_update( env, env->curr_states[a].location, env->curr_states[a].orientation, env->curr_timestep);
        }

        if (env->curr_timestep > 0 && env->curr_timestep % ps.gcm_freq == 0) {
            for (auto& loc : ps.wait_map) loc.fill(0);
        }
    }


}
