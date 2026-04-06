#include "drop-lns.h"

#include <chrono>
#include <thread>
#include <condition_variable>
#include "pibt.h"

namespace CustomAlgo {

    using Clock = std::chrono::steady_clock;


    /**
     * @brief Method utama untuk menjalankan lns, membuat thread sebanyak m, dan digabungkan pada vector utama.
     * 
     * @param lns 
     * @param non_disabled_agents 
     * @param env 
     * @param time_limit_ms 
     */
    void run_lns(
        LNS& lns,
        const std::vector<int> non_disabled_agents,
        SharedEnvironment* env,
        int time_limit_ms
    ) {
        auto start_t = Clock::now();

        /*
        Vector thread, langsung melakukan destroy dan repair tanpa enkapsulasi tambahan (agar sederhana)
        */
        std::vector<std::thread> threads;
        for (int i = 0 ; i < env->m ; i++) {
            threads.emplace_back([&](){
                while (true) {
                    auto remaining_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_t).count();

                    if (remaining_duration >= time_limit_ms) break;

                    destroy_and_repair(lns, non_disabled_agents, env);
                }
            });
        }

        //Join ke main thread
        for (std::thread& t : threads) t.join();
    
    }


    /**
     * @brief Method untuk melakukan destroy dan repair (D&R) dari LNS.
     * 
     * @param lns 
     * @param non_disabled_agents 
     * @param env 
     */
    void destroy_and_repair( 
        LNS& lns,
        const std::vector<int> non_disabled_agents,
        SharedEnvironment* env
    ){
        std::vector<std::vector<int>> P;
        std::array<float, (int)DestroyHeuristic::COUNT> weights;
        
        // Lock guard mutex untuk akses shared variables nya
        {
            std::lock_guard<std::mutex> lk(lns.mtx);
            P = lns.P_min;
            weights = lns.weights;
        }

        float C = compute_soc(P, non_disabled_agents,env);

        DestroyHeuristic h = select_heuristic_method(weights);
        int N = max(1, (int)(non_disabled_agents.size() * env->N_prctg));

        // Pilih dan sort si neighbourhood berdasarkan DestroyHeuristic h
        auto neighbourhood = select_neighbourhood(h, N, P, non_disabled_agents, env);

        /*
        Hitung soc antara neighbourhood lama dan yang baru
        */
        float destroyed_soc = compute_soc(P, neighbourhood, env);

        std::vector<std::vector<int>> empty_warm(env->num_of_agents);

        auto repaired_neighbourhood = init_pibt_window(neighbourhood, empty_warm, env);

        float repaired_soc = compute_soc(repaired_neighbourhood, neighbourhood, env); 

        //Update menggunakan mutex
        if ( repaired_soc < destroyed_soc) {
            std::vector<std::vector<int>> P_new(P);
            for (int a : neighbourhood) {
                P_new[a] = repaired_neighbourhood[a];
            }

            float C_new = compute_soc(P_new, non_disabled_agents, env);

            std::lock_guard<std::mutex> lk(lns.mtx);
            lns.weights[(int)h] = max(0.01f, env->gamma * (C - C_new) + (1-env->gamma) * lns.weights[(int)h]);
            if (C_new < lns.P_min_soc) {
                lns.P_min_soc = C_new;
                lns.P_min = P_new;
            }
        } else {
            std::lock_guard<std::mutex> lk(lns.mtx);
            lns.weights[(int)h] = (1- env->gamma) * lns.weights[(int)h];
        }
    }

    DestroyHeuristic select_heuristic_method(
        const std::array<float, (int)DestroyHeuristic::COUNT>& weights
    ) {
        float total = 0.0f;
        for (float v : weights) total += v;
        float random = rng(total);

        float acc = 0.0f;
        for (int i = 0 ; i < (int)DestroyHeuristic::COUNT ; i++) {
            acc+= weights[i];
            if ( random <= acc ) return static_cast<DestroyHeuristic>(i);
        }
        return DestroyHeuristic::RANDOM;
    }

    /**
     * @brief Switch case untuk pemilihan heuristic. Tidak sepenuhnya menerapakan paper "Anytime MAPF", hanya konsep heuristik saja.
     * Random : Diurutkan secara acak
     * AGENT_BASED : Diurutkan berdasarkan jumlah kondisi wait yang lebih banyak
     * MAP-BASED : Ambil lokasi random dan hitung jarak antara setiap agen ke lokasi. 
     * 
     * @param h 
     * @param N 
     * @param plans 
     * @param non_disabled_agents 
     * @param env 
     * @return std::vector<int> 
     */
    std::vector<int> select_neighbourhood( 
        DestroyHeuristic h, 
        int N, 
        const std::vector<std::vector<int>>& plans,
        const std::vector<int> non_disabled_agents,
        SharedEnvironment* env
    ) {
       std::vector<int> candidates(non_disabled_agents.begin(), non_disabled_agents.end());

       switch (h)
       {
       case DestroyHeuristic::RANDOM : {
            std::mt19937 gen(0);
            std::shuffle(candidates.begin(), candidates.end(), gen);

           break;
        }
       case DestroyHeuristic::AGENT_BASED: {
            std::sort(candidates.begin(), candidates.end(), [&](int a, int b){
                int wa = 0;
                int wb = 0;
                int max_path = max(plans[a].size(), plans[b].size());

                for (int step = 0 ; step < max_path ; step++) {
                    if (step < plans[a].size() - 1 ){
                        if (plans[a][step] == plans[a][step+1] ) wa++;
                    }
                    if (step < plans[b].size() - 1 ){
                        if (plans[b][step] == plans[b][step+1] ) wb++;
                    }
                }
                return wa > wb;
            });
           break;
        }
       case DestroyHeuristic::MAP_BASED: {
            int random_loc = CustomAlgo::rng(0,env->map.size() -1 );
            
            std::sort(candidates.begin(), candidates.end(), [&](int a, int b){
                int dist_a = manhattanDistance(env->curr_states[a].location, random_loc,env);
                int dist_b = manhattanDistance(env->curr_states[b].location, random_loc,env);

                return dist_a < dist_b;
            });
            
            break;
        }
       default:
            break;
       }

       if (candidates.size() > N) candidates.resize(N);
       return candidates;
    }


    /**
     * @brief Hitung Sum-of-Costs dari jumlah pergerakan di seluruh plan.
     * 
     * @param plans 
     * @param non_disabled_agents 
     * @param env 
     * @return float 
     */
    float compute_soc(
        const std::vector<std::vector<int>>& plans,
        const std::vector<int> non_disabled_agents,
        SharedEnvironment* env
    ) {
        float soc = 0.0f;
        for (int a : non_disabled_agents) {
            if (plans[a].empty()) continue;


            int goal = env->goal_locations[a].empty() ? plans[a][0] : env->goal_locations[a].front().first;
        
            for (int step = 1 ; step < plans[a].size() ; step++) {
                if (plans[a][step] != goal ) soc += 1.0f;
                else break;
            }
        }
        return soc;
    }
}