#include "map_dumper.h"
#include "nlohmann/json.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>

using json = nlohmann::json;

namespace CustomAlgo {

    /**
     * @brief Turunkan path file maps dari path output simulator.
     *
     * "foo/bar.json"  -> "foo/bar.maps.json"
     * "foo/bar"       -> "foo/bar.maps.json"
     * "output.json"   -> "output.maps.json"
     */
    static std::string derive_maps_path(const std::string& base_output_path) {
        std::filesystem::path p(base_output_path);
        std::filesystem::path stem   = p.stem();      // "bar"
        std::filesystem::path parent = p.parent_path();
        std::filesystem::path out    = parent / (stem.string() + ".maps.json");
        return out.string();
    }

    void dump_maps(SharedEnvironment* env, const std::string& base_output_path) {
        if (env == nullptr) {
            std::cerr << "[dump_maps] env == nullptr, lewati." << std::endl;
            return;
        }

        const std::string out_path = derive_maps_path(base_output_path);
        const int map_size = (int)env->map.size();

        json js;

        // ---- Meta ----------------------------------------------------------
        json meta;
        meta["map_name"]       = env->map_name;
        meta["rows"]           = env->rows;
        meta["cols"]           = env->cols;
        meta["num_of_agents"]  = env->num_of_agents;
        meta["final_timestep"] = env->curr_timestep;
        meta["mode"]           = env->mode;
        meta["k"]              = env->k;
        meta["c_penalty"]      = env->c_penalty;
        meta["r"]              = env->r;
        meta["max_hw"]         = env->max_hw;
        meta["w"]              = env->w;
        meta["m"]              = env->m;
        meta["alpha"]          = env->alpha;
        meta["gcm_freq"]       = env->planner_state.gcm_freq;
        meta["w_baseline"]     = env->planner_state.w_baseline;
        meta["k_base"]         = env->planner_state.k_base;
        meta["max_degree"]     = env->planner_state.max_degree;
        meta["num_task_finished"] = env->num_task_finished;
        js["meta"] = meta;

        // ---- Map (obstacles & degree) -------------------------------------
        // map: 0 = free, 1 = obstacle (sama dengan Grid). Disimpan row-major,
        // index = row * cols + col.
        json mp;
        mp["obstacles"] = env->map;
        if ((int)env->degree_map.size() == map_size) {
            mp["degree_map"] = env->degree_map;
        } else {
            mp["degree_map"] = json::array();
        }
        js["map"] = mp;

        // ---- Heatmap (GCM) ------------------------------------------------
        // gcm[loc][orient]. Orientasi: 0=East, 1=South, 2=West, 3=North.
        json hm;
        const auto& ps = env->planner_state;

        if ((int)ps.gcm.size() == map_size && map_size > 0) {
            json gcm_arr = json::array();
            for (int loc = 0; loc < map_size; loc++) {
                gcm_arr.push_back(json::array({
                    ps.gcm[loc][0], ps.gcm[loc][1],
                    ps.gcm[loc][2], ps.gcm[loc][3]
                }));
            }
            hm["gcm"] = std::move(gcm_arr);
        } else {
            hm["gcm"] = json::array();
        }

        if ((int)ps.w_peak.size() == map_size && map_size > 0) {
            json wp_val = json::array();
            json wp_t   = json::array();
            for (int loc = 0; loc < map_size; loc++) {
                wp_val.push_back(ps.w_peak[loc].val);
                wp_t.push_back(ps.w_peak[loc].timestep);
            }
            hm["w_peak_val"] = std::move(wp_val);
            hm["w_peak_t"]   = std::move(wp_t);
        } else {
            hm["w_peak_val"] = json::array();
            hm["w_peak_t"]   = json::array();
        }

        if ((int)ps.wait_map.size() == map_size && map_size > 0) {
            json wm = json::array();
            for (int loc = 0; loc < map_size; loc++) {
                wm.push_back(json::array({
                    ps.wait_map[loc][0], ps.wait_map[loc][1],
                    ps.wait_map[loc][2], ps.wait_map[loc][3]
                }));
            }
            hm["wait_map"] = std::move(wm);
        } else {
            hm["wait_map"] = json::array();
        }
        js["heatmap"] = std::move(hm);

        // ---- Voronoi ------------------------------------------------------
        json vor;
        vor["k"] = env->k;
        if ((int)env->hpa_h.voronoi_map.size() == map_size) {
            vor["voronoi_map"] = env->hpa_h.voronoi_map;
        } else {
            vor["voronoi_map"] = json::array();
        }
        js["voronoi"] = std::move(vor);

        // ---- Highway ------------------------------------------------------
        // e_hw   : arah yang DI-IZINKAN (tidak kena penalty)
        // r_e_hw : arah yang DI-PENALTY (lawan arah highway)
        // Format: [from_loc, from_orient, to_loc, to_orient]
        json hw;
        {
            json allowed = json::array();
            for (const auto& kv : env->hpa_h.hw.e_hw) {
                allowed.push_back(json::array({
                    kv.first.first,  kv.first.second,
                    kv.second.first, kv.second.second
                }));
            }
            hw["edges_allowed"] = std::move(allowed);

            json pen = json::array();
            for (const auto& kv : env->hpa_h.hw.r_e_hw) {
                pen.push_back(json::array({
                    kv.first.first,  kv.first.second,
                    kv.second.first, kv.second.second
                }));
            }
            hw["edges_penalized"] = std::move(pen);
        }
        js["highway"] = std::move(hw);

        // ---- HPA abstraction ----------------------------------------------
        json hpa;
        hpa["gates"] = env->hpa_h.AG.gates;

        // Gates per cluster
        {
            json gpc = json::array();
            for (const auto& cluster_gates : env->hpa_h.Gates) {
                gpc.push_back(cluster_gates);
            }
            hpa["gates_per_cluster"] = std::move(gpc);
        }

        // Cluster adjacency (cluster -> [neighbor clusters])
        {
            json cadj = json::array();
            for (const auto& adj_map : env->hpa_h.cluster_adj) {
                json neighbors = json::array();
                for (const auto& kv : adj_map) {
                    neighbors.push_back(json::array({kv.first, kv.second}));
                }
                cadj.push_back(std::move(neighbors));
            }
            hpa["cluster_adj"] = std::move(cadj);
        }

        // Entrances (centroid cluster a <-> cluster b melalui loc_a-neigh)
        {
            json ents = json::array();
            for (const auto& e : env->hpa_h.Ents) {
                ents.push_back(json::array({e.loc_a, e.c_a, e.neigh, e.c_b}));
            }
            hpa["entrances"] = std::move(ents);
        }
        js["hpa"] = std::move(hpa);

        // ---- Tulis file ---------------------------------------------------
        try {
            std::ofstream f(out_path, std::ios_base::trunc | std::ios_base::out);
            if (!f.is_open()) {
                std::cerr << "[dump_maps] gagal membuka file: " << out_path << std::endl;
                return;
            }
            // Tanpa setw(4) -- compact, hemat ratusan kB - MB.
            f << js;
            std::cout << "[dump_maps] tersimpan: " << out_path
                      << "  (gcm=" << ps.gcm.size()
                      << ", voronoi=" << env->hpa_h.voronoi_map.size()
                      << ", hw_allowed=" << env->hpa_h.hw.e_hw.size()
                      << ", hw_penalized=" << env->hpa_h.hw.r_e_hw.size()
                      << ", gates=" << env->hpa_h.AG.gates.size()
                      << ")" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[dump_maps] exception saat menulis " << out_path
                      << ": " << e.what() << std::endl;
        }
    }
}