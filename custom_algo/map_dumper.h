#pragma once
#include "SharedEnv.h"
#include <string>

namespace CustomAlgo {
    /**
     * @brief Dump heatmap (GCM), voronoi map, highway, dan HPA abstraction ke file JSON.
     *
     * File output: <base_output_path tanpa ekstensi> + ".maps.json".
     * Contoh: jika output simulator = "./run/result.json", maps di-dump ke
     *         "./run/result.maps.json".
     *
     * Aman dipanggil walaupun preprocessing/planner belum sempat inisialisasi
     * (akan menulis bagian yang kosong saja, tidak crash).
     *
     * @param env  SharedEnvironment milik planner (Entry::env).
     * @param base_output_path  Path file output utama (yang diberikan via -o).
     */
    void dump_maps(SharedEnvironment* env, const std::string& base_output_path);
}