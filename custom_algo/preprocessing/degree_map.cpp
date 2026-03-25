#include "degree_map.h"

namespace CustomAlgo{

    // Hanya hitung nilai neighbor di lokasi loc saja
    int degreeNeighbors(SharedEnvironment* env, int loc) {
        int deg = 0;
        if(env->map[loc] = 0) {
            int row = loc / env->cols;
            int col = loc % env->cols;

            if(row>0 && env->map[loc-env->cols]==0){
                deg++;
            }
            if(row<env->rows-1 && env->map[loc+env->cols]==0){
                deg++;
            }
            if(col>0 && env->map[loc-1]==0){
                deg++;
            }
            if(col<env->cols-1 && env->map[loc+1]==0){
                deg++;
            }
        } 
        return deg;
    }
}