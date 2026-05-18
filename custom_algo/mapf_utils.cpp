#include "mapf_utils.h"

namespace CustomAlgo{
    
    //BAWAAN (SEMENTARA kemungkinan untuk PIBT)
    bool validateMove(int loc, int loc2, const SharedEnvironment* env){
        int loc_row = loc/env->cols;
        int loc_col = loc%env->cols;

        if (loc_row < 0 || loc_col < 0 || loc_row >= env->rows || loc_col >= env->cols || env->map[loc] == 1)
            return false;

        int loc2_row = loc2/env->cols;
        int loc2_col = loc2%env->cols;
        if (loc2_row < 0 || loc2_col < 0 ||loc2_row >= env->rows || loc2_col >= env->cols || env->map[loc2] == 1)
            return false;
        if (abs(loc_row-loc2_row) + abs(loc_col-loc2_col) > 1)
            return false;
        return true;
    };



    int manhattanDistance(int loc, int loc2,const SharedEnvironment* env){
        int loc_row = loc/env->cols;
        int loc_col = loc%env->cols;
        int loc2_row = loc2/env->cols;
        int loc2_col = loc2%env->cols;
        return abs(loc_row-loc2_row) + abs(loc_col-loc2_col);

    }

    void getNeighborLocs(const Neighbors* ns, std::vector<int>& neighbors, int location) {
        neighbors.clear();
        //forward
        assert(location >= 0 && location < ns->size());
        neighbors = ns->at(location);
        return;

    }

    Action getAction(State& prev, int next_loc, SharedEnvironment* env){
        if (prev.location == next_loc){
            return Action::W;
        }
        int diff = next_loc -prev.location;
        int orientation;
        if (diff == 1){
            orientation = 0;
        }
        if (diff == -1){
            orientation = 2;
        }
        if (diff == env->cols){
            orientation = 1;
        }
        if (diff == -env->cols){
            orientation = 3;
        }
        if (orientation == prev.orientation){
            return Action::FW;
        }
        if (orientation  == (prev.orientation+1)%4){
            return Action::CR;
        }
        if (orientation  == (prev.orientation+3)%4){
            return Action::CCR;
        }
        if (orientation  == (prev.orientation+2)%4){
            return Action::CR;
        }
        assert(false);


        return Action::W;

    }
    //==================================================================================

    // @details
    // Modifikasi / Tambahan

    int rng(int min, int max) {
        //Thread local untuk nanti DROP-LNS juga
        thread_local std::random_device rd;
        thread_local std::mt19937 gen(rd());

        std::uniform_int_distribution<int> distrib(min, max);

        return distrib(gen);
    }

    float rng(float max) {
        //Thread local untuk nanti DROP-LNS juga
        thread_local std::random_device rd;
        thread_local std::mt19937 gen(rd());

        std::uniform_real_distribution<float> distrib(max);

        return distrib(gen);
    }

    int getRotationCost(int currentOrientation, int targetOrientation) {
        int diff = abs(currentOrientation-targetOrientation);   // 0, 1 ,2 , 3
        return min(diff, 4-diff);
    }

    int getBackwardLocation(int loc, int orientation, SharedEnvironment* env) {
        int row = loc / env->cols;
        int col = loc % env->cols;

        if (orientation == 0) { 
            row++;
        } else if (orientation == 1) {
            col--;
        } else if (orientation == 2) {
            row--;
        } else {
            col++;
        }

        if (row < 0 || row == env->rows || col < 0 || col == env->cols ) return -1;

        int prevLoc = row * env->cols + col;    //Outputnya lokasi global
        if (env->map[prevLoc] == 0) return prevLoc;
        else return -1;        
    }

    int getOrientationBetween(int u_loc, int v_loc) {
        int dif = v_loc - u_loc;
        
        if (dif < -1) return 0; // Kalau geraknya dari posisi ke atas
        else if (dif == 1) return 1; //Dari posisi ke kanan
        else if (dif > 1) return 2; // Dari posisi ke bawah
        else return 3;  // Dari posisi ke kiri
    }

    int reverseOrientation(int orientation) {
        return (orientation + 2) % 4;
    }

}