#include "mapf_utils.h"

namespace CustomAlgo{
    
    //BAWAAN (SEMENTARA kemungkinan untuk PIBT)
    bool validateMove(int loc, int loc2, const SharedEnvironment* env){
        int cols = env->cols;

        int loc_row = loc/cols;
        int loc_col = loc%cols;

        if (loc_row < 0 || loc_col < 0 || loc_row >= env->rows || loc_col >= cols || env->map[loc] == 1)
            return false;

        int loc2_row = loc2/cols;
        int loc2_col = loc2%cols;

        if (loc2_row < 0 || loc2_col < 0 ||loc2_row >= env->rows || loc2_col >= cols || env->map[loc2] == 1)
            return false;

        // Cek kalo diagonal
        if (abs(loc_row-loc2_row) + abs(loc_col-loc2_col) > 1)
            return false;

        //Cek vertikal
        if (abs(loc_row-loc2_row) > 1)
            return false;

        //Cek horizontal
        if (abs(loc_col-loc2_col) > 1)
            return false;

        // Cek wrapping kiri dan kanan
        if (loc_col == 0 && loc2 == loc - 1) return false;    
        if (loc_col == cols - 1 && loc2 == loc + 1) return false;

        if (loc_col == cols - 1 && loc2_col == 0 ) return false;    
        if (loc_col == 0 && loc2_col == cols - 1 ) return false;  

        return true;
    };

    void getNeighbors(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) {
        neighbors.clear();
        //forward
        assert(location >= 0 && location < env->map.size());
        int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
        int forward = candidates[direction];
        int new_direction = direction;
        assert(forward!=location);

        #ifndef NDEBUG
                std::cout<<"forward: "<<forward<<std::endl;
        #endif
        if (validateMove(location, forward, env)	){
            #ifndef NDEBUG
                std::cout<<"forward yes"<<std::endl;
            #endif
            neighbors.emplace_back(std::make_pair(forward,new_direction));
        }
        //turn left
        new_direction = direction-1;
        if (new_direction == -1)
            new_direction = 3;
        assert(new_direction >= 0 && new_direction < 4);
        neighbors.emplace_back(std::make_pair(location,new_direction));
        //turn right
        new_direction = direction+1;
        if (new_direction == 4)
            new_direction = 0;
        assert(new_direction >= 0 && new_direction < 4);
        neighbors.emplace_back(std::make_pair(location,new_direction));
        neighbors.emplace_back(std::make_pair(location,direction)); //wait
    };

    void getNeighbors_nowait(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) {
        neighbors.clear();
        //forward
        assert(location >= 0 && location < env->map.size());
        int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
        int forward = candidates[direction];
        int new_direction = direction;
        if (validateMove(location, forward, env)){
            assert(forward >= 0 && forward < env->map.size());
            neighbors.emplace_back(std::make_pair(forward,new_direction));
        }
        //turn left
        new_direction = direction-1;
        if (new_direction == -1)
            new_direction = 3;
        assert(new_direction >= 0 && new_direction < 4);
        neighbors.emplace_back(std::make_pair(location,new_direction));
        //turn right
        new_direction = direction+1;
        if (new_direction == 4)
            new_direction = 0;
        assert(new_direction >= 0 && new_direction < 4);
        neighbors.emplace_back(std::make_pair(location,new_direction));
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


    Action getAction(State& prev, State& next){
        if (prev.location == next.location && prev.orientation == next.orientation){
            return Action::W;
        }
        if (prev.location != next.location && prev.orientation == next.orientation){
            return Action::FW;
        }
        if (next.orientation  == (prev.orientation+1)%4){
            return Action::CR;
        }
        if (next.orientation  == (prev.orientation+3)%4){
            return Action::CCR;
        }
        assert(false);
        return Action::W;
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

    float get_random_float(std::mt19937* MT, float from, float to)
    {
    std::uniform_real_distribution<float> r(from, to);
    return r(*MT);
    }

    int get_random_int(std::mt19937* MT, int from, int to)
    {
    std::uniform_int_distribution<int> r(from, to);
    return r(*MT);
    }

    int getRotationCost(int currentOrientation, int targetOrientation) {
        int diff = abs(currentOrientation-targetOrientation);   // 0, 1 ,2 , 3
        return min(diff, 4-diff);
    }

    int getBackwardLocation(int loc, int orientation, SharedEnvironment* env) {
        int row = loc / env->cols;
        int col = loc % env->cols;

        // if (orientation == 0) { 
        //     row++;
        // } else if (orientation == 1) {
        //     col--;
        // } else if (orientation == 2) {
        //     row--;
        // } else {
        //     col++;
        // }
        if (orientation == 0) { 
            col--;
        } else if (orientation == 1) {
            row--;
        } else if (orientation == 2) {
            col++;
        } else {
            row++;
        }

        if (row < 0 || row == env->rows || col < 0 || col == env->cols ) return -1;

        int prevLoc = row * env->cols + col;    //Outputnya lokasi global
        if (env->map[prevLoc] == 0) return prevLoc;
        else return -1;        
    }

    int getForwardLocation(int prev_loc, int prev_orient, SharedEnvironment* env) {
        int map_width = env->cols;
        int next_orient_v[4] = {
            prev_loc + 1,
            prev_loc + map_width,
            prev_loc - 1,
            prev_loc - map_width
        };

        int next_loc = next_orient_v[prev_orient];
        if (!validateMove(prev_loc, next_loc, env)) return -1;
        else return next_loc;
    }

    int getOrientationBetween(int u_loc, int v_loc, int map_width) {
        int dif = v_loc - u_loc;
        if (dif == 1) return 0;           // East
        else if (dif == map_width) return 1;  // South
        else if (dif == -1) return 2;     // West
        else if (dif == -map_width) return 3; // North
        return -1; // Invalid adjacent move
    }

    int get_neighbor_orientation(int u_loc, int v_loc, int map_width, int current_orient = -1){
        int dif = v_loc - u_loc;
        if (dif == 0)          return current_orient; // stay — keep current orientation
        if (dif == 1)          return 0; // east
        if (dif == map_width)  return 1; // south
        if (dif == -1)         return 2; // west
        if (dif == -map_width) return 3; // north
        return current_orient; // fallback
    }

    int reverseOrientation(int orientation) {
        return (orientation + 2) % 4;
    }

}