#pragma once

#include "Types.h"
#include "random"
#include "SharedEnv.h"

namespace CustomAlgo{

    enum ROTATION {NORTH, EAST, SOUTH, WEST};

    bool validateMove(int loc, int loc2, const SharedEnvironment* env);

    void getNeighbors(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) ;

    void getNeighbors_nowait(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) ;

    void getNeighborLocs(const Neighbors* ns, std::vector<int>& neighbors, int location) ;


    int manhattanDistance(int loc, int loc2,const SharedEnvironment* env);

    Action getAction(State& prev, State& next);

    Action getAction(State& prev, int next_loc, SharedEnvironment* env);

    // Modifikasi
    int rng(int min, int max);
    float rng(float max);

    float get_random_float(std::mt19937* MT, float from = 0, float to = 1);
    int get_random_int(std::mt19937* MT, int from = 0, int to = 1);

    int getRotationCost(int currentOrientation, int targetOrientation);

    int getBackwardLocation(int loc, int orientation, SharedEnvironment* env);

    int getForwardLocation(int prev_loc, int prev_orient, SharedEnvironment* env);

    int getOrientationBetween(int u_loc, int v_loc, int map_width);
    int get_neighbor_orientation(int u_loc, int v_loc, int map_width, int current_orient);

    int reverseOrientation(int orientation);
}

