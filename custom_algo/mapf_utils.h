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

    void getNeighborLocs(const Neighbors* ns, int neighbors[], int location) ;

    int manhattanDistance(int loc, int loc2,const SharedEnvironment* env);

    // Modifikasi
    int rng(int min, int max);

    int getRotationCost(int currentOrientation, int targetOrientation);

    int getBackwardLocation(int loc, int orientation, SharedEnvironment* env);

    int getOrientationBetween(int u_loc, int v_loc);

    int reverseOrientation(int orientation);
}

