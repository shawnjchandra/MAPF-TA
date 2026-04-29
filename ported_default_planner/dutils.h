
#ifndef utils_hpp
#define utils_hpp

#include "dTypes.h"

namespace CustomAlgo{
int get_d(int diff, const SharedEnvironment* env)  ;


bool d_validateMove(int loc, int loc2, const SharedEnvironment* env);

int d_manhattanDistance(int loc, int loc2,const SharedEnvironment* env);

void getNeighbors(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) ;

void getNeighbors_nowait(const SharedEnvironment* env, std::vector<std::pair<int,int>>& neighbors, int location,int direction) ;

void d_getNeighborLocs(const Neighbors* ns, std::vector<int>& neighbors, int location) ;

void d_getNeighborLocs(const Neighbors* ns, int neighbors[], int location) ;
}

#endif