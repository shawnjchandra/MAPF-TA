
#ifndef search_hpp
#define search_hpp

#include "dTypes.h"
#include "dutils.h"
#include "Memory.h"
#include "heap.h"
#include "search_node.h"
#include "dheuristic.h"

namespace CustomAlgo{
//a astar minimized the opposide traffic flow with existing traffic flow

s_node astar(SharedEnvironment* env, std::vector<dInt4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns);
}

#endif