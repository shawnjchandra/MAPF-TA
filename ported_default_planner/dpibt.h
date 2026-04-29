
#ifndef pibt_hpp
#define pibt_hpp

#include <vector>
#include <list>
#include <unordered_set>
#include <tuple>
#include "dTypes.h"
#include "dutils.h"
#include "dheuristic.h"
#include "TrajLNS.h"





namespace CustomAlgo{

int get_gp_h(TrajLNS& lns, int ai, int target);


bool causalPIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	 std::vector<State>& next_states,
      std::vector<int>& prev_decision, std::vector<int>& decision, 
	  std::vector<bool>& occupied,TrajLNS& lns
	  );


Action d_getAction(State& prev, State& next);

Action d_getAction(State& prev, int next_loc, SharedEnvironment* env);

bool moveCheck(int id, std::vector<bool>& checked,
		std::vector<dDCR>& decided, std::vector<Action>& actions, std::vector<int>& prev_decision);
}
#endif