#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"
#include <cstdlib> // NEEDED FOR exit(0)
#include <iostream>

void Entry::initialize(int preprocess_time_limit)
{
    
    preprocessing->initialize(preprocess_time_limit);
    
    //Stop pada preprocessing saja    
    exit(0); 
}

void Entry::compute(int time_limit, std::vector<Action> & plan, std::vector<int> & proposed_schedule)
{

}

void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{

}