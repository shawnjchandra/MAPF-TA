#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"
#include <cstdlib> // NEEDED FOR exit(0)
#include <iostream>

void Entry::initialize(int preprocess_time_limit)
{
    std::cout << "🚀 RUNNING PREPROCESSING..." << std::endl;
    
    // FIXED: Use the arrow operator (->) to call it on your object!
    preprocessing->initialize(preprocess_time_limit);
    
    std::cout << "✅ PREPROCESSING DONE. KILLING SIMULATION." << std::endl;
    
    // This instantly stops the program. It will NEVER run compute or the planner!
    exit(0); 
}

// RESTORE THIS! It will just sit here doing nothing, but the compiler needs it.
void Entry::compute(int time_limit, std::vector<Action> & plan, std::vector<int> & proposed_schedule)
{
    // Leave empty. It will never run because of exit(0).
}

// RESTORE THIS!
void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    // Leave empty. It will never run because of exit(0).
}