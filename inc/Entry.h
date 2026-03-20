#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"

// Restore these so the .cpp files compile!
#include "MAPFPlanner.h"
#include "TaskScheduler.h"
#include "../custom_algo/preprocessing/preprocessing.h"

class Entry
{
public:
    SharedEnvironment* env;
    MAPFPlanner* planner;
    TaskScheduler* scheduler;
    Preprocessing* preprocessing; // Your custom class

    Entry(SharedEnvironment* env): env(env)
    {
        planner = new MAPFPlanner(env);
        scheduler = new TaskScheduler(env);
        preprocessing = new Preprocessing(env);
    };
    
    Entry()
    {
        env = new SharedEnvironment();
        planner = new MAPFPlanner(env);
        scheduler = new TaskScheduler(env);
        preprocessing = new Preprocessing(env);
    };
    
    virtual ~Entry(){
        delete env;
        delete planner;
        delete scheduler;
        delete preprocessing;
    };

    virtual void initialize(int preprocess_time_limit);

    // RESTORE THESE! We need them to exist to pass compilation
    virtual void compute(int time_limit, std::vector<Action> & plan, std::vector<int> & proposed_schedule);
    void update_goal_locations(std::vector<int> & proposed_schedule);
};