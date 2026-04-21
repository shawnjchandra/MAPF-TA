#pragma once
#include "States.h"
#include "Grid.h"
#include "nlohmann/json.hpp"
#include "Tasks.h"
#include <unordered_map>
#include "Types.h"

#include <random>

typedef std::chrono::steady_clock::time_point TimePoint;
typedef std::chrono::milliseconds milliseconds;
typedef std::unordered_map<int, Task> TaskPool;

class SharedEnvironment
{
public:
    int num_of_agents;
    int rows;
    int cols;
    std::string map_name;
    std::vector<int> map;
    std::string file_storage_path;

    // Modifikasi
    // ------------------------------
    std::mt19937 rng;


    //Preprocessing
    CustomAlgo::HPA_H hpa_h;
    CustomAlgo::Neighbors ns; // Untuk penggunaan yang berulang
    std::vector<int> degree_map;
    std::vector<int> offsets; 
    int k;  //Number of cluster
    int c_penalty;  //Penalty constant for going against highway
    int r; //radius
    int max_hw; //Limiting the amount of highways generated

    //Scheduling
    std::vector<int> agent_starts;

    // vector<int> agt_unopened_tasks; 
    // vector<int> agt_opened_tasks; 

    int total_actual_duration = 0;
    int total_min_duration = 0;
    int num_task_finished = 0;

    std::vector<int> square_density;
    std::vector<int> newly_completed_tasks;
    std::vector<int> reserved_task_schedule;
    std::vector<bool> dbc_reserved; // Kalo agen sudah direserved, jangan masukin ke dtr ato dbc lagi sampai task reserved selesai
    std::unordered_map<int,int> makespan;

    //Planning
    string mode;

    CustomAlgo::PlannerState planner_state;
    int horizon;
    int m; //Jumlah thread Worker
    int N_prctg; // Neighborhood size percentage (LNS)
    float gamma; // Variabel untuk update weights dari DestroyHeuristics
    float alpha; // Variabel untuk update gcm

    // ------------------------------


    // goal locations for each agent
    // each task is a pair of <goal_loc, reveal_time>
    std::vector<std::vector<pair<int, int> > > goal_locations;

    int curr_timestep = 0;
    vector<State> curr_states;

    TaskPool task_pool; // task_id -> Task
    vector<int> new_tasks; // task ids of tasks that are newly revealed in the current timestep
    vector<int> new_freeagents; // agent ids of agents that are newly free in the current timestep
    vector<int> curr_task_schedule; // the current scheduler, agent_id -> task_id

    // plan_start_time records the time point that plan/initialise() function is called; 
    // It is a convenient variable to help planners/schedulers to keep track of time.
    // plan_start_time is updated when the simulation system call the entry plan function, its type is std::chrono::steady_clock::time_point
    TimePoint plan_start_time;

    SharedEnvironment(){}
};
