#include "planner.h"
#include "mapf_utils.h"
#include "heuristics.h"
#include "instance.h"


namespace CustomAlgo {

    std::vector<State> starts;
    std::vector<std::vector<std::pair<int,int>>> goal_locations;
    std::vector<Path> paths;
    bool need_replan;
    int timestep;
    int num_of_drives;
    // int total_feasible_timestep = 0;
    int simulation_window;
    bool consider_rotation;
    std::mt19937 mt;
    Instance* ins;
    PIBT* pibt;
    Graph* G;
    std::vector<AgentInfo> agent_infos;

    void update_start_locations()
        {
            for (int k = 0; k < num_of_drives; k++)
            {
                starts[k] = State(paths[k][timestep].location, 0, paths[k][timestep].orientation);
            }
        }

    void update_goal_locations(const SharedEnvironment* env){
        for (int i=0;i<num_of_drives;++i){
            goal_locations[i].clear();
            for (int j=0;j<env->goal_locations[i].size();++j){
                // we require path of at least length one, even the start and the goal are the same.
                goal_locations[i].emplace_back(env->goal_locations[i][j].first,max(env->goal_locations[i][j].second-timestep,1));
            }
        }
    }

    Action get_action_from_states(const State & state, const State & next_state, bool consider_rotation){
        

        assert(state.timestep+1==next_state.timestep);
        // TODO: assert consider_rotation

        if (consider_rotation){
            if (state.location==next_state.location){
                // either turn or wait
                if (state.orientation==next_state.orientation) {
                    return Action::W;
                } else if ((state.orientation-next_state.orientation+4)%4==1) {
                    return Action::CR;
                } else if ((state.orientation-next_state.orientation+4)%4==3) {
                    return Action::CCR;
                } else {
                    assert(false);
                    return Action::W;
                }
            }
            else {
                return Action::FW;
            }
        } else {
            // TODO
            assert(false);
            return Action::W;
        }
    }
    int get_next_location(const State& s, Action a, SharedEnvironment* env) {
            switch(a) {
                case Action::FW:
                    switch(s.orientation) {
                        case 0: return s.location + 1;           // east
                        case 1: return s.location + env->cols;   // south
                        case 2: return s.location - 1;           // west
                        case 3: return s.location - env->cols;   // north
                    }
                case Action::W:  return s.location; // wait
                case Action::CR: return s.location; // rotate right (loc unchanged)
                case Action::CCR:return s.location; // rotate left  (loc unchanged)
                default: return s.location;
            }
        }

    bool is_valid_actions(const vector<State>& curr_states, const vector<Action>& actions, SharedEnvironment* env) {

    int n = curr_states.size();
    vector<int> next_locs(n);
    
    for (int i = 0; i < n; ++i) {
        next_locs[i] = get_next_location(curr_states[i], actions[i], env);
    }
    
    // check vertex conflicts
    unordered_map<int, int> loc_to_agent;
    for (int i = 0; i < n; ++i) {
        if (loc_to_agent.count(next_locs[i])) {
            std::cerr << "vertex conflict: agent " << i << " and agent " 
                 << loc_to_agent[next_locs[i]] << " both at " << next_locs[i] << endl;
            return false;
        }
        loc_to_agent[next_locs[i]] = i;
    }
    
    // check swap conflicts
    for (int i = 0; i < n; ++i) {
        for (int j = i+1; j < n; ++j) {
            if (next_locs[i] == curr_states[j].location &&
                next_locs[j] == curr_states[i].location) {
                std::cerr << "swap conflict: agent " << i << " and agent " << j << endl;
                return false;
            }
        }
    }
    cout << "asli udah returnnya bener" << std::endl; 
    
    return true;
}

   void get_step_actions(SharedEnvironment* env, vector<Action>& actions) {
    cout << "agent 0 env->curr_states loc=" << env->curr_states[0].location 
     << " paths[timestep] loc=" << paths[0][timestep].location
     << " paths[timestep+1] loc=" << paths[0][timestep+1].location
     << " internal timestep=" << timestep << endl;
    assert(actions.empty());

    for (int i = 0; i < num_of_drives; ++i) {
        if (paths[i].size() <= timestep + 1) {
            cerr << "path too short agent " << i << endl;
            assert(false);
        }
        
        auto& curr = paths[i][timestep];
        auto& next = paths[i][timestep+1];
        
        if (i < 3) { // just check first 3 agents
            cout << "agent " << i 
                 << " curr=(" << curr.location << "," << curr.orientation << ",ts=" << curr.timestep << ")"
                 << " next=(" << next.location << "," << next.orientation << ",ts=" << next.timestep << ")"
                 << endl;
        }
        
        actions.push_back(get_action_from_states(curr, next, consider_rotation));
    }
    
    cout << "actions BEFORE validation: [0]=" << (int)actions[0] 
    << " [1]=" << (int)actions[1] << endl;
    
    if (!is_valid_actions(env->curr_states, actions, env)) {
        cerr << "actions invalid!" << endl;
        actions.resize(num_of_drives, Action::W);
    } else {

        cout << "valid actionnya [0]=" << (int)actions[0] << endl;
    }
    timestep++;
    
    cout << "actions AFTER validation: [0]=" << (int)actions[0] << endl;
}

   

    State compute_next_state(const State& curr, Agent* agent, SharedEnvironment* env) {
        State next;
        next.timestep    = curr.timestep + 1;
        next.location    = agent->v_next->index;
        next.orientation = agent->o_next;
        return next;
    }

    Instance* build_instance(const SharedEnvironment* env, std::vector<Path>* precomputed_paths) {
        auto starts = std::vector<std::pair<uint,int>>();
        auto goals  = std::vector<std::pair<uint,int>>();

        for (int i = 0; i < env->num_of_agents; ++i) {
            starts.emplace_back(env->curr_states[i].location, env->curr_states[i].orientation);

            assert(env->goal_locations[i].size() > 0);
            int goal_location = env->goal_locations[i][0].first;

            auto& agent_info = agent_infos[i];

            if (goal_location != agent_info.goal_location) {
                agent_info.goal_location = goal_location;
                agent_info.elapsed       = 0;
                agent_info.tie_breaker   = get_random_float(&mt, 0, 1);
                agent_info.stuck_order   = 0;
            } else {
                agent_info.elapsed += 1;
            }

            goals.emplace_back(goal_location, -1);
        }

        if (ins != nullptr) delete ins;

        return new Instance(*G, starts, goals, agent_infos, -1, precomputed_paths);
        }

    void initialize(int preprocess_time_limit, SharedEnvironment* env) {
        //PS
        int map_size = env->map.size();
        int agent_size = env->num_of_agents;

        auto& ps = env->planner_state;
        ps.w = max(3, ps.w);
        ps.h = max(1, ps.h);
        env->m = 3;

        ps.gcm.assign(map_size, {1.0f, 1.0f, 1.0f, 1.0f});
        ps.wait_map.assign(map_size, {0,0,0,0});
        ps.w_peak.resize(map_size);
        for (auto& wp : ps.w_peak) {
            wp.val = 1.0f;
            wp.timestep = 0;
        }

        ps.gcm_freq = max(ps.gcm_freq, 5);
        ps.w_baseline = max(ps.w_baseline, 1.0f);
        ps.k_base = max(ps.k_base, 0.1f);
        ps.max_degree = 4;

        //PIBT
        need_replan = true;
        timestep = 0;
        num_of_drives = env->num_of_agents;
        // cout << "num_of_drives: " << num_of_drives << endl;

        simulation_window = env->planner_state.w;
        // cout << "simulation_window: " << simulation_window << endl;

        consider_rotation = true;
        // cout << "consider_rotation set" << endl;

        mt = std::mt19937(std::random_device{}());
        // cout << "mt initialized" << endl;

        agent_infos.resize(num_of_drives);
        // cout << "agent_infos resized" << endl;

        for (int i = 0; i < num_of_drives; ++i) {
            agent_infos[i].id = i;
            agent_infos[i].goal_location = -1; // will be set properly in build_instance
        }
        // cout << "agent_infos built" << endl;

        // std::vector<std::pair<uint,int>> start_indexes(num_of_drives);
        // std::vector<std::pair<uint,int>> goal_indexes(num_of_drives);
        // for (int i = 0; i < num_of_drives; ++i) {
        //     start_indexes[i] = {env->curr_states[i].location, env->curr_states[i].orientation};
        //     goal_indexes[i]  = {agent_infos[i].goal_location, -1};
        // }
        // // cout << "start/goal indexes built" << endl;

        G = new CustomAlgo::Graph(*env);
        // cout << "graph built V.size()=" << G->V.size() << " U.size()=" << G->U.size() << endl;

        // ins = new Instance(*G, start_indexes, goal_indexes, agent_infos);
        // // cout << "instance built" << endl;

        pibt = new PIBT(&mt, env, nullptr);
        // // cout << "pibt constructed" << endl;

        pibt->initialize(G);
        // cout << "pibt initialized" << endl;
    }

    void plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env) {
    // cout << "plan called, timestep=" << timestep << endl;

    if (paths.size() == 0) {
        // cout << "initializing paths" << endl;
        starts.resize(num_of_drives);
        goal_locations.resize(num_of_drives);
        paths.resize(num_of_drives);

        for (int i = 0; i < num_of_drives; ++i) {
            paths[i].push_back(env->curr_states[i]);
        }
        // cout << "paths initialized" << endl;
    }

    if (need_replan) {
        // cout << "need_replan=true, building instance" << endl;
        for (int i = 0; i < num_of_drives; ++i) {
            paths[i].resize(timestep + 1);
            paths[i][timestep] = env->curr_states[i]; // <-- sync here
        }
        // cout << "paths resized" << endl;

        update_start_locations();
        // cout << "starts updated" << endl;

        update_goal_locations(env);
        // cout << "goals updated" << endl;

        ins = build_instance(env);
        // cout << "instance built" << endl;

        pibt->ins = ins;

        Config ctx(num_of_drives);
        // cout << "ctx created" << endl;

        for (int i = 0; i < num_of_drives; ++i) {
            ctx.locs[i] = G->U[env->curr_states[i].location];
            // cout << "agent " << i << " loc=" << env->curr_states[i].location  << " vertex=" << (ctx.locs[i] == nullptr ? "NULL" : std::to_string(ctx.locs[i]->index))  << endl;
            ctx.orients[i] = env->curr_states[i].orientation;
        }
        // cout << "ctx filled" << endl;

        ctx.timestep = timestep;

        //PIBT
        bool success = pibt->get_new_config(ctx);
        // cout << "get_new_config done, success=" << success << endl;

        for (int i = 0; i < 3; ++i) {
            cout << "pibt->A[" << i << "]->v_next = " << (pibt->A[i]->v_next == nullptr ? "NULL" : std::to_string(pibt->A[i]->v_next->index))  << endl;
            cout << "pibt->A[" << i << "]->v_now = " << (pibt->A[i]->v_now == nullptr ? "NULL" : std::to_string(pibt->A[i]->v_now->index)) << endl;
        }

        for (int i = 0; i < num_of_drives; ++i) {
            if (pibt->A[i]->v_next == nullptr) {
                std::cerr << "agent " << i << " has null v_next, using curr" << endl;
                paths[i].push_back(env->curr_states[i]); // wait
            } else {
                paths[i].push_back(compute_next_state(
                    env->curr_states[i], pibt->A[i], env));
            }
        }
        // cout << "paths updated" << endl;

        // total_feasible_timestep = timestep + simulation_window;
    } else {
        // cout << "skip planning" << endl;
    }

    get_step_actions(env, actions);
    // cout << "actions extracted" << endl;
}
    

}