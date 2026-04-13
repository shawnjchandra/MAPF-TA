#pragma once
#include <vector>

namespace CustomAlgo {
    //table[loc][t] = agent_id 
    class ReservationTable {
        private:
        std::vector<std::vector<int>> table;
        
        public:
        ReservationTable(){};

        void init(int map_size, int window_size);

        void reserve(int loc, int t, int agent_id);

        void unreserve(int loc, int t);

        int get(int loc, int t) ;

        bool is_free(int loc, int t) ;

        bool can_move(int from, int to, int t) ;

        void clear();

        int get_window_size();
    };
}