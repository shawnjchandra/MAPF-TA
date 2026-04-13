#include "reservation_table.h"

namespace CustomAlgo{
    

    void ReservationTable::init(int map_size, int window_size) {
        table.assign(map_size, std::vector<int>(window_size+1 , -1));
    }

    int ReservationTable::get_window_size() {
        return table.empty() ? 0 : table[0].size()  ; 
    }

    void ReservationTable::reserve(int loc, int t,int agent_id) {
        if (loc >= 0 && loc < table.size() && t >= 0 && t < table[loc].size()) {
            table[loc][t] = agent_id;
        }
    }

    void ReservationTable::unreserve(int loc, int t) {
        if (loc >= 0 && loc < table.size() && t >= 0 && t < table[loc].size()) {
            table[loc][t] = -1;
        }
    }

    int ReservationTable::get(int loc, int t) {
        if (loc >= 0 && loc < table.size() && t >= 0 && t < table[loc].size()) {
            return table[loc][t];
        }

        return -1;
    }

    bool ReservationTable::is_free(int loc, int t) {
        return get(loc,t) == -1;
    }

    bool ReservationTable::can_move(int from, int to, int t) {

        if (!is_free(to, t+1)) return false;

        return true;
    }

    void ReservationTable::clear() {
        for (auto& col : table){
            for (auto& cell : col){
                cell = -1;
            }
        }
    }
    
}