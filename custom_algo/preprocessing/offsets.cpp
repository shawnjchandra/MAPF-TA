#include "offsets.h"


namespace CustomAlgo{
    std::vector<int> generateOffset(int r, int cols) {
        std::vector<int> offsets;
        for (int dr = -r ; dr <= r ; dr++) {
            for (int dc = -r ; dc <= r ; dc++){
                if (dr*dr + dc* dc <= r*r) offsets.push_back(dr*cols+dc);
            }
        }
        return offsets;
    }
}