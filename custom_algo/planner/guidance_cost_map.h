#pragma once
#include "SharedEnv.h"

namespace CustomAlgo {
    void gcm_update(SharedEnvironment* env ,int loc, int orient, int curr_timestep);
    void gcm_cooldown(SharedEnvironment* env, int curr_timestep);
}