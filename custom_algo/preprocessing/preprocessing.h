#pragma once

#include "hpa.h"
#include "voronoi.h"
#include "offsets.h"
#include "degree_map.h"
#include "SharedEnv.h"

class Preprocessing {
public:
    SharedEnvironment* env;

    Preprocessing(SharedEnvironment* env):env(env){};
    Preprocessing(){env = new SharedEnvironment();};
    virtual void initialize(int preprocess_time_limit);
}