#pragma once

#include <string>

class TeamPosition {
public:

    std::string team;
    std::string position;

    std::string asString() {
        return team + "_" + position;
    }
};

namespace selector {

    inline TeamPosition auton;

    void init();

}