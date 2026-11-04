#pragma once

#include <string>

class TeamPosition {
public:

    std::string team;
    std::string position;

    std::string asString() {
        return team + "_" + position;
    }

    void setTeamPosition(std::string team, std::string position) {
        this->team = team;
        this->position = position;
    }
};

namespace selector {

    inline TeamPosition auton;

    void init();

}