
#ifndef BEHAVIOR_TOUR_ROBOT_MOVEMENT_STORAGE_H
#define BEHAVIOR_TOUR_ROBOT_MOVEMENT_STORAGE_H

#include <set>
#include <map>
#include <nlohmann/json.hpp>
#include <movement.h>
#include <dance.h>
#include <fstream>
#include <iostream>
#include "movementsContainer.h"

using json = nlohmann::json;

class MovementStorage
{
private:

    MovementsContainer m_movementsContainer;

public:
    MovementStorage() {}
    ~MovementStorage() {}
    static MovementStorage &GetInstance(const std::string &pathJSONMovements);

    MovementStorage(const MovementStorage &) = delete;
    MovementStorage &operator=(const MovementStorage &) = delete;
    MovementStorage(MovementStorage &&) = delete;
    MovementStorage &operator=(MovementStorage &&) = delete;

    nlohmann::ordered_json ReadFileAsJSON(const std::string &path);
    bool LoadMovements(const std::string &pathJSONMovements);
    MovementsContainer &GetMovementsContainer();
};

#endif // BEHAVIOR_TOUR_ROBOT_MOVEMENT_STORAGE_H
