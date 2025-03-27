
#ifndef BEHAVIOR_TOUR_ROBOT_TOUR_STORAGE_H
#define BEHAVIOR_TOUR_ROBOT_TOUR_STORAGE_H

#include "Tour.h"
#include <fstream>
#include <iostream>
#include <string>
#include <map>

#include "nlohmann/json.hpp"

class TourStorage
{
public:
    TourStorage() {}
    ~TourStorage() {}

    Tour m_loadedTour; // The Tour object that was loaded
    // static bool GetInstance(char* pathJSONTours, char* tourName, std::shared_ptr<TourStorage> instance_passed);

    TourStorage(const TourStorage &) = delete;
    TourStorage &operator=(const TourStorage &) = delete;
    TourStorage(TourStorage &&) = delete;
    TourStorage &operator=(TourStorage &&) = delete;

    nlohmann::ordered_json ReadFileAsJSON(const std::string &path);
    bool WriteJSONtoFile(const nlohmann::ordered_json &j, const std::string &path);
    bool LoadTour(const std::string &pathTours, const std::string &tourName);
    Tour &GetTour();
};

#endif // BEHAVIOR_TOUR_ROBOT_TOUR_STORAGE_H
