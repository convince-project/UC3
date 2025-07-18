#include "movementStorage.h"

// YARP_LOG_COMPONENT(MOVEMENT_STORAGE, "behavior_tour_robot.aux_modules.movementstorage", yarp::os::Log::TraceType)


MovementStorage &MovementStorage::GetInstance(const std::string &pathJSONMovements)
{
    static MovementStorage instance; // Guaranteed to be destroyed. Instantiated on first use.
    instance.LoadMovements(pathJSONMovements);
    return instance;
}

bool MovementStorage::LoadMovements(const std::string &pathJSONMovements)
{
    // Load movements
    nlohmann::ordered_json movements_json = ReadFileAsJSON(pathJSONMovements);
    if(pathJSONMovements == "" || movements_json == nlohmann::ordered_json())
    {
        return false;
    }
    MovementsContainer loadedMovements = movements_json.get<MovementsContainer>();
    m_movementsContainer = loadedMovements;
    // yCInfo(MOVEMENT_STORAGE) << "Loaded:" << m_movementsContainer.GetPartNames().size() << "robot parts.";
    // yCInfo(MOVEMENT_STORAGE) << "Loaded:" << m_movementsContainer.GetDances().size() << "dances.";

    for (auto &dance : m_movementsContainer.GetDances()) // Needs to be done here as json parser calls the default contructor and copies variables
    {
        dance.second.UpdateDuration();
    }

    return true;
}

nlohmann::ordered_json MovementStorage::ReadFileAsJSON(const std::string &path)
{
    std::cout << "Reading file: " << path << std::endl;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file" << std::endl;
        return nlohmann::ordered_json();
    }
    std::string sentence = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if(sentence == "")
    {
        std::cerr << "File is empty" << std::endl;
        return nlohmann::ordered_json();
    }
    return nlohmann::ordered_json::parse(sentence);
}

MovementsContainer &MovementStorage::GetMovementsContainer()
{
    return m_movementsContainer;
}

