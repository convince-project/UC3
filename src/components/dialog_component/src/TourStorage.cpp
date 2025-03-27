#include "TourStorage.h"


// static bool GetInstance(char* pathJSONTours, char* tourName, std::shared_ptr<TourStorage> instance_passed)
// {
//     static std::shared_ptr<TourStorage> instance = std::make_shared<TourStorage>(); // Guaranteed to be destroyed. Instantiated on first use.
//     if (!instance->LoadTour(pathJSONTours, tourName))
//     {
//         return false;
//     }
//     instance_passed = instance;
//     return true;

// }

nlohmann::ordered_json TourStorage::ReadFileAsJSON(const std::string &path)
{
    std::ifstream file(path);
    std::string sentence = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return nlohmann::ordered_json::parse(sentence);
}

bool TourStorage::WriteJSONtoFile(const nlohmann::ordered_json &j, const std::string &path)
{
    try
    {
        std::string str = j.dump(4);
        std::ofstream out(path);
        out << str;
        out.close();
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

bool TourStorage::LoadTour(const std::string &pathTours, const std::string &tourName)
{

    // Load tour
    nlohmann::ordered_json tours_json = ReadFileAsJSON(pathTours);
    std::unordered_map<std::string, Tour> tours = tours_json.get<std::unordered_map<std::string, Tour>>();
    auto foundTour = tours.find(tourName);
    if (foundTour != tours.end())
    {
        m_loadedTour = foundTour->second;
    }
    else
    {
        return false;
    }
    return true;
}

Tour &TourStorage::GetTour()
{
    return m_loadedTour;
}
