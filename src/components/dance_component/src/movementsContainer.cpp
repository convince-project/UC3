#include "movementsContainer.h"


MovementsContainer::MovementsContainer(std::set<std::string> partNames, std::map<std::string, Dance> dances) : m_partNames(std::move(partNames)),
                                                                                                               m_dances(std::move(dances))
{
}

std::set<std::string> &MovementsContainer::GetPartNames()
{
    return m_partNames;
}

std::map<std::string, Dance> &MovementsContainer::GetDances()
{
    return m_dances;
}

bool MovementsContainer::GetDance(const std::string &danceName, Dance &outDance) const
{
    auto foundDance = m_dances.find(danceName);
    if (foundDance != m_dances.end())
    {
        outDance = foundDance->second;
        return true;
    }
    return false;
}
