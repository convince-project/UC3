/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "yarp/sig/Sound.h"

#include "VerbalOutputBatchReader.hpp"

YARP_LOG_COMPONENT(VERBAL_OUTPUT_BATCH_READER, "convince.narrate_component.VerbalOutputBatchReader")

void VerbalOutputBatchReader::onRead(yarp::sig::Sound &msg)
{
    m_audioQueue->push(msg);
    yCInfo(VERBAL_OUTPUT_BATCH_READER) << "[VerbalOutputBatchReader::onRead] Received audio message. Queue size is now: " << m_audioQueue->size();
}
