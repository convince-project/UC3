#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <string>

#include <iostream>
#include "StateMachineToTest.h"
#include "TestStateMachineDataModel.h"

#include <thread>
#include <chrono>




int main(int argc, char *argv[])
{
  StateMachineToTest stateMachineToTest;
  std::string eventName;
  TestStateMachineDataModel dataModel;
    stateMachineToTest.setDataModel(&dataModel);
      stateMachineToTest.start();

  stateMachineToTest.connectToEvent("SEND_OUTSIDE", [](const QScxmlEvent &){
    std::cout << "received!";

  });
  do
  {
    std::cin >> eventName;

    if (eventName == "CMD_START"){
      stateMachineToTest.submitEvent("CMD_START");
    } else if (eventName == "CMD_ALARM") {
      stateMachineToTest.submitEvent("CMD_ALARM");
    } else if (eventName == "CMD_HALT") {
      stateMachineToTest.submitEvent("CMD_HALT");
    } 
  } while (eventName != "exit");
    
  return 0;
  
}

