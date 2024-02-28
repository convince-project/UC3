#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <string>

#include <iostream>
#include "StateMachineToTestSM.h"
#include "TestStateMachineDataModel.h"

#include <thread>
#include <chrono>

void startApp(StateMachineToTestSM* stateMachineToTest) {
  std::string eventName;
  for (const auto& state : stateMachineToTest->activeStateNames()) {
      std::cout  << "active state: " << state.toStdString() << std::endl;    
  }  
  do
  {
    std::cin >> eventName;

    if (eventName == "CMD_START"){
      std::cout << "sending event CMD_START" << std::endl;
      stateMachineToTest->submitEvent("CMD_START");
    } else if (eventName == "CMD_ALARM") {
      std::cout << "sending event CMD_ALARM" << std::endl;
      stateMachineToTest->submitEvent("CMD_ALARM");
    } else if (eventName == "CMD_HALT") {
      std::cout << "sending event CMD_HALT" << std::endl;
      stateMachineToTest->submitEvent("CMD_HALT");
    } 
    std::this_thread::sleep_for (std::chrono::milliseconds(200));

    for (const auto& state : stateMachineToTest->activeStateNames()) {
        std::cout  << "active state: " << state.toStdString() << std::endl;    
    }  
  } while (eventName != "exit");
}

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StateMachineToTestSM stateMachineToTest;
  TestStateMachineDataModel dataModel;
  stateMachineToTest.setDataModel(&dataModel);
  stateMachineToTest.start();

  stateMachineToTest.connectToEvent("SEND_OUTSIDE", [](const QScxmlEvent & event){
    std::cout << event.data().isNull() << std::endl;
    std::cout << event.data().toMap()["state"].toString().toStdString() << std::endl;
    std::cout << "received!" << std::endl;
  });

  auto m_threadSpin = std::make_shared<std::thread>(startApp, &stateMachineToTest);
  app.exec();


    
  return 0;
  
}

