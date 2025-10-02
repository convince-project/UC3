#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "GoToChargingStationSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  GoToChargingStationSkill stateMachine("GoToChargingStation");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "GoToChargingStationSkill successfully closed" << std::endl;
  return ret;
}

