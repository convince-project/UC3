#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "StopPoiTimerSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StopPoiTimerSkill stateMachine("StopPoiTimer");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "StopPoiTimerSkill successfully closed" << std::endl;
  return ret;
}

