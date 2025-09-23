#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "StartPoiTimerSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StartPoiTimerSkill stateMachine("StartPoiTimer");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "StartPoiTimerSkill successfully closed" << std::endl;
  return ret;
}

