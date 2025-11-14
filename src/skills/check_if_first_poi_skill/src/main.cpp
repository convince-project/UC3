#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "CheckIfFirstPoiSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  CheckIfFirstPoiSkill stateMachine("CheckIfFirstPoi");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "CheckIfFirstPoiSkill successfully closed" << std::endl;
  return ret;
}

