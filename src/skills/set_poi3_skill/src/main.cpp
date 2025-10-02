#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetPoi3Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetPoi3Skill stateMachine("SetPoi3");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SetPoi3Skill successfully closed" << std::endl;
  return ret;
}

