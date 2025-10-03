#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetPoi1Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetPoi1Skill stateMachine("SetPoi1");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SetPoi1Skill successfully closed" << std::endl;
  return ret;
}

