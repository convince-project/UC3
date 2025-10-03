#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetPoi0Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetPoi0Skill stateMachine("SetPoi0");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SetPoi0Skill successfully closed" << std::endl;
  return ret;
}

