#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetPoi4Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetPoi4Skill stateMachine("SetPoi4");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SetPoi4Skill successfully closed" << std::endl;
  return ret;
}

