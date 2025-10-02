#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetPoi5Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetPoi5Skill stateMachine("SetPoi5");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SetPoi5Skill successfully closed" << std::endl;
  return ret;
}

