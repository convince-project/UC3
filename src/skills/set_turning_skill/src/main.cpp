#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetTurningSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetTurningSkill stateMachine("SetTurning");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SetTurningSkill successfully closed" << std::endl;
  return ret;
}

