#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetTurnedSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetTurnedSkill stateMachine("SetTurned");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SetTurnedSkill successfully closed" << std::endl;
  return ret;
}

