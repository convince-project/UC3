#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsAllowedToMoveSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsAllowedToMoveSkill stateMachine("IsAllowedToMove");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsAllowedToMoveSkill successfully closed" << std::endl;
  return ret;
}

