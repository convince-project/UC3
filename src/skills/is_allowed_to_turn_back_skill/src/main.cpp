#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsAllowedToTurnBackSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsAllowedToTurnBackSkill stateMachine("IsAllowedToTurnBack");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsAllowedToTurnBackSkill successfully closed" << std::endl;
  return ret;
}

