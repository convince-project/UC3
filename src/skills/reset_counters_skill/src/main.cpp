#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "ResetCountersSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  ResetCountersSkill stateMachine("ResetCounters");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "ResetCountersSkill successfully closed" << std::endl;
  return ret;
}

