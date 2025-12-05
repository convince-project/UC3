#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "PlanSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  PlanSkill stateMachine("Plan");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "PlanSkill successfully closed" << std::endl;
  return ret;
}

