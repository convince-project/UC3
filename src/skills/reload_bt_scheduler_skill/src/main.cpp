#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "ReloadBtSchedulerSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  ReloadBtSchedulerSkill stateMachine("ReloadBtScheduler");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "ReloadBtSchedulerSkill successfully closed" << std::endl;
  return ret;
}

