#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "NotifyChargedSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  NotifyChargedSkill stateMachine("NotifyCharged");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "NotifyChargedSkill successfully closed" << std::endl;
  return ret;
}

