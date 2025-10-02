#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsMaximumDurationSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsMaximumDurationSkill stateMachine("IsMaximumDuration");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsMaximumDurationSkill successfully closed" << std::endl;
  return ret;
}

