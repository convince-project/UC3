#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsWarningDurationSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsWarningDurationSkill stateMachine("IsWarningDuration");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsWarningDurationSkill successfully closed" << std::endl;
  return ret;
}

