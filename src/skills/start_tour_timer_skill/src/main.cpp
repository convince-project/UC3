#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "StartTourTimerSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StartTourTimerSkill stateMachine("StartTourTimer");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "StartTourTimerSkill successfully closed" << std::endl;
  return ret;
}

