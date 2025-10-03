#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "ResetTourAndFlagsSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  ResetTourAndFlagsSkill stateMachine("ResetTourAndFlags");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "ResetTourAndFlagsSkill successfully closed" << std::endl;
  return ret;
}

