#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "ResetTourSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  ResetTourSkill stateMachine("ResetTour");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

