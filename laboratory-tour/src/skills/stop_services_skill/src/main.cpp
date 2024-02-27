#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "StopServicesSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StopServicesSkill stateMachine("StopServicesSkill");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

