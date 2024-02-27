#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "StartServicesSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StartServicesSkill stateMachine("StartServicesSkill");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

