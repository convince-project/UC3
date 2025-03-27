#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "NetworkStatusChangedSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  NetworkStatusChangedSkill stateMachine("NetworkStatusChanged");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

