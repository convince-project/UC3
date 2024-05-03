#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "NetworkUpSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  NetworkUpSkill stateMachine("NetworkUp");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

