#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "SayByeSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SayByeSkill stateMachine("SayBye");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

