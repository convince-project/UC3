#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "StopAndTurnBackSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StopAndTurnBackSkill stateMachine("StopAndTurnBack");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

