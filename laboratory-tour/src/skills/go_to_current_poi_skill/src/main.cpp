#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "GoToCurrentPoiSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  GoToCurrentPoiSkill stateMachine("GoToCurrentPoi");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

