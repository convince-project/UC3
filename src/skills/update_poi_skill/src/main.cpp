#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "UpdatePoiSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  UpdatePoiSkill stateMachine("UpdatePoi");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

