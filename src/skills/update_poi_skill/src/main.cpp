#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "UpdatePoiSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  UpdatePoiSkill stateMachine("UpdatePoi");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "UpdatePoiSkill successfully closed" << std::endl;
  return ret;
}

