#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsAtCurrentPoiSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsAtCurrentPoiSkill stateMachine("IsAtCurrentPoi");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsAtCurrentPoiSkill successfully closed" << std::endl;
  return ret;
}

