#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsPoiDone0Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsPoiDone0Skill stateMachine("IsPoiDone0");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsPoiDone0Skill successfully closed" << std::endl;
  return ret;
}

