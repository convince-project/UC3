#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsPoiDone5Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsPoiDone5Skill stateMachine("IsPoiDone5");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsPoiDone5Skill successfully closed" << std::endl;
  return ret;
}

