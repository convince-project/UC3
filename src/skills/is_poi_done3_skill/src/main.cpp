#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsPoiDone3Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsPoiDone3Skill stateMachine("IsPoiDone3");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsPoiDone3Skill successfully closed" << std::endl;
  return ret;
}

