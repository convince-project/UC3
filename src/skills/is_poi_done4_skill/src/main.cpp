#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsPoiDone4Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsPoiDone4Skill stateMachine("IsPoiDone4");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsPoiDone4Skill successfully closed" << std::endl;
  return ret;
}

