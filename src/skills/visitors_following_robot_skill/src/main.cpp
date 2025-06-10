#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "VisitorsFollowingRobotSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  VisitorsFollowingRobotSkill stateMachine("VisitorsFollowingRobot");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

