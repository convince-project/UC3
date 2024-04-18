#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "VisitorsFollowingRobotSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  VisitorsFollowingRobotSkill stateMachine("VisitorsFollowingRobot");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

