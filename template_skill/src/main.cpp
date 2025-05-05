#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "$className$.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  $className$ stateMachine("$skillName$");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

