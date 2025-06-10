#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "DialogSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  DialogSkill stateMachine("Dialog");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

