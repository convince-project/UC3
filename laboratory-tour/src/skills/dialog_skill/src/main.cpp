#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "DialogSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  DialogSkill stateMachine("Dialog");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}
