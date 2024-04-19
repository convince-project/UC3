#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "NarrateSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  NarrateSkill stateMachine("Narrate");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

