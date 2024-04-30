#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "NarratePoiSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  NarratePoiSkill stateMachine("Narrate");
  stateMachine.start(argc, argv);

  int ret=app.exec();

  return ret;

}

