

#include <iostream>
#include "CheckNetworkComponent.h"

#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
  
  CheckNetworkComponent component;
  component.set_name("CheckNetworkComponent");
  component.setup(argc, argv);
  component.start();

  return true;

}

