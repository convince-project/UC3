# Code for the execution of use case 3 for the [convince project](https://convince-project.eu)

## run the simulation with docker

first run the docker 

```
sudo xhost + 
docker run --rm -it --privileged --network host --pid host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1  ste93/convince:tour_sim_ubuntu_22.04_qt_6.8.3_iron_devel
```


to run the simulation, first you need to change directory to tour guide robot:

```
cd /usr/local/src/robot/tour-guide-robot/app/navigation2/scripts/
```

and then start the simulation:

```
./start_sim_madama.sh
```

after this run all the navigation stack from the **Navigation_ROS2_R1_Madama_SIM** app

once the navigation is done, you need to run the behavior tree from the app

first launch a server for yarp run on the docker terminal:

``` 
yarp run --server /bt --log
```

then launch the various files from the application **convince_bt.xml**

## compile the docker with ROS2 IRON

to compile the docker with ros2 iron:
```
cd UC3/docker/; docker build -t ste93/convince:tour_sim_ubuntu_22.04_qt_6.8.3_iron_devel  -f Dockerfile.bt_iron --build-arg base_img=ste93/r1images:tourCore2_ubuntu22.04_iron_stable_qt_6.8.3 .
```
<!-- 
<div align="center">
  Journal, vol. X, no. y, pp. abc-def, Month Year
</div>



<!-- ## Table of Contents

- [Update](#updates)
<!-- - [Installation](#installation) 
- [running the code](#running-the-code)
- [Reproduce the results](#reproduce-the-paper-results)
- [Run the code with custom data](#run-the-code-with-custom-data-optional)
- [License](#license)
- [Citing this paper](#citing-this-paper)

## Updates


### Execution inside a container (alternative)

look in the specific folders -->
<!-- ## Reproduce the paper results

Before running the experiments, it is suggested to run the following sanity checks to make sure that the environment is properly configure:

```console
<all the instructions required to check that the environent has been configured properly>
```

Instructions for reproducing the experiments:

```console
<all the instructions required to reproduce the results>
```

Adding an example of the expected outcome might be useful.

## Run the code with custom data (optional)

Adding information on the structure of the input data and how it gets processed might be useful.

```console
<all the instructions required to run your code on custom data>
```
-->
<!-- ## License

Information about the license.

:warning: Please read [these](https://github.com/hsp-iit/organization/tree/master/licenses) instructions on how to license HSP code. -->

<!-- ## Citing this paper

```bibtex
@ARTICLE{9568706,
author={Author A, ..., Author Z},
journal={Journal},
title={Title},
year={Year},
volume={X},
number={y},
pages={abc-def},
doi={DOI}
}
``` -->

## Maintainer

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="assets/image.png" width="40">](https://github.com/hsp-iit) | [@hsp-iit](https://github.com/hsp-iit) |

## Prerequisites
The source code is heavily based on the concept of ROS2 services and actions, if you are not familiar with this subject, here are some useful references to get started:
- [Understanding services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Understanding actions](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [Ros2 Interfaces](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html)
- [Writing a simple C++ service and client](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
- [Writing an action server and client (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [Writing a simple Python service and client](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Writing an action server and client (Python)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)

## Structure of the code
The software architecture is composed by 3 main software entities:
- [**Components**](src/components/): software entities which collect a series of ROS2 services. Components are responsible to directly interact with the environment and to manage the actual computational load. Components are actually not monitorable, therefore they fail silently. In order to maintain a log in the component execution, there come to hand skills, which act as interfaces between components and the rest of the system. Components are supposed to implement the functional logic of the services, therefore they should execute the majority of the computational load of the functionality. 
- [**Skills**](src/skills/): software entities that reflect the processing logic of each leaf inside the main behavior tree. Each skill is characterized by its state machine, that represents the functional pipeline of the leaf behavior. The main responsibility of each skill is acting as an intermediate actor between its own state machine and the components. Analyzing the scheme bottom-up, in component-skill communication, skills implement ROS2 nodes that act as service clients and communicate with components that implement ROS2 service servers. Here components are responsible to provide the computational logic, while skills act as controllers, allowing to monitor the state of the pipeline. In skill-state machine communication, skills directly interact with their state machine, which is an attribute of the skill. The state machine is responsible to manage the state of the skill and to provide the logic for the transitions between states. The purpose of the skills is monitoring the state of the pipeline thanks to their state machine, and to accordingly interact with the components to provide the correct service for that specific state. As opposed to the components which manage the majority of the computational load, the skills should be as light as possible. Their main purpose is just monitoring the state of the execution. 
- [**Interfaces**](src/interfaces/): For each component implementing service servers/clients and action servers/client, there are respectively a `srv` and `action` folders defining the interface type. If you are not familiar with this concepts, take a look to the ROS2 Interfaces in the [Prerequisites](#prerequisites) section. The interfaces are used to define the messages exchanged between the components and the skills.

## Description of the functionalities
The code is structured in a modular way, allowing to easily add new functionalities. All the functionalities are characterized by dedicated components, skills and interfaces. The main functionalities are:
- **Dialog**: the dialog functionality is composed by the dialog component, the dialog skill and the dialog interfaces. You can check the dialog skill [README](dialog.md) for more information.
