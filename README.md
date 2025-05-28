# Code for the execution of use case 3 for the [convince project](https://convince-project.eu)

## run the simulation with docker

first run the docker 

```
sudo xhost + 
docker run --rm -it --privileged --network host --pid host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1  ste93/convince:ubuntu_22.04_qt_6.8.3_sim
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
git checkout iron_docker;
cd UC3/docker/; docker build -t ste93/convince:ubuntu_22.04_qt_6.8.3_sim  -f Dockerfile.bt --build-arg base_img=ste93/r1images:tourCore2_ubuntu22.04_iron_stable_qt_6.8.3 .
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

