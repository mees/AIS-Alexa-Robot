# AIS Alexa Robot Skill
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This repository contains an Alexa skill to control and interact with a robot via [ROS](https://www.ros.org/) topics. Concretely, the current skill implementation was used for picking-and-placing tabletops objects from natural language. More information at the [project page](http://speechrobot.cs.uni-freiburg.de/).

## Reference
If you find the code helpful please consider citing our work
```
@inproceedings{mees21iser,
  author = {Oier Mees and Wolfram Burgard},
  title = {Composing Pick-and-Place Tasks By Grounding Language},
  booktitle = {Proceedings of the International Symposium on Experimental Robotics (ISER)},
  year = 2021,
  address = {La Valletta, Malta}
}
```
```
@INPROCEEDINGS{mees20icra_placements,
author = {Oier Mees and Alp Emek and Johan Vertens and Wolfram Burgard},
title = {Learning Object Placements For Relational Instructions by Hallucinating Scene Representations},
booktitle = {Proceedings of the IEEE International Conference on Robotics and Automation  (ICRA)},
year = 2020,
address = {Paris, France}
}
```
## Usage

First, launch ``rosbridge_server``:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
In a separate shell, make your local port visible for the VM
```
ssh -R 9090:localhost:9090 aisalexa /opt/aisalexa/forward_port.sh
```
In case your local server does not run on port 9090, replace the first port number only.

There will be three ros topics: ``/alexa/command``, ``alexa/debug`` and ``alexa/reply``. See
[test_response.py](test_response.py) for examplary handling of commands.

## Start server

Start all containers
* ``cd /opt/aisalexa/nginx``
* ``sudo ./start.sh``

Show container logs
* ``cd /opt/aisalexa/nginx``
* ``sudo docker-compose logs aisalexa`` (leave out the service name to get the logs of all services)



## Setup

see [Setup](./setup.md)

For configuration, change the environment variables in ``nginx/.env``.

Update configuration by using ``git pull``. When running ``start.sh``,
updated images (i.e., the updated skill) are retrieved automatically.
Remember that changes to the skill must be pushed to ``aisgit`` for
the updated docker container to be built

## License
For academic usage, the code is released under the [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html) license. For any commercial purpose, please contact the authors.

## Authorship
This code was developed by [Henrich Kohlkhorst](http://www2.informatik.uni-freiburg.de/~henkolk/) and [Oier Mees](http://www.oiermees.com) at the University of Freiburg, Germany.
