# Setup

The basic setup is based on https://github.com/evertramos/docker-compose-letsencrypt-nginx-proxy-companion

## Basic installation in VM

* ``sudo apt install socat``
* install Docker https://docs.docker.com/install/linux/docker-ce/ubuntu/#set-up-the-repository
* install Docker compose https://docs.docker.com/compose/install/#install-compose

## Initial setup

* ``docker login aisdocker.informatik.uni-freiburg.de``
* ``sudo mkdir -p /opt/nginx_data``
* ``cd /opt && git clone https://aisgit.informatik.uni-freiburg.de/henkolk/aisalexa.git``

To setup the systemd unit

* ``sudo ln -s /opt/aisalexa/nginx/nginx_aisalexa_docker_compose.service /etc/systemd/system``
* ``sudo systemctl daemon-reload``
* ``sudo systemctl start nginx_aisalexa_docker_compose.service``
* Check the status with ``sudo systemctl status nginx_aisalexa_docker_compose.service``
* Enable autostart with ``sudo systemctl enable nginx_aisalexa_docker_compose.service``

## Background for network setup

All docker containers run in a separate bridge network (``webproxy``). This should be ``172.18.0.*`` (the default bridge network has ``172.17.0.*``). From the container, we can access host ports by the first IP. We could also extract this programmatically based on the network config in the container using ``/sbin/ip route | awk '/default/ { print $3 }' | awk '!seen[$0]++'``

If we forward a port using ``ssh -R ...``, it is only bound to the loopback interface (``localhost``/``127.0.0.1``), but not to the other ip addresses, especially not to the host's address in the docker network (``webproxy``).

Hence, we run ``socat TCP4-LISTEN:9000,fork,range=172.18.0.1/16 TCP4:localhost:8080`` as a workaround to forward all incoming connections on port 9000 (from the container network) to the forwarded SSH port.

Hence, we have the following setup

```
ROS host                 aisalexa                 aisalexa                         alexa-skill container

localhost:9090 <-------- localhost:9090 <-------- 172.18.0.1:9000 <--------------- 172.18.0.1:9000
                (ssh -R)                  (socat)                  (docker bridge)
```

If you want to debug the network config in the container, you can run a container in the same network, e.g. using ``sudo docker run --network webproxy --rm -it aisdocker.informatik.uni-freiburg.de/henkolk/aisalexa:master /bin/bash``.