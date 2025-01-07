#!/bin/bash

#if there are no arguments, then assume we want an interactive shell
if [ $# -eq 0 ]; then
    # this is the docker command to run
    docker run -it -v $(pwd):/root -w /root \
        snidercs/bot-2024 
# otherwise, assume we want to run a command
else
    # this is the docker command to run
    docker run -it -v $(pwd):/root -w /root \
        snidercs/bot-2024 $@
fi
