#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

docker build -f $DIR/Dockerfile -t bluerov2_sitl:latest ..
#docker build --no-cache -f $DIR/Dockerfile -t bluerov2_sitl:latest ..



