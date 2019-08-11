#!/bin/bash

if [ -z "${1}" ]; then
  echo USAGE: $0 path_to_catkin_ws
  exit
fi



for directory in `pwd`/../catkin_ws/src/*; do
  ln -s "${directory}" "${1}"
done
