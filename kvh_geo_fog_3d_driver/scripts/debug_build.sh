#!/bin/bash

colcon build --packages-select kvh_geo_fog_3d_driver --cmake-args -DCMAKE_BUILD_TYPE=Debug
gdb -tui ../../../build/kvh_geo_fog_3d_driver/kvh_geo_fog_3d_driver_node