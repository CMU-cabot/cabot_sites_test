#!/bin/bash

## $sitedir is ros package directory

localizer=multicart

map=$sitedir/maps/test-room.yaml
anchor=
world=$sitedir/worlds/test-room.world
global_map_name=map_global

if [ $gazebo -eq 1 ]; then
    wireless_config=$sitedir/worlds/test_wireless.yaml
fi
