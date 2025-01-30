#!/bin/bash

CABOT_SITE=$1

if [[ -z $CABOT_SITE ]]; then
    echo "Usage:"
    echo " $0 <cabot_site pkg name>"
    exit
fi

echo "build $CABOT_SITE package"

docker run --rm \
    -v ./pkg:/opt/build_ws/pkg \
    -v ./${CABOT_SITE}:/opt/build_ws/src/${CABOT_SITE} \
    ros:humble \
    /bin/sh -c "\
        apt update && \
        apt install -y zip && \
	. /opt/ros/humble/setup.sh && \
	cd /opt/build_ws && colcon build && \
	cd install && \
	zip -qr ../pkg/${CABOT_SITE}.zip ${CABOT_SITE} && \
        chown $UID:$UID ../pkg/${CABOT_SITE}.zip \
	"
