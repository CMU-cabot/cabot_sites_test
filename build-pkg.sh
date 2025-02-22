#!/bin/bash

# set CABOT_SITE to the current dir name
CABOT_SITE=$1
VERSION=$2

if [[ -z $VERSION ]]; then
    echo "Usage:"
    echo " $0 <cabot_site pkg version>"
    exit
fi

echo "build $CABOT_SITE package version $VERSION"

# check if ${VERSION} (${VERSION#v}) is matched with version in $CABOT_SITE/package.xml
if ! grep -q "<version>${VERSION#v}</version>" ./${CABOT_SITE}/package.xml; then
    echo "Version ${VERSION#v} is not matched with version in ${CABOT_SITE}/package.xml: $(grep '<version>' ./${CABOT_SITE}/package.xml)"
    # make a tmpfile
    tmpfile=$(mktemp)
    sed "s|<version>.*</version>|<version>${VERSION#v}</version>|" ./${CABOT_SITE}/package.xml > $tmpfile
    mv $tmpfile ./${CABOT_SITE}/package.xml
    echo "Version updated to $(grep '<version>' ./${CABOT_SITE}/package.xml)"
    exit 1
fi

docker run --rm \
    -v ./pkg:/opt/build_ws/install \
    -v ./${CABOT_SITE}:/opt/build_ws/src/${CABOT_SITE} \
    ros:humble \
    /bin/sh -c "\
        . /opt/ros/humble/setup.sh && \
        cd /opt/build_ws && colcon build \
    "

zip -qr ./pkg/${CABOT_SITE}-${VERSION}.zip ./pkg/${CABOT_SITE}

# delete all files and dirs in pkg dir except the zip file and .gitignore (this is for test build)
find ./pkg -mindepth 1 -maxdepth 1 -type d -exec rm -r {} \;
find ./pkg -mindepth 1 -maxdepth 1 -type f ! -name "*-${VERSION}.zip" ! -name .gitignore -exec rm {} \;
