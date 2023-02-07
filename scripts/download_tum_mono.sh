#!/bin/bash

if [ -z "$SEQUENCE" ]
then
    echo "Set \$export SEQUENCE=<dataset sequence number> before running"
    exit
fi

apt install wget zip  # Assume this is on a docker container

wget -c http://vision.in.tum.de/mono/dataset/sequence_${SEQUENCE}.zip -P data
unzip -o data/sequence_${SEQUENCE}.zip -d data
unzip -o data/sequence_${SEQUENCE}/images.zip -d data/sequence_${SEQUENCE}/images/
