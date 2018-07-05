#!/bin/sh

cd $GAMS_ROOT
mvn -f port/java versions:set -DnewVersion=`cat VERSION.txt`
mvn -f port/java -Dmadara.version=`cat $MADARA_ROOT/VERSION.txt` -P release clean package deploy

