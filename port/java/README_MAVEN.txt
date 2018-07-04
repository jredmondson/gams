To generate jar files for development purpose:

* mvn -f port/java versions:set -DnewVersion=`cat VERSION.txt`
* mvn -f port/java -Dmadara.version=`cat $MADARA_ROOT/VERSION.txt` -P development clean package install



To release the artifacts in Maven repository or any internal maven repository:

* mvn -f port/java versions:set -DnewVersion=`cat VERSION.txt`
* mvn -f port/java -Dmadara.version=`cat $MADARA_ROOT/VERSION.txt` -P release clean package deploy
