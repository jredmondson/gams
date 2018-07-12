#!/bin/sh

DIST_DIR=.

VERSION="$(cat $GAMS_ROOT/VERSION.txt)"
MADARA_VERSION="$(cat $MADARA_ROOT/VERSION.txt)"
PACKAGE_NAME="gams"

LIB_VERSION="$(grep -oE '[0-9]+\.[0-9]+\.[0-9]+' $GAMS_ROOT/VERSION.txt)"
ARCHITECTURE=amd64
ROOT_DIR=usr
REVISION=1
INCLUDE_ALL_BIN=0
DEPENDS="build-essential (>= 12.1), subversion (>= 1.9.3-2), git-core (>= 1:2.7.4-0), perl (>= 5.22.1-9), libboost-all-dev (>= 1.58.0.1)"
DEPENDS_ROS=0
DEPENDS_CLANG=0
DEPENDS_JAVA=0
PRINT_USAGE=0

until [ -z "$1" ] ;
do
  if [ "$1" = "--help" -o "$1" = "-h" -o $PRINT_USAGE -eq 1 ] ;
  then
    echo "Packager options:"
    echo "  --lib-version VER    GAMS so version (def $VERSION)"
    echo "  --arch        ARCH   architecture type (e.g. i386, x64, all, etc.)"
    echo "  --root        DIR    root directory to install to (e.g. usr)"
    echo "  --revision    REV    revision number (1 by default)"
    echo "  --include-all-bin    include all files in $GAMS_ROOT/bin"
    echo "  --clang              package depends on clang"
    echo "  --java               package depends on java"
    echo "  --ros                package depends on ros"
    exit 1
  elif [ "$1" = "--lib-version" ] ;
  then
    shift
    echo "Setting LIB_VERSION to $1"
    LIB_VERSION=$1
    shift
  elif [ "$1" = "--arch" ] ;
  then
    shift
    echo "Setting ARCHITECTURE to $1"
    ARCHITECTURE=$1
    shift
  elif [ "$1" = "--root" ] ;
  then
    shift
    echo "Setting ROOT_DIR to $1"
    ROOT_DIR=$1
    shift
  elif [ "$1" = "--revision" ] ;
  then
    shift
    echo "Setting REVISION to $1"
    REVISION=$1
    shift
  elif [ "$1" = "--include-all-bin" ] ;
  then
    echo "Including all bins in GAMS_ROOT/bin"
    INCLUDE_ALL_BIN=1
    shift
  elif [ "$1" = "--clang" ] ;
  then
    echo "Marking clang dependencies"
    DEPENDS="$DEPENDS, clang-5.0 (>= 1:5.0-3~16.04.1), libc++-dev (>= 3.7.0-1), libc++abi-dev (>= 3.7.0-1)"
    DEPENDS_CLANG=1
    shift
  elif [ "$1" = "--java" ] ;
  then
    echo "Marking java dependencies"
    DEPENDS="$DEPENDS, oracle-java8-set-default (>= 8u171-1~webupd8~0), maven (>= 3.3.9-3)"
    DEPENDS_JAVA=1
    shift
  elif [ "$1" = "--ros" ] ;
  then
    echo "Marking ros dependencies"
    DEPENDS="$DEPENDS, ros-kinetic-desktop-full (>= 1.3.2-0), python-rosinstall (>= 0.7.8-1), ros-kinetic-ros-type-introspection (>= 1.1.1-0), ros-kinetic-move-base-msgs (>= 1.13.0-0), ros-kinetic-navigation (>= 1.14.4-0), libactionlib-dev (>= 1.11.4-2), libactionlib-msgs-dev (>= 1.12.3-5), libmove-base-msgs-dev (>= 1.13.0-1)"
    DEPENDS_ROS=1
    shift
  else
    PRINT_USAGE=1
    shift;
  fi

done

sudo apt-get install dpkg-dev dh-make debhelper devscripts pbuilder fakeroot dos2unix

echo "VERSION is $VERSION"
echo "ARCHITECTURE is $ARCHITECTURE"
echo "Generating debian packaging into $DIST_DIR"

# copy source to local directories
cd $DIST_DIR

PACKAGE_DIR="${VERSION}-${REVISION}_${ARCHITECTURE}"

MADARA_REQS="madara"

if [ $DEPENDS_CLANG -eq 1 ] ; then
  MADARA_REQS="${MADARA_REQS}-clang"
  PACKAGE_NAME="${PACKAGE_NAME}-clang"
fi

if [ $DEPENDS_JAVA -eq 1 ] ; then
  MADARA_REQS="${MADARA_REQS}-java"
  PACKAGE_NAME="${PACKAGE_NAME}-java"
fi

if [ $DEPENDS_ROS -eq 1 ] ; then
  PACKAGE_NAME="${PACKAGE_NAME}-ros"
fi

DEPENDS="$DEPENDS, $MADARA_REQS (>= $MADARA_VERSION)"
DEPENDS="${DEPENDS} | ${MADARA_REQS}-python  (>= $MADARA_VERSION)"
DEPENDS="${DEPENDS} | ${MADARA_REQS}-ssl  (>= $MADARA_VERSION)"
DEPENDS="${DEPENDS} | ${MADARA_REQS}-python-ssl  (>= $MADARA_VERSION)"
DEPENDS="${DEPENDS} | ${MADARA_REQS}-zmq  (>= $MADARA_VERSION)"
DEPENDS="${DEPENDS} | ${MADARA_REQS}-python-zmq  (>= $MADARA_VERSION)"
DEPENDS="${DEPENDS} | ${MADARA_REQS}-ssl-zmq  (>= $MADARA_VERSION)"
DEPENDS="${DEPENDS} | ${MADARA_REQS}-python-ssl-zmq  (>= $MADARA_VERSION)"

PACKAGE_DIR="${PACKAGE_NAME}_${PACKAGE_DIR}"

# create a source tarball for ACE and GAMS source
echo "Creating ${PACKAGE_NAME}-$VERSION directory"
sudo rm -rf $PACKAGE_DIR
mkdir $PACKAGE_DIR
cd $PACKAGE_DIR

# we'll use the dpkg-deb script to read our debian directory, and
# dpkg-deb requires a DEBIAN folder and not the debian that dh does

mkdir DEBIAN

# create needed directories 
echo "Creating directories for source code and libraries"
mkdir -p $ROOT_DIR/include
mkdir -p $ROOT_DIR/lib
mkdir -p $ROOT_DIR/bin
mkdir -p $ROOT_DIR/doc/gams

# we want to add the new LD_LIBRARY_PATH
echo "Updating ld.so.conf.d"
mkdir -p etc/ld.so.conf.d
echo /$ROOT_DIR/lib >> etc/ld.so.conf.d/gams.conf

# copy the GAMS source includes into the appropriate directory
echo "Copying GAMS and EIGEN source trees for developers"
cp -r $GAMS_ROOT/src/gams $ROOT_DIR/include/
cp -r $EIGEN_ROOT/Eigen $ROOT_DIR/include/

# remove any shared objects, symbolic links and svn from the include directory
echo "Removing all symbolic links from source tree"
find $ROOT_DIR/include -type l -exec rm -f {} \;
echo "Removing all shared objects from source tree"
find $ROOT_DIR/include -name "*.so*" -type f -exec rm -f {} \;
echo "Removing all shared objects from source tree"
find $ROOT_DIR/include -name "*.o*" -type f -exec rm -f {} \;
echo "Removing all git directories from source tree"
find $ROOT_DIR/include -name .git -type d -exec rm -rf {} 2> /dev/null \;

# update changelog with the last 10 entries from the repository
echo "Saving last 10 git changelog entries into debian changelog"
git -C $GAMS_ROOT log --no-merges -10 > $ROOT_DIR/doc/gams/changelog

# merge GAMS copyright and license information
cat $GAMS_ROOT/LICENSE.txt >> $ROOT_DIR/doc/gams/copyright
echo "" >> $ROOT_DIR/doc/gams/copyright
cat $EIGEN_ROOT/COPYING.README >> $ROOT_DIR/doc/gams/copyright
dos2unix $ROOT_DIR/doc/gams/copyright

# we recently stopped using named sos, so we only need to copy over libGAMS
#cp $GAMS_ROOT/libGAMS.so.$LIB_VERSION $ROOT_DIR/lib
cp $GAMS_ROOT/lib/libGAMS.so $ROOT_DIR/lib

# 
if [ $INCLUDE_ALL_BIN -eq 1 ] ; then
  cp $GAMS_ROOT/bin/* $ROOT_DIR/bin/
elif [ $DEPENDS_ROS -eq 1 ] ; then
  cp $GAMS_ROOT/bin/ros2gams $ROOT_DIR/bin/
  cp $GAMS_ROOT/bin/gams_controller $ROOT_DIR/bin/
else
  cp $GAMS_ROOT/bin/gams_controller $ROOT_DIR/bin/
fi

OLD_DIR=$(pwd)

cd $ROOT_DIR/lib

#ln -s libGAMS.so.$LIB_VERSION libGAMS.so 

cd $OLD_DIR

sudo chown -R root.root $ROOT_DIR

# modify the control file to be specific to GAMS
echo "Package: ${PACKAGE_NAME}" >> DEBIAN/control
echo "Priority: extra" >> DEBIAN/control
echo "Section: libs" >> DEBIAN/control
echo "Architecture: $ARCHITECTURE" >> DEBIAN/control
echo "Maintainer: James Edmondson <jedmondson@gmail.com>" >> DEBIAN/control 
echo "Version: $VERSION" >> DEBIAN/control
echo "Depends: $DEPENDS" >> DEBIAN/control
#echo "Depends: debhelper (>= 8.0.0), libdl (>=2), librt (>=1), libstdc++ (>=6), libgcc (>=1), libpthread (>=0), libc (>=6), ld-linux (>=2)" >> DEBIAN/control
echo "Homepage: http:\/\/gams.ai" >> DEBIAN/control
echo "Description: Libraries for the GAMS middleware, version $VERSION" >> DEBIAN/control 

# create the debian package
cd ..
dpkg-deb -b $PACKAGE_DIR

exit 0
