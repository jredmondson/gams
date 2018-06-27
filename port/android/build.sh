#!/bin/bash

unzip_strip() (
  local zip=$1
  local dest=${2:-.}
  local temp=$(mktemp -d) && unzip -d "$temp" "$zip" && mkdir -p "$dest" &&
  shopt -s dotglob && local f=("$temp"/*) &&
  if (( ${#f[@]} == 1 )) && [[ -d "${f[0]}" ]] ; then
    mv "$temp"/*/* "$dest"
  else
    mv "$temp"/* "$dest"
  fi && rmdir "$temp"/* "$temp"
)


#Beginning of script
#Process:
#Download SDK
#Download Build Tools
#Download Platform Tools
#Download Platform
#Change SDK & NDK dir in demo app
#Run Gradle

START_DIR=`pwd`
ANDROID_SDK_DIR=`pwd`"/sdk"
PLATFORM_VERSION="android-27"
BUILD_TOOLS_VERSION="27.0.3"

ANDROID_TOOLS_ZIP="sdk-tools-linux-3859397.zip"
ANDROID_TOOLS_DIR="$ANDROID_SDK_DIR/tools"
DEMO_PRJ_DIR=`pwd`"/demo-prj"

JAR_LIBS_DIR="$DEMO_PRJ_DIR/app/libs"
JNI_LIBS_DIR_ARCH="$DEMO_PRJ_DIR/app/src/main/jniLibs/$ANDROID_ARCH"


#Download android sdk tools

if [ ! -z "$ANDROID_HOME" ]; then
   ANDROID_SDK_DIR=$ANDROID_HOME
   ANDROID_TOOLS_DIR="$ANDROID_SDK_DIR/tools"
  echo "Android SDK Home is set to $ANDROID_SDK_DIR"
else 
   echo "Android SDK Home not set, SDK dir is set to $ANDROID_SDK_DIR"
fi 


if [ ! -d "$ANDROID_TOOLS_DIR" ]; then
       #Download tools
        if [ ! -f "$ANDROID_TOOLS_ZIP" ]; then
          wget "https://dl.google.com/android/repository/$ANDROID_TOOLS_ZIP" || exit $?
	fi
	unzip_strip $ANDROID_TOOLS_ZIP $ANDROID_TOOLS_DIR || exit $?
fi


#Workaround. There are times android repositories config file fail to download. Weird, but true.
mkdir -p ~/.android
touch ~/.android/repositories.cfg


#Check for Platform or download if unavailable
if [ ! -d "$ANDROID_SDK_DIR/platforms/$PLATFORM_VERSION" ]; then
    echo "Installing android plaform $PLATFORM_VERSION" 
    "$ANDROID_TOOLS_DIR/bin/sdkmanager" --sdk_root="$ANDROID_SDK_DIR" "platform-tools" "platforms;$PLATFORM_VERSION"
fi

#Check build-tools or download if unavailable
if [ ! -d "$ANDROID_SDK_DIR/build-tools/$BUILD_TOOLS_VERSION" ]; then
    echo "Installing build tools $BUILD_TOOLS_VERSION"
    "$ANDROID_TOOLS_DIR/bin/sdkmanager" --sdk_root="$ANDROID_SDK_DIR" "build-tools;27.0.3"
fi


#Copy all required JNI files into respective project directory.

MADARA_LIB=$MADARA_ROOT/lib/libMADARA.so
MADARA_JAR_LIB=$MADARA_ROOT/libMadara_Jar.so
MADARA_JAR=$MADARA_ROOT/lib/madara.jar
GAMS_LIB=$GAMS_ROOT/lib/libGAMS.so
GAMS_JAR_LIB=$GAMS_ROOT/libgams_jar.so
GAMS_JAR=$GAMS_ROOT/lib/gams.jar

if [ ! -f $MADARA_LIB ]  || [ ! -f $MADARA_JAR_LIB ] || [ ! -f $GAMS_LIB ]  || [ ! -f $GAMS_JAR_LIB ] || [ ! -f $MADARA_JAR ] || [ ! -f $GAMS_JAR ]; then 
   echo "Looks like not all required libraries are available. Please ensure BOOST, MADARA, GAMS libraries are available";
   exit;
fi

#Remove old ones
rm -r $JNI_LIBS_DIR_ARCH

#Copy JNI files
mkdir -p $JNI_LIBS_DIR_ARCH

cp $MADARA_LIB $JNI_LIBS_DIR_ARCH
cp $GAMS_LIB $JNI_LIBS_DIR_ARCH
cp $MADARA_JAR_LIB $JNI_LIBS_DIR_ARCH
cp $GAMS_JAR_LIB $JNI_LIBS_DIR_ARCH
cp $BOOST_ANDROID_ROOT/lib/*.so $JNI_LIBS_DIR_ARCH

#Copy Java files
mkdir -p $JAR_LIBS_DIR
cp $MADARA_JAR $JAR_LIBS_DIR
cp $GAMS_JAR $JAR_LIBS_DIR


#Build and compile the APK
cd $DEMO_PRJ_DIR
echo "sdk.dir=$ANDROID_SDK_DIR" > "$DEMO_PRJ_DIR/local.properties"
echo "ndk.dir=$NDK_ROOT" >> "$DEMO_PRJ_DIR/local.properties"
./gradlew clean build

if [ ! -f "$DEMO_PRJ_DIR/app/build/outputs/apk/debug/app-debug.apk" ]; then
 	echo -e "\e[91mAPK File not created\e[39m"
  else
    	echo -e "APK File =\e[92m $DEMO_PRJ_DIR/app/build/outputs/apk/debug/app-debug.apk \e[39m"
  fi

cd $START_DIR

