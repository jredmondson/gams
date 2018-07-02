While `base_build.sh` will handle most of the build process for android,
if given the `android` build flag, it does not install boost for Android
as required.

Follow the steps to download and install Boost

* export ANDROID_ARCH environment variable to armeabi-v7a, x86, arm64, x86_64
* export `NDK_ROOT` pointing to your android-ndk root folder  (should be version r16b)
* `git clone git@github.com:amsurana/Boost-for-Android.git`
* `cd` into the new directory
* Run `./build-android.sh --boost=1.65.1 --arch=$ANDROID_ARCH $NDK_ROOT`
* This build process will take some time. Once finished, you must add an
environment variable `BOOST_ANDROID_ROOT` to your `.bashrc`. Set it to the root directory of boost.

Once this is setup, you should be able to install and compile Madara and GAMS for android by running:

* `$GAMS_ROOT/scripts/linux/base_build.sh prereqs madara gams android`
