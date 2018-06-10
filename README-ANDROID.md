While `base_build.sh` will handle most of the build process for android,
if given the `android` build flag, it does not install boost for Android
as required.

To install boost for android, checkout the Boost-for-Android repo:

`git clone git@github.com:moritz-wundke/Boost-for-Android.git`

Then, `cd` into the new directory, and run:

`./build-android.sh --boost=1.65.1 --arch=$ARCH $NDK_ROOT`

Where `NDK_ROOT` is the root path to the Android NDK (should be version r16b),
and `ARCH` is an architecture. You can see the available architectures with
`./build-android.sh --help`.

This build process will take some time. Once finished, you must add an
environment variable `BOOST_ANDROID_ROOT` to your `.bashrc`. Set it to
the directory `build/out/$ARCH_DIR/`, where `ARCH_DIR` varies based on
`ARCH`, but it should be obvious what it is. This directory should have
`include` and `lib` directories.

Once this is setup, you should be able to install and compile Madara and GAMS
for android by running:

`$GAMS_ROOT/scripts/linux/base_build.sh prereqs mpc madara gams android`
