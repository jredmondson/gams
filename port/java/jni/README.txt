To generate new JNI bindings, you should do the following:


LINUX:

cd $GAMS_ROOT/port/java/src
javah -classpath .:$MADARA_ROOT/lib/Madara.jar -d ../jni ai.gams.<whatever you're generating>



WINDOWS:

cd %GAMS_ROOT%/port/java/src
javah -classpath .;%MADARA_ROOT%\lib\Madara.jar -d ..\jni ai.gams.<whatever you're generating>
