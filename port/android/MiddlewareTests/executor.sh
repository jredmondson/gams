if ! adb get-state 1>/dev/null 2>&1
  then
    echo "Android device not connected."
    exit 1;
fi
./gradlew clean assemble assembleAndroidTest installDebug installDebugAndroidTest

echo "Running basic tests"

basicTests=($(jar tf app/libs/madara-tests.jar | grep "ai/madara/tests/basic/.*class$"))
for testClass in "${basicTests[@]}"
do

testClass="${testClass/.class/}"
testClass="${testClass//\//.}"

echo "Executing: $testClass"

adb shell am instrument -w -r  -e debug false -e class "${testClass}" ai.madara.testcoverage.test/android.support.test.runner.AndroidJUnitRunner

done

adb uninstall ai.madara.testcoverage.test
adb uninstall ai.madara.testcoverage
