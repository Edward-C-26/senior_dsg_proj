# BMS Test Platform

Pulls in BMS primary code, simulates the world around it to allow us to test the response in a controlled environment.
This runs on your local device, no BMS, no accumulator, no STMCubeIDE needed.

To run tests, you must build the test first. Make sure you have `make` and `gcc` installed.

'''
cd BMS_2024/Test/build
make # builds the test
./test # run the test
'''

Currently TestAlgorithims.c is only testing balancing.
You can edit various `#define` parameters to change the way the test runs.


