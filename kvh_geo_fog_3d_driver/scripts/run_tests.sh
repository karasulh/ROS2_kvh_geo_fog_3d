#! /bin/bash

# Example usage: ./scripts/run_tests.sh [-v] [-b]

# Output:
# ./scripts/run_tests.sh Uses grep to show only results from the tests
# ./scripts/run_tests.sh -v Shows all output, it is a lot so good luck
# ./scripts/run_tests.sh -b Builds with the tests, good for debugging, but doesn't show test results


if [[ $1 == '-v' ]]; then
{
    colcon test --packages-select kvh_geo_fog_3d_driver
    colcon test-result --all --verbose
}
elif [[ $1 == '-b' ]]; then
    colcon test --packages-select kvh_geo_fog_3d_driver
else
{
    colcon test --packages-select kvh_geo_fog_3d_driver
    colcon test-result --all
}  
fi