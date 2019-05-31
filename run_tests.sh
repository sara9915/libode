#!/bin/bash

: '
This short bash/shell script will compile and run all the tests for the C++
integrators, plotting the output along the way by calling the appropriate
Python scripts.
'

function test {
    echo "---" ${1} "---"
    ./bin/ode_test_${1}.exe
    cd scripts
    python plot_${1}.py
    cd ..
    python clean.py out
}

python clean.py

make tests

for T in adapt conv fixed snaps work; do
    test $T
done
