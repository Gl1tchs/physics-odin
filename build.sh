#!/bin/bash

if [ "$1" == "--run" ]; then
    echo "Running the program..."
    odin run src -out=build/physics-odin.exe -debug
    exit 0
else
    echo "Building the program..."
    odin build src -out=build/physics-odin.exe -debug
    exit 0
fi
