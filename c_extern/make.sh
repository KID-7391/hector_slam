#!/bin/bash
g++ -shared -O2 map_update.cpp map_update.hpp -ldl -o map_update.so -fPIC -std=c++11
g++ -shared -O2 find_path.cpp find_path.hpp -ldl -o find_path.so -fPIC -std=c++11
