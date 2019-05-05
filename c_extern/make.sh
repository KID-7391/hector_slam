#!/bin/bash
gcc -shared -O2 map_update.c map_update.h -ldl -o map_update.so -fPIC