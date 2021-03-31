cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr)
project(lexxpluss_apps)

FILE(GLOB app_sources src/*.cpp)
target_sources(app PRIVATE ${app_sources})
