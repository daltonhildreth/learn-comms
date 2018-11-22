# Copyright (c) 2016-2018 Dalton Hildreth
# This file is under the MIT license. See the LICENSE file for details.
#to be copied into glad/
cmake_minimum_required(VERSION 3.0)
project(glad)

add_library(${PROJECT_NAME} glad.c)
