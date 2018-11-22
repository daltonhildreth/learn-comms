#!/usr/bin/python3

import shutil
import os
import subprocess
import sys


# TODO: replace this with 4 targets in CMake


def build_setting(no_comm, no_render):
    original = os.getcwd()

    # Create Build Folder
    build_folder = \
        "./build/" \
        + "_".join([
            ("no" if no_comm else "") + "comm",
            ("no" if no_render else "") + "render"
        ])
    try:
        os.makedirs(build_folder)
        os.chdir(build_folder)
    except OSError as e:
        print("Error:  %s - %s" % (e.filename, e.strerror))
        print("Failed to create %s build folder!" % build_folder)
        sys.exit()

    # Make build given settings
    if os.path.isdir("../../enginee"):
        args = ["cmake", "../../enginee"]
        if no_comm:
            args += ["-DNO_COMM:BOOL=ON"]
        if no_render:
            args += ["-DNO_RENDER:BOOL=ON"]
        subprocess.call(args)
        subprocess.call(["make"])
    else:
        print("Failed to find project to build!")
        sys.exit()

    # Undo side-effects
    try:
        os.chdir(original)
    except OSError as e:
        print("Error: %s - %s" % (e.filename, e.strerror))
        print("Failed to return to project directory!")
        sys.exit()


try:
    shutil.rmtree("./build/")
except OSError as e:
    print("Error: %s - %s." % (e.filename, e.strerror))
    print("Failed to delete build folder!")

for comm_setting in [False, True]:
    for render_setting in [False, True]:
        build_setting(comm_setting, render_setting)
