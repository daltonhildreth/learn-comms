#!/bin/bash
# Copyright (c) 2016-2018 Dalton Hildreth
# This file is under the MIT license. See the LICENSE file for details.
set -exuo

printf "%-45s" "Removing old 3rd-party libs..."
{
    if [pwd == .*"gg-engine/lib/"]; then
        cd ..
    fi
    if [pwd != .*"gg-engine"]; then
        printf "ERROR: wrong dir\ngoto gg-engine/lib or gg-engine"
        exit
    fi
    rm -rf lib/*/ lib/catch.hpp lib/stb_image.h
    cd lib
} &> /dev/null
printf "%15s\n" "Done."
printf "%-45s\n" "Downloading latest 3rd-party libs..."

# Refresh Catch
printf "%-45s" "Downloading latest catch.hpp..."
curl -s https://api.github.com/repositories/1062572/releases/latest \
    | jq --raw-output '.assets[0] | .browser_download_url' \
    | xargs wget \
    &> /dev/null
printf "%15s\n" "Done."

# Refresh glad
printf "%-45s" "Install glad and generate latest..."
{
    mkdir glad
    cd glad
    pip3 install --user glad
    python3 -m glad --out-path=. --generator=c --local-files --spec=gl --no-loader
    cd ..
} &> /dev/null
printf "%15s\n" "Done."

#Refresh glfw/glm/assimp
printf "%-45s" "Downloading latest version of glfw..."
git clone --depth=1 https://github.com/glfw/glfw.git &> /dev/null
printf "%15s\n" "Done."
printf "%-45s" "Downloading latest version of glm..."
git clone https://github.com/g-truc/glm.git &> /dev/null
cd glm
git reset --hard d214fbaaf1bebee952f9949ab9c649d02ca00552 &> /dev/null
cd ..
printf "%15s\n" "Done."

#Refresh stb_image*
printf "%-45s" "Downloading latest version of stb_image.h..."
{
    git clone --depth=1 https://github.com/nothings/stb.git
    # find . -not -name 'stb_image.h' | xargs rm -rf
    mv stb/stb_image.h .
    # mv stb/stb_image_write.h .
    rm -rf stb
} &> /dev/null
printf "%15s\n" "Done."

# Refresh Eigen3
printf "%-45s" "Downloading latest version of Eigen v3..."
git clone --depth=1 --single-branch -b branches/3.0 https://github.com/eigenteam/eigen-git-mirror.git eigen &> /dev/null
printf "%15s\n" "Done."

#Remove the git histories. (unneeded)
printf "%-45s" "Cleaning libs..."
{
    cd ..
    find lib -name '.git' | xargs rm -rf
} &> /dev/null
printf "%15s\n" "Done."
