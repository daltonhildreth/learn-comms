# Author
Dalton Hildreth

# Plan
At some point there will be the following components, perhaps rearranged when
I read more about Game Engine Architecture, or as I decide something is better:
* Manager/interface module 
    * timer
    * Memory Unit
        * pool
* Render/lighting module
    * scene\_graph
    * light
    * mesh
    * texture
    * shader
    * model
    * primitive mesh
        * line/cylinder
        * cubic/quad
        * ellipsoid/ellipse
        * triangle/prism
        * k-DOP
* Physics/simulation module
    * BVH
    * collider
    * particle
    * integrator
* AI/planning module
    * Agent
    * PRM
    * Graph
    * Global (may not actually be a module)
        * A\*
        * RRT\*
    * Local (may not actually be a module)
        * Boids
        * TTC
        * VBO
        * ORCA
* Math module 
    * glm
* OS I/O module 
    * parser for any internal format (levels/scenes/etc.)
    * image loader -- stb\_image
    * mesh loader -- assimp
    * writer (for any internal format/log)
* UI module 
  * Terminal/CLI input
  * Keymapper
  * GUI
* Test module (testing / debug-vis / debug-only-code / etc.)
    * unit test suite
    * visual debug overlay
    * debug only
    * benchmarking / logger

# Resources:
I used http://learnopengl.com to, well, learn OpenGL, so some of my code is
based on his to do rendering.

# Build
gg is written to be cross-platform, however, at the moment the build system's
first step (fetching) is not windows compatible. I am working to fix this.

## Linux/OS X
In the terminal perform the following to go from nothing to a running build:
```
$ git clone https://github.com/fiblit/gg-engine.git
$ cd gg-engine
$ . lib/init.sh
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./bin/gg-engine
```
If you run into issues, it may be because you don't have bash or CMake. On
Linux, do this to get them:
```
$ sudo apt-get install cmake
$ sudo apt-get install bash
```
