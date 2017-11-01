# oi.client.rgbd.libfreenect2

Stream KinectV2 data using libfreenect2 through UDP accross the network.

# Installation

## Windows:
 - Download and compile libfreenect2 first:
   - git clone https://github.com/hmi-utwente/oi.client.rgbd.libfreenect2.git
   - Follow the instructions to compile it on your platform
 - Get a copy of asio:
   - git clone https://github.com/chriskohlhoff/asio.git
 - Make a folder oi.client.rgbd.libfreenect2/depends and clone asio into it:
   - cd oi.client.rgbd.libfreenect2 && mkdir depends && cd depends
   - git clone https://github.com/chriskohlhoff/asio.git
 - make a folder "build" in oi.client.rgbd.libfreenect2
   - cd build
   - cmake -G "Visual Studio 14 2015 Win64" -D asio\_ROOT\_DIR="path/to/asio" -D freenect2\_ROOT\_DIR="path/to/libfreenect2" ..
   - (or your respective string for your version of visual studio)
   - (default locations for search for asio and libfreenect2 folders is next to this folder)
 - Now you can open oi.client.rgbd.libfreenect2.sln with Visual Studio

## macOS
 - Download and compile libfreenect2 first:
   - git clone https://github.com/hmi-utwente/oi.client.rgbd.libfreenect2.git
   - Follow the instructions to compile it on your platform
 - Get a copy of asio:
   - git clone https://github.com/chriskohlhoff/asio.git
 - Make a folder oi.client.rgbd.libfreenect2/depends and clone asio into it:
   - cd oi.client.rgbd.libfreenect2 && mkdir depends && cd depends
 - Go back to oi.client.rgbd.libfreenect2 folder and make a "build" folder, then cmake...:
   - cd build
   - cmake -G Xcode -D asio\_ROOT\_DIR="path/to/asio" -D freenect2\_ROOT\_DIR="path/to/libfreenect2" ..
   - (default locations for search for asio and libfreenect2 folders is next to this folder)
 - Now you have a solution for your IDE with which you can compile and run oi.client.rgbd.libfreenect2

## Linux (TODO)
 - Probably same as macOS, but also stay on the master branch of asio.
