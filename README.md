# oi.client.rgbd.libfreenect2

Stream KinectV2 data using libfreenect2 through UDP accross the network.

# Installation

## Windows (TODO!):
 - Download and compile libfreenect2 firstake sure you can compile libfreenect first:
   - git clone https://github.com/hmi-utwente/oi.client.rgbd.libfreenect2.git
 - Make a folder oi.client.rgbd.libfreenect2/depends and clone asio into it(stay on the master branch for Windows!):
   - cd oi.client.rgbd.libfreenect2 && mkdir depends && cd depends
   - git clone https://github.com/chriskohlhoff/asio.git
 - make a folder "build" in oi.client.rgbd.libfreenect2
   - cd build
   - cmake -G "Visual Studio 14 2015 Win64" -D freenect2\_ROOT\_DIR="</path/to/libfreenect2>" ..
   - (or your respective string for your version of visual studio)
 - Now you can open oi.client.rgbd.libfreenect2.sln with Visual Studio

## macOS
 - Place this in a folder next to the libfreenect2 folder, and make sure you can compile libfreenect first:
   - git clone https://github.com/hmi-utwente/oi.client.rgbd.libfreenect2.git
 - Make a folder oi.client.rgbd.libfreenect2/depends and clone asio into it, checkout the asio-1-10-branch:
   - cd oi.client.rgbd.libfreenect2 && mkdir depends && cd depends
   - git clone https://github.com/chriskohlhoff/asio.git
   - cd asio
   - git checkout asio-1-10-branch
 - Go back to oi.client.rgbd.libfreenect2 folder and make a "build" folder, then cmake...:
   - cd build
   - cmake -G Xcode -D freenect2\_ROOT\_DIR="</path/to/libfreenect2>" ..
 - Now you have a solution for your IDE with which you can compile and run oi.client.rgbd.libfreenect2

## Linux (TODO)
 - Probably same as macOS, but also stay on the master branch of asio.
