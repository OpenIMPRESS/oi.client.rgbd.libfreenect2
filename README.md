# oi.client.rgbd.libfreenect2

Stream KinectV2 data using libfreenect2 through UDP accross the network.

# Installation

Overall steps:

 - Install [cmake](https://cmake.org/download/)
 - Install libjpeg-turbo ([windows binary here](https://sourceforge.net/projects/libjpeg-turbo/files/1.5.3/libjpeg-turbo-1.5.3-vc64.exe/download))
 - Clone [asio](https://github.com/chriskohlhoff/asio), the [libfreenect2](https://github.com/OpenKinect/libfreenect2) source and [this repository](https://github.com/OpenIMPRESS/oi.client.rgbd.libfreenect2). 
 - Use this folder structure:
```
  OpenIMPRESS/
    dependencies/
      asio/
      libfreenect2/
    clients/
      oi.client.rgbd.libfreenect2/
```


## Windows:
 - Go to the libfreenect2 source and follow the instructions to compile it on windows
 - make a folder "build" in oi.client.rgbd.libfreenect2
   - cd build
   - cmake -G "Visual Studio 14 2015 Win64" ..
   - (or your respective string for your version of visual studio)
 - Now you can open oi.client.rgbd.libfreenect2.sln with Visual Studio to compile your binaries

## macOS
 - Go to the libfreenect2 source and follow the instructions to compile it on macOS
 - Go to the oi.client.rgbd.libfreenect2 folder and make a "build" folder, then cmake...:
   - cd build
   - cmake -G Xcode ..
 - Now you have an XCode project which you can use to compile and run the binaries

## Linux (TODO)
 - Probably same as macOS
