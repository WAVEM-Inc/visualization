Required Environment

- Tested on Ubuntu 22.04
- CMake 3.22.1
- Qt5
- qtwebengine5-dev
- @googlemaps/markerclusterer

## Required Environment
- Ubuntu 22.04
- ROS2
- cmake 3.22.1 or higher
- Qt5
- qtwebengine5-deb
- @googlemaps/markerclusterer

## Install Guid
### install qtwebengine5
~~~
sudo apt-get install qtwebengine5-dev
~~~

### Install & Build route-editor 
#### Install & Build
~~~
git clone  -b KEC/Linux/JaeHyeok/Feature/route_editor/NodeEditor https://github.com/WAVEM-Inc/visualization.git --recurse-submodules
cd visualization
mkdir build
cd build
cmake ..
make
~~~

Run program file will be locate in 'YOUR_DIRECTORY/visualization/build/route_editor'

#### Run route_editor
~~~
./YOUR_DIRECTORY/visualization/build/route_editor
~~~
