## Required Environment
- Linux
- ROS2
- cmake 3.22.1 or higher
- Qt5
- qtwebengine5-deb
- @googlemaps/markerclusterer
- can_msgs

## Install Guid
### install qtwebengine5
~~~
sudo apt-get install qtwebengine5-dev
~~~

### Install & Build route-editor 
#### Install & Build
~~~
git clone  -b KEC/Linux/Develop https://github.com/WAVEM-Inc/visualization.git --recurse-submodules
cd visualization
mkdir build
cd build
cmake ..
make
~~~

#### Set Google Map API Key
From /resources/map/map.html file, you should change dummy google map api key to YOUR_API_KEY.
Find this cord and insert your google map api key.
~~~
<script async defer src="https://maps.googleapis.com/maps/api/js?key=YOUR_GOOGLE_MAP_API_KEY&libraries=places&callback=initMap"></script>
~~~


#### Run route_editor
~~~
./YOUR_DIRECTORY/visualization/build/route_editor
~~~
