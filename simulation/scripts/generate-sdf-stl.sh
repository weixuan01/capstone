#!/bin/bash

WALL_HEIGHT=$1    # not affected by resolution, typically 1m
IMAGE_RESOLUTION=$2    # resolution (meters per pixel) = real-world size / image pixel size
PNG_FILE=$3

SDF_STL_NAME="$(basename "${PNG_FILE%.*}")"

BASE_DIR=$(readlink -f "$(dirname "${BASH_SOURCE[0]}")") # get current dir
DESTINATION_PATH="$BASE_DIR/../crazyflie_mapping_demo/ros2_ws/install/ros_gz_crazyflie_gazebo/share/ros_gz_crazyflie_gazebo/worlds/" ## assumes crazyflie_mapping_demo and scripts folder are always in the same dir (simulation)

PNG23D_EXISTS=$(command -v png23d)
if [ "$PNG23D_EXISTS" = "" ]
then
	echo "png23d is not installed"
	exit 1
fi

IDENTIFY_EXISTS=$(command -v identify) # identify is a command within the imagemagick package used to get an image's dimensions in pixels
if [ "$IDENTIFY_EXISTS" = "" ]
then
	echo "imagemagick is not installed"
	exit 1
fi

cat << EOF > "$DESTINATION_PATH/$SDF_STL_NAME.sdf"
<?xml version="1.0" ?>

<sdf version="1.8">
  <world name="demo">
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
    <include>
      <uri>model://crazyflie</uri>
      <name>crazyflie</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <model name="$SDF_STL_NAME">
      <pose>-8 6 0 0 0 0</pose>	
      <static>true</static>
      <link name="map_link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>$SDF_STL_NAME.stl</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>$SDF_STL_NAME.stl</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
EOF

IMAGE_WIDTH=$(identify -format '%w' "$PNG_FILE")
IMAGE_HEIGHT=$(identify -format '%h' "$PNG_FILE")
REAL_WIDTH=$(awk "BEGIN {print $IMAGE_RESOLUTION * $IMAGE_WIDTH}")
REAL_HEIGHT=$(awk "BEGIN {print $IMAGE_RESOLUTION * $IMAGE_HEIGHT}")

png23d -o "stl" -w "$REAL_WIDTH" -h "$REAL_HEIGHT" -d "$WALL_HEIGHT" "$PNG_FILE" "$DESTINATION_PATH/$SDF_STL_NAME.stl"

echo "sdf and stl files generated successfully"
