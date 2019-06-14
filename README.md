# Robotiq modular gripper

 - [Install](#install)
 - [Launch](#launch)

## Industrial and modular end-of-arm tooling

<a href="http://www.acutronicrobotics.com"><img src="https://acutronicrobotics.com/products/modular-grippers/images-v2/v2_Robotiq_gripper_2-08.jpg.pagespeed.ce.2tQN4GJ5PS.jpg" align="right" hspace="8" vspace="2" width="200"></a>

As they are key to highly flexible automation tasks, we have enhanced three different grippers from Robotiq through the H-ROS SoM. Ranging from 50mm to 140mm stroke.

 - **ROS 2.0 native**: Ready to be steered with MARA, the first truly modular collaborative robot.
 - **Independent ROS 2.0 adapter, plug & play**:  Easy to install manually and seamlessly working with other modular robot components.
 - **Real-time, time-sensitive communications**: Qualified -through the H-ROS SoM- to achieve time synchronization and deterministic communications enabled with TSN standards.
 - [**Ideal for MARA**](https://acutronicrobotics.com/products/mara/): Three highly adaptable grippers to be used in combination with MARA or other ROS 2.0 enabled robots. Now you can operate them from ROS 2.0 directly, by using a simple Ethernet connection.



<table id="specs-table" style="width:100%">
   <tr id="table-header">
       <th style="width: 16.66%"></th>
       <td style="width: 16.66%"><img style="width: 75%; padding-bottom: 1rem;" src="https://acutronicrobotics.com/products/modular-grippers/images-v2/xModular_Gripers-09-1.png.pagespeed.ic.a_R_yG8mwh.webp" data-pagespeed-url-hash="3896914255" onload="pagespeed.CriticalImages.checkImageForCriticality(this);"></td>
       <th style="width: 16.66%"><img style="width: 75%; padding-bottom: 1rem;" src="https://acutronicrobotics.com/products/modular-grippers/images-v2/xModular_Gripers-09-2.png.pagespeed.ic.ajuwMJNajn.webp" data-pagespeed-url-hash="4191414176" onload="pagespeed.CriticalImages.checkImageForCriticality(this);"></th>
       <td style="width: 16.66%"><img style="width: 75%; padding-bottom: 1rem;" src="https://acutronicrobotics.com/products/modular-grippers/images-v2/xModular_Gripers-09-3.png.pagespeed.ic.059x3Em1eg.webp" data-pagespeed-url-hash="190946801" onload="pagespeed.CriticalImages.checkImageForCriticality(this);"></td>
   </tr>
   <tr>
       <td class="row-title"></td>
       <td><center><b>S50</b></center></td>
       <td><center><b>S85</b></center></td>
       <td><center><b>S140</b></center></td>
   </tr>
   <tr>
       <td class="row-title">Original gripper</td>
       <td>Hand-E</td>
       <td>2F-85</td>
       <td>2F-140</td>
   </tr>
   <tr>
       <td class="row-title">Stroke (adjustable)</td>
       <td>50 mm</td>
       <td>85 mm</td>
       <td>140 mm</td>
   </tr>
   <tr>
       <td class="row-title">Grip force (adjustable)</td>
       <td>60 to 130 N</td>
       <td>20 to 235 N </td>
       <td>10 to 125 N</td>
   </tr>
   <tr>
       <td class="row-title">Form-fit grip payload</td>
       <td>5 kg</td>
       <td>5 kg</td>
       <td>2.5 kg</td>
   </tr>
   <tr>
       <td class="row-title">Friction grip payload</td>
       <td>3 kg</td>
       <td>5 kg</td>
       <td>2.5 kg</td>
   </tr>
   <tr>
       <td class="row-title">Gripper mass</td>
       <td>1 kg</td>
       <td>0.9 kg</td>
       <td>1 kg</td>
   </tr>
   <tr>
       <td class="row-title">Position resolution (fingertip)</td>
       <td>0.2 mm</td>
       <td>0.4 mm</td>
       <td>0.6 mm</td>
   </tr>
   <tr>
       <td class="row-title">Closing speed (adjustable)</td>
       <td>20 to 150 mm/s</td>
       <td>20 to 150 mm/s</td>
       <td>30 to 250 mm/s</td>
   </tr>
</table>

# Install

## Install ROS 2.0

Install ROS 2.0 following the official instructions: [source](https://index.ros.org/doc/ros2/Linux-Development-Setup/) [debian packages](https://index.ros.org/doc/ros2/Linux-Install-Debians/).

## Create mara ROS 2.0 workspace
Create a ROS workspace, for example:

```sh
mkdir -p ~/ros2_mara_ws/src
cd ~/ros2_mara_ws
sudo apt install -y python3-vcstool python3-numpy
wget https://raw.githubusercontent.com/acutronicrobotics/MARA/dashing/mara-ros2.repos
vcs import src < mara-ros2.repos
```

Generate HRIM dependencies:

```sh
cd ~/ros2_mara_ws/src/HRIM
sudo pip3 install hrim
python3 hrim.py generate models/actuator/servo/servo.xml
python3 hrim.py generate models/actuator/gripper/gripper.xml
```

## Compile
Right now you can compile the code:

```sh
source /opt/ros/dashing/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install --packages-skip individual_trajectories_bridge
```

# Launch

### S85 ( Robotiq 2f-85 )

```
ros2 launch robotiq_gazebo robotiq_85.launch.py
```

### S140 ( Robotiq 2f-140 )

```
ros2 launch robotiq_gazebo robotiq_140.launch.py
```

### S50 ( Robotiq Hand-E )

```
ros2 launch robotiq_gazebo robotiq_hande.launch.py
```
