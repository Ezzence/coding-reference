
**Dependencies**

Open3D, a modern 3D pointcloud processing library with integrated machine learning capabilities:

http://www.open3d.org/docs/release/compilation.html

The only additional steps necessary are to set the following in the Open3D cmake before compiling:
 - GLIBCXX_USE_CXX11_ABI ON
 - BUILD_SHARED_LIBS ON
 - BUILD_CUDA_MODULE ON in case the device is CUDA compatible like the TX2

This additional package for converting data between ROS and Open3D:
https://github.com/ros-perception/perception_open3d

**Open3D Fence Scanning Flow**
- Pointcloud data (PCD) is taken from a Lidar and passed onto this pointcloud_processing node through ROS
- The pointcloud is converted to Open3D and cropped into the 3D space where the fence is expected to be at all times (a bounding box to the right of the robot)
- PCD is rotated to counteract the rotation in the placement of the physical Lidar sensor
- The PCD is segmented in order to extract the biggest plane available, if this is not a plane that approximately aligns with where the fence is expected to be, the data is discarded. The data could also be further segmented until a suitable plane is found, but this has not been necessary so far
- The segmented plane is further analysed for expected distances relative to the robot, and if it is found to be too far or too close, it is discarded
- The segmented plane is translated and rotated to be centered in the middle of the reference coordinate system, this is done in order to align it with the hit grid, and it is node in a way where bottom of the plane is always forced onto the bottom of the hit grid
- The segmented and now centered plane is run through the hit grid algorithm point-by-point. The hit grid is a planar collection of voxels/cells, and if a cell is found to contain no points inside, the cell is considered breached
- Finally, the algorithm reports wheter a breach is considered detected or not. A breached hit grid is only considered a breached fence, if the majority of the hit grid is not breached, but a smaller part is (to avoid issues where the fence is too misaligned with the sensor), and only if two consecutive hit grids have been found to be breached, on two consecutive Lidar iterations

**Performance**

The algorithm itself is fairly robost, and live autonomous testing at Sivlandvænget has shown 5 out of 5 missions to report correctly on the 3 breaches at the fence. More thorough testing would be required to establish accurate statistics on the performance.

The speed of the algorithm significantly outperforms the speed of of the data that a VLP16 sensor can provide. One full iteration of the algorithm takes between 30-50ms, meaning that it could support a Lidar sensor with higher resolution.

The performance is currently more limited by the movement characteristics of the Capra P3 robot, and the lack of a wide FOV camera, than by the fence scanning algorithm.

Current limitations:
- The algorithm's ability to detect smaller holes in fences is proportial to the resolution of the Lidar, with the VLP16 sensor, holes below 20 cm radius cannot be detected
- The algorithm is currently robost enough against the uneven ground found next to the fence at Sivlandvænget. However, it can be susceptible to false-positive detections in case of steeper inclines, where the robot has to pitch up or down much. A quick solution for this is given in the comments inside the code, but is not yet implemented.

**Debugging**

The solution has been developed with visual debugging of the algorithm in mind. Launch rviz after bagging a run of the fence scanning and see how the algorithm performed in detail.
/open3d/grid shows the hit grid, white voxels for healthy sections, black voxels for breaches
/open3d/debug shows the PCD after it has been setup and cropped, but before segmentation
/open3d/aligned shows the PCD after it has been also segmented and aligned with the hit grid


**Behaviour logic changes**
A fence scanning behaviour has been added to behaviour logic based on the move_to_waypoint behaviour. A non-robot-type specific fence scanning class has also been added, but the behaviour is only configured for driving robots. To enable this behaviour and replace the normal waypoint behaviour with it, use the
- FT_FENCE_SCAN

feature toggling environment variable. The behaviour is similar to waypoint behaviour, except that it tries to perfrom fence scanning when a LOI is given during a waypoint task. Align the LOI with the fence when giving the mission, and the robot will try to detect breaches in the fence, save photos of the breached areas of the fence, and then upload these photos with their gps coordinates to a collection in the hive.

Additionally, a robot active behaviour has been defined for driving robots, that sets the robot as active while a mission is active. The reason behind this change is that the robot did not automatically upload data without it, and tying the logic of the robot being active to drone landing and takeoff is not compatible with driving robots.
Some other minor changes have also been added, such as an increased camera timeout for driving robots, due to the Axis camera.

--------------------------------------------------

**Capra movement changes**
The sensor movement class associated with the capra has been revised to fit better with the Axis camera, and its customizability has been extended. Some smaller changes to the movement characteristics of the capra robot have also been added, mostly in the form of parameter tuning. 

**DEPRECATED PCL Hit Grid algorithm**
This first version of a hit grid algorithm has been added as a PCL-based python node. Pointcloud data from the LiDar sensor are compared against a grid of cuboids, that are statically arranged in the expected configuration of how the fence will appear to the LiDar sensor. These cuboids are referred to as cells, and if a point from the pointcloud falls inside the cell, the hit counter of the cell is incremented. Currently, if a cell receives no hits, it is considered a breach, and forwarded in a message to behaviour logic, and as a visualization message that can be displayed in rviz. The algorithm does not stitch together a larger pointcloud from multiple sensor messages, but rather processes each pointcloud message individually, and uses voxel grid filtering on them to boost performance.

Currently one the biggest drawbacks of this algorithm are that if the fence during fence scanning does not accurately line up with the static setup of the hit grid, false positive detections can easily occur. This could be solved by estimating the plane of the fence during runtime, and adjusting the shape of the hit grid dynamically.

**Axis PTZ camera**
This PR includes support for the Axis camera, in the form of a placeholder cpp interface for the camera part, and a python class for the PTZ part. They are both designed for passing through communication to an existing open-source ROS driver for the Axis camera, a fork of which is also added with this PR.

**OS configurations, libraries, etc.**
There are currently severe issues and incompatibility between the latest docker version used by the Balena OS, and ROS nodelets. This issue has been mitigated (though not resolved), by setting the docker container to host networking mode. Additionally, it was necessary to add some new libraries for enabling PCL, that had to be compiled manually due to our devices running noetic on ubuntu 18.