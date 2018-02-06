# marker_publisher
ros-package for [aruco304][1] released recently, by the end of 2017. 

It detects all the markers that belong to a particular dictionary that has been specified in *dict_type* with their sizes and publish them in MarkerArray message and as tranformations [tf][2]

### size configuration
I suppose that all the markers have the same size which you can modify in the launch file in *markerSizeMeters* param. Now, if you have another marker(s) with different size then you have to specify that separately like so

    <param name="marker_i" value="m_i" /> <!-- Marker_id=i Size meter-->

Where i with the *id* of the marker and *m_i* its size.

### Prerequisites
* Calibration:

You need to calibrate the camera and copy the generated file in */config* folder in *marker_publisher*

* Install Aruco 3:

[ArUco: a minimal library for Augmented Reality applications based on OpenCV](http://www.uco.es/investiga/grupos/ava/node/26)


### ROS API

#### Messages

 * marker_publisher/Marker.msg

        uint32 idx
        geometry_msgs/PoseWithCovariance pose

 * marker_publisher/MarkerArray.msg

        Header header
        marker_publisher/Marker[] markers

### TODO
This is a first version

Much things to add (publish only if there's subscriber..etc)
        
[1]: https://sourceforge.net/projects/aruco/files/3.0.0/
[2]: http://wiki.ros.org/tf
