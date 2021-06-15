# RMV2D
An improved ROS map web tool based on ROS2D and NAV2D.

## Dependencies
RMV2D depends on:

[EventEmitter2](https://github.com/hij1nx/EventEmitter2). The current supported version is 0.4.14. The current supported version can be found on the Robot Web Tools CDN: ([full](https://static.robotwebtools.org/EventEmitter2/0.4.14/eventemitter2.js)) | ([min](https://static.robotwebtools.org/EventEmitter2/0.4.14/eventemitter2.min.js))

[EaselJS](https://github.com/CreateJS/EaselJS/). The current supported version is 0.7.1. The current supported version can be found on the Robot Web Tools CDN: ([full](https://static.robotwebtools.org/EaselJS/0.7.1/easeljs.js)) | ([min](https://static.robotwebtools.org/EaselJS/0.7.1/easeljs.min.js))

[roslibjs](https://github.com/RobotWebTools/roslibjs). The current supported version is 0.14.0. The current supported version can be found on the Robot Web Tools CDN: ([full](https://static.robotwebtools.org/roslibjs/0.14.0/roslib.js)) | ([min](https://static.robotwebtools.org/roslibjs/0.14.0/roslib.min.js))

## Documentation

### RMV2D.Viewer
The viewer is the viewport for the map. 

#### Options
* **divID (required)**: The ID of the div that the canvas is going to be generated in
* **width (required)**: The width of the canvas in pixels. This must be set on generation, and cannot be changed
* **height (required)**: The height of the canvas in pixels. This must be set on generation, and cannot be changed
* **background (optional)**: The background color of the canvas. Defaults to *'#111111'*
* **enableZoom (optional)**: Boolean value that enables zooming on the viewport with a mouse. Defaults to *false*
* **enablePan (optional)**: Boolean value that enables panning on the viewport with a mouse. Defaults to *false*

#### Example
```javascript
var mapView = new RMV2d.Viewer({
  divID : "mapView",
  width: 1000,
  height: 800,
  background: "#a1a1a1",
  enableZoom: true,
  enablePan: true
});
```


### RMV2D.OccupancyGridClient 
The OccupancyGridClient takes the ROS *nav_msgs/OccupancyGrid* messages and displays them on the viewport. 

#### Options
* **ros (required)**: The *ROSLIB.Ros* object
* **rootObject (required)**: The *scene* object of the *ROS2D.Viewer* object to display the map in
* **mapTopic (optional)**: The topic that the OccupancyGrid is published on. Defaults to *'/map'*

#### Example
```javascript
var occupancyGridClient = new RMV2D.OccupancyGridClient({
  ros: ros,
  rootObject: mapView.scene
  mapTopic: "/cartographer_map"
});
```


### RMV2D.PoseClient
The PoseClient listens to a *geometry_msg/Pose* topic and displays the pose on the viewport with an image.

#### Options
* **ros (required)**: The *ROSLIB.Ros* object
* **rootObject (required)**: The *scene* object of the *ROS2D.Viewer* object to display the pose in
* **topic (optional)**: The topic that the Pose is published on. Defaults to *'/robot_pose'*
* **imageSrc (optional)**: The image that is used to display the pose. Defaults to *'robotPose.png*
* **trackPose (optional**: Boolean value that enables tracking of the robot pose. Enabling this will keep the robot centered in the viewport
* **trackOrientation (optional)**: Boolean value that enables tracking of the robot orientation. Enabling this will keep the robot orientated north in the viewport.

#### Example
```javascript
var poseClient = new RMV2D.PoseClient({
  ros: ros,
  rootObject: mapView.scene,
  topic: "/second_robot_pose",
  imageSrc: "robotDog.png",
  trackPose: true,
  trackOrientation: false
});
```


### RMV2D.NavigationClient
The NavigationClient enables sending a marker to the robot to navigate to. 

#### Options
* **ros (required)**: The *ROSLIB.Ros* object
* **rootObject (required)**: The *scene* object of the *ROS2D.Viewer* object to use for the NavigationClient
* **occupancyGridClient (required)**: The RMV2D.OccupancyGridClient object
* **imageSrc (optional)**: The image that is put on the map whenever a goal is created. Defaults to *'goalMarker.png'*
* **serverName (optional)**: The server name for the goal action. Defaults to *'/move_base'*
* **actionName (optional)**: The action name for the goal action. Defaults to *'move_base_msgs/MoveBaseAction*

#### Example
```javascript
var NavigationClient = new RMV2D.NavigationClient({
  ros: ros,
  rootObject, mapView.scene,
  occupancyGridClient: occupancyGridClient,
  imageSrc: "marker.png"
});
```


### RMV2D.HeatmapClient
The HeatmapClient enables drawing a heatmap over the map. This object uses the *simpleheat.js* library to generate the heatmap.

#### Options
* **rootObject (required)**: The *scene* object of the *ROS2D.Viewer* object to display the heatmap in

#### Functions 
#####setData(options)
This function generates a heatmap with the given data and displays it in the Viewer. 

###### Example
```javascript
var heatmapClient = new RMV2D.HeatmapClient({
  rootObject: mapView.scene
});

var dataPoints = {
  
}



