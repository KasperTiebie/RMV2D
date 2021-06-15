var RMV2D = RMV2D || {
  REVISION : '1.0.0'
};

// convert a ROS quaternion to theta in degrees
var rosQuaternionToGlobalTheta = function(orientation) {
  // See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Rotation_matrices
  // here we use [x y z] = R * [1 0 0]
  var q0 = orientation.w;
  var q1 = orientation.x;
  var q2 = orientation.y;
  var q3 = orientation.z;
  // Canvas rotation is clock wise and in degrees
  return -Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0 / Math.PI;
};

RMV2D.Viewer = function(options){
	var that = this;
	var divID = options.divID; //Div to generate the canvas object in
	var width = options.width; //Width and height of the canvas object has to be predefined
	var height = options.height;
	var background = options.background || '#111111';
	var enableZoom = options.enableZoom || false;
	var enablePan = options.enablePan || false;

	var mouseDown = false; // If the mouse button is currently pressed
	var startX; // Start position of the scene when the mouse button gets pressed
	var startY;
	var startMouseX; // Start position of the mouse when the mouse button gets pressed
	var startMouseY;

	// Create the canvas to render to
	var canvas = document.createElement('canvas');
	canvas.width = width;
	canvas.height = height;
	canvas.style.background = background;
	document.getElementById(divID).appendChild(canvas);

	// Create the easel to use
	this.scene = new createjs.Stage(canvas);

	// Set the inital scale
	this.scene.scaleX = 100; // Set initial scale of 100 pixels per meter
	this.scene.scaleY = 100;

	// Update at 30fps
	createjs.Ticker.setFPS(30);
	createjs.Ticker.addEventListener('tick', this.scene);

	// Scale is in pixels/m
	this.scene.zoom = function(scale){
		this.scaleX = scale*this.resolution;
		this.scaleY = scale*this.resolution;
	}

	this.scene.pan = function(positionX, positionY){
		this.x = positionX;
		this.y = positionY;
	}

	// Event listener for the panning of the map
	if(enablePan){
		this.scene.on("stagemousedown", function(event) {
			mouseDown = true;
			startX = this.x;
			startY = this.y;
			startMouseX = event.rawX;
			startMouseY = event.rawY;
		});

		this.scene.on("stagemousemove", function(event) {
			if(mouseDown){
				this.x = (startX + (event.rawX - startMouseX));
		    	this.y = (startY + (event.rawY - startMouseY));
			}
		});

		this.scene.on("stagemouseup", function(event) {
			mouseDown = false;
		});
	}

	if(enableZoom){
		canvas.addEventListener("wheel", function(event){
			var mouseX = event.layerX - that.scene.x; //Mouse position relative to 0,0. Easel always zooms towards 0,0. That's why a translation is also necessary in order to zoom towards the cursor
			var mouseY = event.layerY - that.scene.y;
			if(event.deltaY >= 0){ // Zoom out
				that.scene.x -= mouseX/1.1 - mouseX;  
				that.scene.y -= mouseY/1.1 - mouseY;
				that.scene.scaleX /= 1.1;
				that.scene.scaleY /= 1.1;
			}else if(event.deltaY <= 0){ // Zoom in
				that.scene.x -= mouseX*1.1 - mouseX;
				that.scene.y -= mouseY*1.1 - mouseY;
				that.scene.scaleX *= 1.1;
				that.scene.scaleY *= 1.1;
			}
		});
	}
};

RMV2D.OccupancyGrid = function(options){
	var message = options.message;

	var canvas = document.createElement("canvas");
	var context = canvas.getContext("2d");

	this.width = message.info.width;
	this.height = message.info.height;
	canvas.width = this.width;
	canvas.height = this.height;


	var imageData = context.createImageData(this.width, this.height);
	for(var row = 0; row < this.height; row++){
		for(var col = 0; col < this.width; col++){
			var map_index = col + ((this.height - row - 1) * this.width); //row * this.width + col;
			var data = message.data[map_index];
			var val;
			if(data === -1){
				val = 127;
			}else{
				val = 255 - (data / 100 * 255);
			}

			var image_index = (col + (row * this.width)) * 4;
			// r
			imageData.data[image_index] = val;
			// g
			imageData.data[image_index+1] = val;
			// b
			imageData.data[image_index+2] = val;
			// a
			imageData.data[image_index+3] = 255;
		}
	}

	context.putImageData(imageData, 0, 0);

	// Create the bitmap
	createjs.Bitmap.call(this, canvas);

	// change Y direction
	this.y = -this.height * message.info.resolution;

	// scale the image
	this.scaleX = message.info.resolution;
	this.scaleY = message.info.resolution;
	this.width *= this.scaleX;
	this.height *= this.scaleY;

	// set the pose
	this.x += message.info.origin.position.x;
	this.y -= message.info.origin.position.y;
}
RMV2D.OccupancyGrid.prototype.__proto__ = createjs.Bitmap.prototype;

RMV2D.OccupancyGridClient = function(options){
	var that = this;
	var ros = options.ros;
	var mapTopic = options.mapTopic || "/map";
	this.rootObject = options.rootObject || new createjs.Container();

	// Subscribe to the map topic
	var rosMapTopic = new ROSLIB.Topic({
		ros : ros,
		name : mapTopic,
		messageType : "nav_msgs/OccupancyGrid",
		compression : "png"
	});

	rosMapTopic.subscribe(function(message){
		// Check if an old map exists, and if so, delete it from the scene
		var index;
		if(that.occupancyGrid){
			index = that.rootObject.getChildIndex(that.occupancyGrid);
			that.rootObject.removeChild(that.occupancyGrid);
		}

		// Create a new occupancyGrid and save it in the OccupancyGridClient object
		that.occupancyGrid = new RMV2D.OccupancyGrid({
			message : message
		});

		// Show the new occupancyGrid by adding it as a child to the scene
		if(index !== null){
			that.rootObject.addChildAt(that.occupancyGrid, index);
		}else{
			that.rootObject.addChild(that.occupancyGrid);
		}

		that.emit("change");
	});
};
RMV2D.OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;

RMV2D.PoseClient = function(options) {
	var that = this;
	var ros = options.ros;
	var topic = options.topic || "/robot_pose";
	var imageSrc = options.imageSrc || "robotPose.png";
	this.rootObject = options.rootObject || new createjs.Container();
	this.trackPose = options.trackPose || false;
	this.trackOrientation = options.trackOrientation || false;

	that.poseImage = new RMV2D.NavigationImage({
						size : 0.3,
						imageSrc : imageSrc,
						alpha : 1,
						pulse : true,
						positionX : 0,
						positionY : 0
					});

	// Add the poseImage to the scene
	that.rootObject.addChild(that.poseImage);

	var poseListener = new ROSLIB.Topic({
		ros : ros,
		name : topic,
		messageType : 'geometry_msgs/Pose',
		throttle_rate : 100
	}); 

	poseListener.subscribe(function(message){
		// Get the index of the poseImage
		var index = that.rootObject.getChildIndex(that.poseImage);	
		var poseImageObject = that.rootObject.children[index]
		poseImageObject.x = message.position.x;
		poseImageObject.y = -message.position.y;
		poseImageObject.rotation = rosQuaternionToGlobalTheta(message.orientation)+90;

		if(that.trackPose){
			// Calculate the position in order the keep the pose centered
			// The incoming position is in meters. The viewport is based on it's scale, so a small conversion has to be done
			var positionX = -(message.position.x - (that.rootObject.canvas.width/that.rootObject.scaleX)/2)*that.rootObject.scaleX;
			var positionY = -(-message.position.y - (that.rootObject.canvas.height/that.rootObject.scaleY)/2)*that.rootObject.scaleY;
			that.rootObject.pan(positionX, positionY);
		}

		if(that.trackOrientation){
			var orientation = rosQuaternionToGlobalTheta(message.orientation);
			that.rootObject.rotation = 270-orientation;
			that.rootObject.regX = message.position.x; //Set the registration point equal to the center of the dog. This way the map rotates around the dog
			that.rootObject.regY = -message.position.y;
			that.rootObject.x += message.position.x*that.rootObject.scaleX; //Because of the change in the registration point, another transformation has to be done
			that.rootObject.y += -message.position.y*that.rootObject.scaleY;
		}

		that.emit("change");
	});

	this.panToPose = function(){
		var positionX = -(this.poseImage.x - (this.rootObject.canvas.width/this.rootObject.scaleX)/2)*this.rootObject.scaleX;
		var positionY = -(this.poseImage.y - (this.rootObject.canvas.height/this.rootObject.scaleY)/2)*this.rootObject.scaleY;
		this.rootObject.pan(positionX, positionY);
	}
};
RMV2D.PoseClient.prototype.__proto__ = EventEmitter2.prototype;

RMV2D.NavigationImage = function(options) {
	var that = this;
	var size = options.size || 10;
	var imageSrc = options.imageSrc || "marker.png";
	var pulse = options.pulse || false;
	this.alpha = options.alpha || 1;
	this.x = options.positionX || 0;
	this.y = options.positionY || 0;

	var paintImage = function(){
		var scale = (size/image.width);
		createjs.Bitmap.call(that, this);
		that.scaleX = scale;
		that.scaleY = scale;
		that.regX = image.width/2;
		that.regY = image.height/2;
	}

	var image = new Image();
	image.src = imageSrc;
	image.onload = paintImage;

	// check if we are pulsing
	if (pulse) {
		// have the model "pulse"
		var growCount = 0;
		var growing = true;
		createjs.Ticker.addEventListener('tick', function() {
			if (growing) {
				that.scaleX *= 1.01;
				that.scaleY *= 1.01;
				growing = (++growCount < 15);
			} else {
				that.scaleX /= 1.01;
				that.scaleY /= 1.01;
				growing = (--growCount < 0);
			}
		});
	}
};
RMV2D.NavigationImage.prototype.__proto__ = createjs.Bitmap.prototype;


// The navigationclient isn't supposed 
RMV2D.NavigationClient = function(options){
	var that = this;
	var ros = options.ros;
	this.rootObject = options.rootObject || new createjs.Container();
	this.occupancyGridClient = options.occupancyGridClient;
	var imageSrc = options.imageSrc || "goalMarker.png";
	var serverName = options.serverName || "/move_base";
  	var actionName = options.actionName || "move_base_msgs/MoveBaseAction";
	var doubleClick = false;
	var doubleClickTimeout;

	// setup the actionlib client
	var actionClient = new ROSLIB.ActionClient({
		ros : ros,
		actionName : actionName,
		serverName : serverName
	});

	this.rootObject.on("stagemousedown", function(event) {
		if(doubleClick){
			// If a goalImage already exists, delete it
			var goalImageIndex;
			if(that.goalImage){
				goalImageIndex = that.rootObject.getChildIndex(that.goalImage);
				that.rootObject.removeChild(that.goalImage);
			}

			var occupancyGridIndex = that.rootObject.getChildIndex(that.occupancyGridClient.occupancyGrid);	
			var occupancyGridObject = that.rootObject.children[occupancyGridIndex];
			var positionX = event.stageX/that.rootObject.scaleX-that.rootObject.x/that.rootObject.scaleX;
			var positionY = event.stageY/that.rootObject.scaleY-that.rootObject.y/that.rootObject.scaleY;

			that.goalImage = new RMV2D.NavigationImage({
				size : 0.3,
				imageSrc : imageSrc,
				pulse : false,
				alpha : 0.8,
				positionX : positionX,
				positionY : positionY
			});

			// Show the new goalImage by adding it as a child to the scene
			if(goalImageIndex != null){
				that.rootObject.addChildAt(that.goalImage, goalImageIndex);
			}else{
				that.rootObject.addChild(that.goalImage);
			}

			// create a pose
			var pose = new ROSLIB.Pose({
	          position : new ROSLIB.Vector3({
	          	x : positionX,
	          	y : -positionY,
	          	z : 0
	          })
	        });

			var goal = new ROSLIB.Goal({
				actionClient : actionClient,
				goalMessage : {
					target_pose : {
					  header : {
					  	frame_id : "map"
					  },
					  pose : pose
					}
				}
			});
			goal.send();

			goal.on('result', function() {
				that.rootObject.removeChild(that.goalImage);
				delete that.goalImage;
			}); 
			doubleClick = false;
		}

		doubleClick = true;
		doubleClickTimeout = setTimeout(function(){
			doubleClick = false;
		}, 500);
	});
}

RMV2D.HeatmapClient = function(options){
	var that = this;
	this.rootObject = options.rootObject; 

	// Create a new heatmap based on new points
	this.setData = function(points){
		var heatmapIndex;
		if(that.heatmap){
			delete that.heatmap.canvasContainer;
			heatmapIndex = that.rootObject.getChildIndex(that.heatmap);
			that.rootObject.removeChild(that.heatmap);
		}

		that.heatmap = new RMV2D.Heatmap({
			points: points
		});

		if(heatmapIndex != null){
			that.rootObject.addChildAt(that.heatmap, heatmapIndex);
		}else{
			that.rootObject.addChild(that.heatmap);
		}
	}

	// Clear the heatmap (remove it from the viewport)
	this.clear = function(){
		if(that.heatmap){
			that.heatmap.canvas.remove();
			that.rootObject.removeChild(that.heatmap);
			delete that.heatmap;
		}
	}
}

RMV2D.Heatmap = function(options){
	var that = this;
	var points = options.points;
	//The area to render around the outermost points so it doesn't appear to be cut off
	var margin = 2;
	var data = [];
	// Transform point objects with x, y and val, to an array in the form of [[x, y, val], [x, y, val], ...]
	for(i in points.data){
		var point = [points.data[i].x, points.data[i].y, points.data[i].value];
		data.push(point);
	}

	this.canvas = document.createElement("canvas");

	// Get the minimum and maximum values of the x and y coordinates so a width/height can be made
	//0: x, 1: y, 2: val
	var minPointsX = Math.min.apply(Math, data.map(function(o){return o[0];}));
	var maxPointsX = Math.max.apply(Math, data.map(function(o){return o[0];}));
	var minPointsY = Math.min.apply(Math, data.map(function(o){return o[1];}));
	var maxPointsY = Math.max.apply(Math, data.map(function(o){return o[1];}));

	// Increment all values with the minimum value. If the minimum value is 10, all points get shifted down by -10. If the minimum value is -15, all points will be shifted up by 15.
	for(var i = 0; i < data.length; i++){
		data[i][0] -= minPointsX;
		data[i][1] -= minPointsY;
		data[i][0] += margin;
		data[i][1] += margin;
		data[i][0] *= 100;
		data[i][1] *= 100;
	}

	this.canvas.width = (maxPointsX - minPointsX + margin*2)*100;
	this.canvas.height = (maxPointsY - minPointsY + margin*2)*100;
	this.canvas.id = "heatmapCanvas";
	this.canvas.style.visibility = "hidden";

	this.width = maxPointsX - minPointsX + margin*2;
	this.height = maxPointsY - minPointsY + margin*2;

	// The container has to be appended to the web page in order to draw on the canvas
	document.body.appendChild(this.canvas);

	var heatmapInstance = simpleheat("heatmapCanvas");
	heatmapInstance.data(data);
	heatmapInstance.max(points.max);
	heatmapInstance.radius(25, 15);
	heatmapInstance.draw();

	var canvas = this.canvas;
	createjs.Bitmap.call(this, canvas);
	this.scaleX = 1/100;
	this.scaleY = 1/100;
	this.x = minPointsX - margin;
	this.y = minPointsY - margin;
	this.alpha = 0.2;
	this.canvas.remove();
}
RMV2D.Heatmap.prototype.__proto__ = createjs.Bitmap.prototype;

