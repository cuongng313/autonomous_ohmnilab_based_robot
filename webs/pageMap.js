var appPage = new Vue({
      el: '#mapArea',
      data: page1Data,
      mounted() {
            this.connectPage()
      },
      methods: {
            connectPage: function () {

                  var pose_js = new ROSLIB.Topic({
                        ros: ros,
                        name: "/pose_js",
                        messageType: 'geometry_msgs/Pose',

                  })
                  var touch_area = document.getElementById("touchArea")
                  var x_old = 0.0
                  var y_old = 0.0

                  
                  var deltaX = 0.0
                  var deltaY = 0.0
                  var valid = false

                  var scaleX = 300
                  var scaleY = 300


                  var marker_msg = new ROSLIB.Message({
                        header: {
                              frame_id: '/map',
                              stamp: {
                                    secs: 0,
                                    nsecs: 0
                              }
                        },
                        scale: {
                              x: 1.0,
                              y: 1.0,
                              z: 1.0
                        },
                        color: {
                              r: 0.0,
                              g: 1.0,
                              b: 0.0,
                              a: 1.0
                        },
                        pose: {
                              position: {
                                    x: 0,
                                    y: 0,
                                    z: 0
                              },
                              orientation: {
                                    x: 0,
                                    y: 0,
                                    z: 0,
                                    w: 1
                              },
                        },
                        type: 2,
                        action: 0, // ADD
                        ns: "basic_shapes",
                        id: 0
                  })

                  this.viewer = new ROS3D.Viewer({
                        divID: 'map',
                        width: 800,
                        height: 600,
                        antialias: true,
                        intensity: 1.0,
                        cameraPose: { x: 0, y: -1, z: 40 },
                        displayPanAndZoomFrame: true
                  })
                  // this.viewer.addObject(new ROS3D.Grid())

                  // Setup a client to listen to TFs.
                  this.tfClient = new ROSLIB.TFClient({
                        ros: ros,
                        angularThres: 0.01,
                        transThres: 0.01,
                        rate: 10.0,
                        fixedFrame: '/map'
                  })

                  this.mapClient = new ROS3D.OccupancyGridClient({
                        ros: ros,
                        rootObject: this.viewer.scene,
                        continuos: true,
                        serverName: '/map'
                  })

                  this.laser = new ROS3D.LaserScan({
                        ros: ros,
                        tfClient: this.tfClient,
                        rootObject: this.viewer.scene,
                        topic: '/scan',

                        max_pts: 10000,
                        pointRatio: 1,
                        messageRatio: 1,
                        visible: true,
                        material: {
                              size: 0.2,
                              color: 0xff00ff
                        }
                  })
                  this.pose_js = new ROSLIB.Topic({
                        ros: ros,
                        name: "/pose_js",
                        messageType: 'geometry_msgs/Pose',

                  })
                  var marker_js = new ROSLIB.Topic({
                        ros: ros,
                        name: "/marker_js",
                        messageType: 'visualization_msgs/Marker',
                  })
                  // this.pose = new ROS3D.PoseWithCovariance({
                  //       ros: this.ros,
                  //       tfClient: this.tfClient,
                  //       rootObject: this.viewer.scene,
                  //       topic: '/amcl_pose'
                  // });

                  // Setup the marker client.
                  this.markerClient = new ROS3D.MarkerClient({
                        ros: ros,
                        tfClient: this.tfClient,
                        topic: '/marker_js',
                        lifetime: 0,
                        rootObject: this.viewer.scene
                  })


                  // TOUCH AREA
                  // touch_area.addEventListener('mousedown', function (event) {
                  //       var pose_msg = new ROSLIB.Message({
                  //             position: {
                  //                   x: 0.0,
                  //                   y: 0.0,
                  //                   z: 0.0
                  //             },
                  //             orientation: {
                  //                   x: 0.0,
                  //                   y: 0.0,
                  //                   z: 0.0,
                  //                   w: 1.0
                  //             }
                  //       })
                  //       pose_js.publish(pose_msg)
                  // })
                  touch_area.addEventListener('touchstart', function (event) {
                        valid = false
                  })
                  touch_area.addEventListener('touchend', function (event) {

                  })
                  touch_area.addEventListener('touchmove', function (event) {
                        var x = event.touches[0].clientX;
                        var y = event.touches[0].clientY;
                        if (valid == false) {
                              x_old = x
                              y_old = y
                              valid = true
                        } else {
                              deltaX = (x - x_old) / scaleX
                              deltaY = (y - y_old) / scaleY


                              var currentTime = new Date();
                              var secs = Math.floor(currentTime.getTime() / 1000);
                              var nsecs = Math.round(1000000000 * (currentTime.getTime() / 1000 - secs));
                              marker_msg.header.stamp.secs = secs
                              marker_msg.header.stamp.nsecs = nsecs


                              //marker_msg.id ^= 1
                              originX += deltaX
                              originY -= deltaY

                        }
                        event.preventDefault()
                  }, false)
                  setInterval(function () {
                        marker_msg.pose.position.x = originX
                        marker_msg.pose.position.y = originY

                        marker_msg.id ^= 1
                        marker_msg.action = 0
                        marker_js.publish(marker_msg)
                        document.getElementById("xyplane").innerHTML = "X: " + originX + ", Y: " + originY
                  }, 50)


            },
            sendPoseMsg: function () {
                  var pose_msg = new ROSLIB.Message({
                        position: {
                              x: originX,
                              y: originY,
                              z: 0.0
                        },
                        orientation: {
                              x: 0.0,
                              y: 0.0,
                              z: 0.0,
                              w: 1.0
                        }
                  })
                  this.pose_js.publish(pose_msg)
            },
            resetPose: function() {
                  originX=0
                  originY=0
            },

      },
})