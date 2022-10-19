// import * as cv from 'js/opencv.js'
// import userData from 'common.js'
var userData = {
      connected: false,
      ros: null,
      ws_address: 'ws://192.168.100.9:9090',
      //ws_address: 'ws://10.111.195.136:9090',
      //ws_address: 'ws://localhost:9090',
      logs: [],
      mode_selector: null,
      zone_selector: null,

      mode_listener: null,
      zone_listener: null,
      viewer: null,
      tfClient: null,
      mapClient: null,
      markerClient: null,
      laser: null,
      pose: null,
      methods: {
            connect: function () {

            },
      },

};
var app = new Vue({
      el: '#app',
      data: userData,
      mounted() {
            this.connect()
      },
      methods: {
            connect: function () {
                  this.ros = new ROSLIB.Ros({
                        url: this.ws_address
                  })
                  this.addLog('Connect to rosbridge server')
                  this.ros.on('connection', () => {
                        this.connected = true
                        this.logs.push('Connected')
                        // MODE PUB/SUB
                        this.mode_selector = new ROSLIB.Topic({
                              ros: this.ros,
                              name: '/mode_manager',
                              messageType: 'std_msgs/Int8'
                        })

                        this.mode_listener = new ROSLIB.Topic({
                              ros: this.ros,
                              name: '/mode_manager',
                              messageType: 'std_msgs/Int8'
                        })
                        // ZONE PUB/SUB
                        this.zone_selector = new ROSLIB.Topic({
                              ros: this.ros,
                              name: '/zone_select',
                              messageType: 'std_msgs/Int8'
                        })
                        this.zone_listener = new ROSLIB.Topic({
                              ros: this.ros,
                              name: '/zone_select',
                              messageType: 'std_msgs/Int8'
                        })

                        this.mode_listener.subscribe(this.receiveModeMsg)
                        this.zone_listener.subscribe(this.receiveZoneMsg)

                        // Viewer:
                        // this.viewer = new ROS2D.Viewer({
                        //       divID: 'map',
                        //       width: 600,
                        //       height: 600
                        // })
                        // this.nav = NAV2D.OccupancyGridClientNav({
                        //       ros: this.ros,
                        //       rootObject: this.viewer.scene,
                        //       viewer: this.viewer,
                        //       serverName: '/move_base'
                        // });
                        this.viewer = new ROS3D.Viewer({
                              divID: 'map',
                              width: 600,
                              height: 600,
                              antialias: true,
                              intensity: 1.0,
                              cameraPose: { x: -1, y: 0, z: 20 },
                              displayPanAndZoomFrame: true
                        });
                        // this.viewer.addObject(new ROS3D.Grid())

                        // Setup a client to listen to TFs.
                        this.tfClient = new ROSLIB.TFClient({
                              ros: this.ros,
                              angularThres: 0.01,
                              transThres: 0.01,
                              rate: 10.0,
                              fixedFrame: 'map'
                        });

                        this.mapClient = new ROS3D.OccupancyGridClient({
                              ros: this.ros,
                              rootObject: this.viewer.scene,
                              continuos: true,
                              serverName: '/map'
                        })
                        this.laser = new ROS3D.LaserScan({
                              ros: this.ros,
                              tfClient: this.tfClient,
                              rootObject: this.viewer.scene,
                              topic: '/scan',

                              max_pts: 10000,
                              pointRatio: 1,
                              messageRatio: 1,
                              visible: true,
                              material: {
                                    size: 0.5,
                                    color: 0xff00ff
                              }
                        });
                        // this.pose = new ROS3D.PoseWithCovariance({
                        //       ros: this.ros,
                        //       tfClient: this.tfClient,
                        //       rootObject: this.viewer.scene,
                        //       topic: '/amcl_pose'
                        // });

                        // Setup the marker client.
                        this.markerClient = new ROS3D.MarkerClient({
                              ros: this.ros,
                              tfClient: this.tfClient,
                              topic: '/visualization_marker',
                              lifetime: 0,
                              rootObject: this.viewer.scene
                        });

                  })
                  this.ros.on('error', (error) => {
                        console.log('Error: ', error)
                        this.addLog('Error connecting to rosbridge server')
                  })
                  this.ros.on('close', () => {
                        this.connected = false
                        this.addLog('Connection to rosbridge server closed')
                  })

            },
            disconnect: function () {
                  this.ros.close()
            },
            sendModeMsg: function (msg) {
                  if (this.connected) {
                        var string = new ROSLIB.Message({
                              data: msg
                        });
                        this.mode_selector.publish(string)
                  }
            },

            sendZoneMsg: function (msg) {
                  if (this.connected) {
                        var string = new ROSLIB.Message({
                              data: msg
                        });
                        this.zone_selector.publish(string)
                  }
            },

            receiveZoneMsg: function (msg) {
                  this.addLog('ZoneSubscribe: ' + msg.data.toString())
            },
            receiveModeMsg: function (msg) {
                  this.addLog('ModeSubscribe: ' + msg.data.toString())
            },


            addLog: function (msg) {
                  // this.logs = []
                  this.logs.push(msg)
                  while (this.logs.length > 5) {
                        this.logs.shift()
                  }
            },

            movingHandler: function (event) {

                  var x = event.touches[0].clientX;
                  var y = event.touches[0].clientY;
                  document.getElementById("demo").innerHTML = x + ", " + y;
                  event.stopPropagation();
            },


      },
})
