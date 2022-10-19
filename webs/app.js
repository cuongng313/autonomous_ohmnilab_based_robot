var app = new Vue({
      el: '#app',
      data: userData,
      mounted() {
            
            this.connect()
      },
      methods: {
            connect: function () {
 
                  ros = new ROSLIB.Ros({
                        url: this.ws_address
                  })
                  this.addLog('Connect to rosbridge server')
                  ros.on('connection', () => {
                        this.connected = true
                        this.logs.push('Connected')
                        // MODE PUB/SUB
                        mode_selector = new ROSLIB.Topic({
                              ros: ros,
                              name: '/mode_manager',
                              messageType: 'std_msgs/Int8'
                        })

                        this.mode_listener = new ROSLIB.Topic({
                              ros: ros,
                              name: '/mode_manager',
                              messageType: 'std_msgs/Int8'
                        })
                        // ZONE PUB/SUB
                        this.zone_selector = new ROSLIB.Topic({
                              ros: ros,
                              name: '/zone_select',
                              messageType: 'std_msgs/Int8'
                        })
                        this.zone_listener = new ROSLIB.Topic({
                              ros: ros,
                              name: '/zone_select',
                              messageType: 'std_msgs/Int8'
                        })

                        this.ohmni_listener = new ROSLIB.Topic({
                              ros: ros,
                              name: '/OhmniStatus',
                              messageType: 'std_msgs/Int8'
                        })

                        this.mode_listener.subscribe(this.receiveModeMsg)
                        this.zone_listener.subscribe(this.receiveZoneMsg)

                        this.ohmni_listener.subscribe(this.receiveOhmniMsg)
                        
                  })
                  ros.on('error', (error) => {
                        console.log('Error: ', error)
                        this.addLog('Error connecting to rosbridge server')
                  })
                  ros.on('close', () => {
                        this.connected = false
                        this.addLog('Connection to rosbridge server closed')
                  })


            },
            disconnect: function () {
                  ros.close()
            },
            sendModeMsg: function (msg) {
                  if (this.connected) {
                        var string = new ROSLIB.Message({
                              data: msg
                        });
                        mode_selector.publish(string)
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

            receiveOhmniMsg: function (msg) {
                  this.addLog('OhmniSubscribe: ' + msg.data.toString())
                  if(msg.data==1)
		            callPhp('play_sound')
            },

            addLog: function (msg) {
                  this.logs.push(msg)
                  while (this.logs.length > 5) {
                        this.logs.shift()
                  }
            },




      },
})
