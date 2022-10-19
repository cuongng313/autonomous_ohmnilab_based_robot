var appPage = new Vue({
      el: '#settingArea',
      data: userData,
      mounted() {
            this.connectPage()
      },
      methods: {
            connectPage: function () {

            },
            sendModeMsg: function (msg) {
                  if (this.connected) {
                        var string = new ROSLIB.Message({
                              data: msg
                        });
                        mode_selector.publish(string)
                  }
            },
      },
})