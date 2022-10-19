

var userData = {
      connected: false,
      // ros: null,
      ws_address: 'ws://192.168.100.9:9090',
      //ws_address: 'ws://192.168.0.107:9090',
      //ws_address: 'ws://localhost:9090',
      logs: [],
      mode_selector: null,
      zone_selector: null,

      mode_listener: null,
      zone_listener: null,

      ohmni_listener: null,

}
var page1Data = {
      viewer: null,
      tfClient: null,
      mapClient: null,
      markerClient: null,
      laser: null,
      pose: null,
}
var ros = null;
var mode_selector = null;

var originX = 0.0
var originY = 0.0
function callPhp(option) {
      $.ajax({
            url: 'temp.php',
            type: 'post',
            data: { "test": option },
            success: function (response) {
                  //alert(response);
            }
      });
}

