<?php 
    

    function test($option)
    {
          echo shell_exec("bash utils.sh $option");
    }
    if (isset($_POST['test'])) {
        echo test($_POST['test']);
    }
    function getIp($interface) {
        //$localIP = getHostByName(getHostName());
        echo shell_exec("ip addr show $interface | grep \"inet\b\" | awk '{print $2}' | cut -d/ -f1");
    }
?>