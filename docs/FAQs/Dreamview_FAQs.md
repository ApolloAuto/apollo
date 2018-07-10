# DreamView FAQs

## I’m having difficulty connecting to localhost:8888 (Dreamview).

The Dreamview web server is provided by the dreamview node(A node is an executable in ROS concept). Before accessing the Dreamview page, you need to build the system(including dreamview node) within the docker container following the [guide](https://github.com/ApolloAuto/apollo/blob/master/README.md). Once built, dreamview node will be started after the step `bash scripts/bootstrap.sh`.

So if you can not access Dreamview, please check:

* Make sure you have dreamview process running correctly. In the latest version, `bash scripts/bootstrap.sh` will report `dreamview: ERROR (spawn error)` if dreamview fails to start. For early version, please check with command: `supervisorctl status dreamview` or `ps aux | grep dreamview`. If dreamview is not running, please refer to [How to Debug a Dreamview Start Problem](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_debug_dreamview_start_problem.md).
* Make sure the address and port are not blocked by the firewall.
* Make sure you're using <apollo_host_ip>:8888 instead of localhost:8888 if you are not accessing the Dreamview page through the host machine.

---
## Dreamview does not open up if I install more than 1 version of Apollo?

This issue occured because of port conflict error.
Even though you setup two different docker environments, both of them are still trying to use port 8888 on your machine, therefore causing a port conflict issue. If you'd like to run both versions at the same time, please make sure different ports are set.

To do so,

1. Open `dreamview.conf` file under modules/dreamview/conf
add ```--server_ports=<PORT_NUMBER>``` to the end of the file.
Ex:
--flagfile=modules/common/data/global_flagfile.txt
--server_ports=5555
2. Restart apollo

This way, dreamview can be accessed from http://localhost:<PORT_NUMBER> (http://localhost:5555 from the example)

---
## How to draw anything in DreamView (e.g. an arrow)

Dreamview uses https://github.com/mrdoob/three.js as graphics library. You can modify the frontend code to draw an arrow using the corresponding API of the library. After that you need to run a `./apollo.sh build_fe` to compile.

---
## How can I test planning algorithms offline?

Use dreamview and enable sim_control on dreamview to test your planning algorithm.

---
## What's the function of sim_control in the backend of dreamview

It simulates a SDC's control module, and moves the car based on planning result. This is a really convenient way to visualize and test planning module

---
## 

**More DreamView FAQs to follow.**
