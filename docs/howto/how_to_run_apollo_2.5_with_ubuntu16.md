# How to Run Apollo in Ubuntu 16 (Apollo 2.5)

We recommend to use Ubuntu 14 for Apollo. For the user using ubuntu 16, here is the extra step you have to follow to run Apollo 2.5 under ubuntu 16. 


1. You need glfw 3.2 and above, which supports EGL and run-time context creation API selection. (Refer to http://www.glfw.org/docs/latest/news.html)

Installed the version from https://launchpad.net/ubuntu/+source/glfw3.

2. Add following one line in glfw_fusion_viewer.cc:

glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);

(in GLFWFusionViewer::window_init() before the glfwCreateWindow() call)
 

With this, the perception_lowcost_vis finally works on my machine without a segfault.
 
The issue comes from some behavior changes in the latest nvidia driver and a glfw bug mentioned in http://www.glfw.org/docs/latest/window_guide.html#window_hints_ctx:

              “On some Linux systems, creating contexts via both the native and EGL APIs in a single process will cause the application to segfault. Stick to one API or the other on Linux for now.”

So, with the driver (on Ubuntu 16), glfw_fusion_viewer needs to be set up to use EGL_CONTEXT_API instead of the default NATIVE_CONTEXT_API to evade the segfault.

We plan to update the Apollo docker to support these.