# How to Run Apollo in Ubuntu 16 (Apollo 2.5)

We recommend using **Ubuntu 14.04** for Apollo. For those users that currently have Ubuntu 16 installed, here is the extra step you have to follow to run Apollo 2.5 successfully on your Ubuntu 16 machine. 


1. You need GLFW(Graphics Library Framework) 3.2 and above, which supports EGL and run-time context creation API selection. (Refer to [http://www.glfw.org/docs/latest/news.html](http://www.glfw.org/docs/latest/news.html) for additional information)
Install the correct version from [this link](https://launchpad.net/ubuntu/+source/glfw3).

2. Add following one line in `glfw_fusion_viewer.cc` file, in GLFWFusionViewer::window_init() before the glfwCreateWindow() call:
    ```
    glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
    ```


With this, the `perception_lowcost_vis` should work on your machine without a segfault.
 
The issue comes from a behavior changes in the latest nvidia driver and a glfw bug. You can find information about the bug [here](http://www.glfw.org/docs/latest/window_guide.html#window_hints_ctx). To summarize the behaviour,

```
“On some Linux systems, creating contexts via both the native and EGL APIs in a single process 
will cause the application to segfault. Stick to one API or the other on Linux for now.”
```

So, with the driver (on Ubuntu 16), glfw_fusion_viewer needs to be set up to use EGL_CONTEXT_API instead of the default NATIVE_CONTEXT_API to evade the segfault.

We plan to update the Apollo docker to support these soon!