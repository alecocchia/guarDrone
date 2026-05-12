## ROS2 Falcon
# Some problems of haptic devices fixed
1)Critic bug (Buffer Overflow): In the original node, the vector which minimizes the inertia (hw_states_inertia_) was dimensioned as 15 elements, but during the reading cycle it was trying to write 21 elements (the triangular part of a 6x6 matrix). This caused a "corruption of memory" (heap corruption). When ROS tried to load the following plugin, it found a damaged memory and went in Segmentation Fault.
2)Incompatibility of ABI (C++14 vs C++17): packages where based on C++14, while ROS 2 Humble uses C++17. This caused issues in the communication between libraries of the system of ROS2 and plugins.
3)Global logger: The use of an object "global logger" in a plugin is risky, because it can be initialized too soon, causing crashes
