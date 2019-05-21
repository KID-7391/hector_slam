# 两轮差速机器人建图定位导航

今年学院买了新的机器人(水滴，是个两轮差速机器人)，课程实验就变成在水滴上实现我们这学期学过的东西，于是开始手撸建图和导航，终于做完了。在此做一个记录，也给大家一个参考，毕竟现在搜到的博客大部分是调包，对于理解算法意义不大。

实现功能为控制机器人建图，然后自动导航回原位置，主要分为SLAM、路径规划、路径跟踪三个部分。
代码主要由Python实现，少部分为了效率，用C++实现。原则是简单易实现，暂不考虑精度，总代码量在700行左右。源码已上传至github:https://github.com/KID-7391/hector_slam

1.hector slam 实现:https://github.com/KID-7391/hector_slam/blob/master/doc/hector_slam.md
2.路径规划:https://github.com/KID-7391/hector_slam/blob/master/doc/path_planning.md
3.路径跟踪(线性控制器):https://github.com/KID-7391/hector_slam/blob/master/doc/ppath_tracking.md
