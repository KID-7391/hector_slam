# Hector SLAM

这一部分利用hector slam完成建图和定位，暂无全局定位功能，使用2D激光雷达和里程计。测试中使用的GUI改编自https://www.cnblogs.com/scomup/p/7074847.html 。

整体分为4步：  
1.motion prediction，即估计当前姿态和上一时刻的变化量  
2.scan matching，通过激光点云修正上一步的估计，也是该算法的重点  
3.map update，更新地图  
4.pose update，更新姿态  

## 地图
首先我们需要确定地图的格式，采用栅格地图。设一个二维数组$\overline{M}$，值域为$[-\infty, +\infty]$，数值越大，表示该点为障碍物的概率越大，反之越小。将其映射到$(0, 1)$上，可以看作概率，因此设$M_{ij} = \frac{e^{\overline{M}_{ij}}}{1+e^{\overline{M}_{ij}}}$，即可看作是概率栅格地图。源码在GridMap.py中，地图更新部分有很慢的Python实现，在注释部分中，也有较快的C++实现(c_extern/map_update.cpp)，测试效率差30倍。
### 地图更新
考虑激光点云中的一个点$P$和当前机器人位置$Q$，$P$附近的点是障碍物的概率应该增大，从$Q$到$P$线的点是障碍物的概率应该减小。

## motion prediction
这一步是为了将激光点云粗略地从机器人坐标系映射到世界坐标系。设k时刻的里程计对应变换矩阵为$T_k$，经过修正的姿态为$\widetilde{\xi}_k$，则可以初步估计k+1时刻姿态为$\widetilde{\xi}_kT^{-1}_kT_{k+1}$。

## scan matching
设激光点云有n个点，我们希望估计一个姿态$\xi=\begin{bmatrix}x\\y\\ \theta\end{bmatrix}$，使得点云尽可能分布在障碍物上，使用最小二乘，即求  <center>$\xi=argmin_{\xi}\sum_{i=0}^n[1-M(S_i(\xi))]^2$,</center>  

其中$S_i(\xi)$表示第i个点在姿态$\xi$下的坐标，$M$为已有的地图。设姿态变化量为$\Delta\xi$，优化目标为  <center>$\sum_{i=0}^n[1-M(S_i(\xi+\Delta\xi))]^2$，</center>   

对其泰勒展开  
<center>$\sum_{i=0}^n[1-M(S_i(\xi))-\nabla M(S_i(\xi))\frac{\partial S_i(\xi)}{\partial \xi}\Delta \xi]^2$</center>

求偏导并令为0
<center>$2\sum_{i=0}^n[\nabla M(S_i(\xi))\frac{\partial S_i(\xi)}{\partial \xi}]^T[1-M(S_i(\xi))-\nabla M(S_i(\xi))\frac{\partial S_i(\xi)}{\partial \xi}\Delta \xi]=0$</center>

求得
<center>$\Delta\xi=H^{-1}\sum_{i=1}^n[\nabla M(S_i(\xi))\frac{\partial S_i(\xi)}{\partial \xi}]^T[1-M(S_i(\xi))]$</center>

其中
<center>$H=\sum_{i=1}^n[\nabla M(S_i(\xi))\frac{\partial S_i(\xi)}{\partial \xi}]^T[\nabla M(S_i(\xi))\frac{\partial S_i(\xi)}{\partial \xi}]$</center>

更新姿态为$\xi=\xi+\Delta\xi$。

这一部分代码见SLAM.py。

### 栅格地图上偏微分求法
<center>
\begin{align*}
\nabla M(S_i(\xi))\frac{\partial S_i(\xi)}{\partial \xi}
=&\begin{bmatrix}
  \frac{\partial M(S_i(\xi))}{\partial x}\\
  \frac{\partial M(S_i(\xi))}{\partial y}\\
  \frac{\partial M(S_i(\xi))}{\partial \theta}
  \end{bmatrix}\\
=&\begin{bmatrix}
  \frac{\partial M(S_i(\xi))}{\partial x}\\
  \frac{\partial M(S_i(\xi))}{\partial y}\\
  (-xsin\theta-ycos\theta)\frac{\partial M(S_i(\xi))}{\partial x} + (xcos\theta-ysin\theta)\frac{\partial M(S_i(\xi))}{\partial y}
  \end{bmatrix}\\
=&\begin{bmatrix}
  1 & 0\\
  0 & 1\\
  -xsin\theta-ycos\theta & xcos\theta-ysin\theta
  \end{bmatrix}
  \begin{bmatrix}
    \frac{\partial M(S_i(\xi))}{\partial x}\\
    \frac{\partial M(S_i(\xi))}{\partial y}
  \end{bmatrix}
\end{align*}
</center>

其中的
<center>
$$
\begin{bmatrix}
  \frac{\partial M(S_i(\xi))}{\partial x} \\
  \frac{\partial M(S_i(\xi))}{\partial y}
\end{bmatrix}
$$
</center>

可以用附近的四个点做双线性插值，不妨设$S_i(\xi)=\begin{bmatrix}x\\y\end{bmatrix}$，
则有

<center>
\begin{align*}
\frac{\partial M(S_i(\xi))}{\partial x}
\approx&([y]+1-y)(M_{[x]+1,[y]}-M_{[x],[y]})+(y-[y])(M_{[x]+1,[y+1]}-M_{[x],[y+1]})
\end{align*}

\begin{align*}
\frac{\partial M(S_i(\xi))}{\partial y}
\approx&([x]+1-x)(M_{[x],[y]+1}-M_{[x],[y]})+(x-[x])(M_{[x]+1,[y+1]}-M_{[x+1],[y]})
\end{align*}
</center>
