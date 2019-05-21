# Path Tracking

这一部分利用极坐标下的线性控制器完成路径跟踪，主要思路就是把路径拆开，变成p2p问题，即从当前姿态到目标姿态。

## 极坐标下的线性控制器
设速度$v$，角速度$\omega$，则有两轮差速机器人在惯性坐标系下的模型
<center>
$$
\begin{bmatrix}
  \dot x\\
  \dot y\\
  \dot \theta
\end{bmatrix}=
\begin{bmatrix}
  cos \theta & 0\\
  sin \theta & 0\\
  0 & 1
\end{bmatrix}
\begin{bmatrix}
   v\\
   \omega
\end{bmatrix}
$$
</center>

如图转换到误差极坐标系下
![image](https://github.com/KID-7391/hector_slam/blob/master/doc/aaa.png)
$\rho=\sqrt{\Delta x^2+\Delta y^2}$
$\alpha=-\theta+arctan2(\Delta y, \Delta x)$
$\beta=-\theta-\alpha$

<center>
$$
\begin{bmatrix}
  \dot \rho\\
  \dot \alpha\\
  \dot \beta
\end{bmatrix}=
\begin{bmatrix}
  -cos \alpha & 0\\
  \frac{sin \alpha}{\rho} & -1\\
  -\frac{sin \alpha}{\rho} & 0
\end{bmatrix}
\begin{bmatrix}
   v\\
   \omega
\end{bmatrix}
$$
</center>

设计线性控制器
$v = k_{\rho}\rho$
$\omega = k_{\alpha}\alpha + k_{\beta}\beta $

近似化以后
<center>
$$
\begin{bmatrix}
  \dot \rho\\
  \dot \alpha\\
  \dot \beta
\end{bmatrix}=
\begin{bmatrix}
  -k_{\rho} & 0 & 0\\
  0 & -(k_{\alpha}-k_{\rho}) & k_{\beta}\\
  0 & -k_{\rho} & 0
\end{bmatrix}
\begin{bmatrix}
  \rho \\
  \alpha \\
  \beta
\end{bmatrix}
$$
</center>

求出特征多项式$(\lambda+k_{\rho})[\lambda^2+\lambda(k_{\alpha}-k_{\rho})-k_{\rho}k_{\beta}]$
从而得到收敛条件
$k_{\rho}\ge 0, k_{\beta}\le 0, k_{\alpha}-k_{\rho}\ge 0$

实际上，静态的参数容易使机器人陷入局部极值，我们做了动态调参。当$\rho$比较大时，我们希望$\alpha$影响更大，反之希望$\beta$影响更大，同时希望不出现突变，因此设定了一个阈值$th$，将控制器改为
$$
\omega=
\begin{cases}
 (\frac{\rho}{th})^2 k_{\alpha}\alpha + k_{\beta}\beta & \text{rho < th}\\
 k_{\alpha}\alpha + (\frac{th}{\rho})^2 k_{\beta}\beta & \text{else}
\end{cases}$$
