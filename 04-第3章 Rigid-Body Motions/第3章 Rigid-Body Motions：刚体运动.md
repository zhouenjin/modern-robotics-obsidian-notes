---
tags:
  - modern-robotics
  - chapter-3
  - rigid-body-motion
---

# 第3章 Rigid-Body Motions：刚体运动

## 1. 本章目标

第 2 章回答了“机器人处在什么 configuration 上”，第 3 章进一步回答：

- 一个刚体的姿态如何表示？
- 一个刚体的位姿如何表示？
- 一个刚体的角速度、线速度如何统一表示？
- 一个刚体受到的力和力矩如何统一表示？

这章是整门课最核心的数学基础之一。

## 2. 官方小节结构

- Introduction to Rigid-Body Motions
- `3.2.1` Rotation Matrices
- `3.2.2` Angular Velocities
- `3.2.3` Exponential Coordinates of Rotation
- `3.3.1` Homogeneous Transformation Matrices
- `3.3.2` Twists
- `3.3.3` Exponential Coordinates of Rigid-Body Motion
- `3.4` Wrenches

## 3. 旋转的表示：从直觉到矩阵

### 3.1 右手定则和坐标系

课程一开始强调：

- 旋转方向服从右手定则；
- 参考系是右手系；
- 区分 space frame 和 body frame。

这是后面所有空间/本体表达的起点。

### 3.2 Rotation matrix 与 $SO(3)$

旋转矩阵满足：

$$
R^T R = I, \qquad \det(R) = 1
$$

所有这样的矩阵构成特殊正交群：

$$
SO(3)
$$

你可以把 $R$ 看成：

- 一个姿态的表示；
- 一个向量换参考系的变换；
- 一个向量或一个坐标系的旋转算子。

### 3.3 为什么课程喜欢矩阵表示

因为矩阵表示虽然不是“最少参数”，但它有几个巨大优点：

- 几何意义清楚；
- 组合自然，直接可乘；
- 与后续 $SE(3)$、Adjoint、PoE 全部兼容。

### 3.4 例子：绕 $z$ 轴旋转 $90^\circ$ 的旋转矩阵

令单位轴为：

$$
\hat{\omega} =
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
$$

令旋转角为：

$$
\theta = \frac{\pi}{2}
$$

则对应的反对称矩阵是：

$$
[\hat{\omega}] =
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
$$

使用 Rodrigues 公式：

$$
R = I + \sin\theta[\hat{\omega}] + (1-\cos\theta)[\hat{\omega}]^2
$$

先代入：

$$
\sin\frac{\pi}{2} = 1, \qquad \cos\frac{\pi}{2} = 0
$$

因此：

$$
R = I + [\hat{\omega}] + [\hat{\omega}]^2
$$

再算：

$$
[\hat{\omega}]^2 =
\begin{bmatrix}
-1 & 0 & 0 \\
0 & -1 & 0 \\
0 & 0 & 0
\end{bmatrix}
$$

所以：

$$
R =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
+
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
+
\begin{bmatrix}
-1 & 0 & 0 \\
0 & -1 & 0 \\
0 & 0 & 0
\end{bmatrix}
=
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

这个计算告诉你：

- $x$ 轴方向被转到了 $y$ 轴方向；
- 旋转矩阵不是抽象符号，而是能直接算出来的姿态对象。

## 4. 角速度与 so(3)

### 4.1 角速度向量

角速度写成：

$$
\omega \in \mathbb{R}^3
$$

它的方向是旋转轴方向，模长是瞬时转速。

### 4.2 hat 映射

把向量写成反对称矩阵：

$$
[\omega] =
\begin{bmatrix}
0 & -\omega_3 & \omega_2 \\
\omega_3 & 0 & -\omega_1 \\
-\omega_2 & \omega_1 & 0
\end{bmatrix}
$$

这些矩阵构成李代数：

$$
so(3)
$$

### 4.3 为什么要从向量转成矩阵

因为一旦写成 $[\omega]$，就能自然进入矩阵指数：

$$
R = e^{[\hat{\omega}] \theta}
$$

这为旋转和刚体运动提供了统一的“生成”视角。

### 4.4 例子：角速度如何产生瞬时线速度

设刚体绕 $z$ 轴以角速度

$$
\omega =
\begin{bmatrix}
0 \\
0 \\
2
\end{bmatrix}
$$

转动，考察刚体上一点：

$$
p =
\begin{bmatrix}
1 \\
0 \\
0
\end{bmatrix}
$$

该点瞬时线速度由叉乘给出：

$$
\dot{p} = \omega \times p
$$

代入计算：

$$
\omega \times p =
\begin{bmatrix}
0 \\
0 \\
2
\end{bmatrix}
\times
\begin{bmatrix}
1 \\
0 \\
0
\end{bmatrix}
=
\begin{bmatrix}
0 \\
2 \\
0
\end{bmatrix}
$$

这说明：

- 点 $p$ 沿正 $y$ 方向运动；
- 角速度向量描述的是“整个刚体怎么转”；
- 点的线速度是由转动和点相对轴的位置共同决定的。

## 5. 旋转的指数坐标

### 5.1 轴角表示

任意旋转都可以视为绕某个单位轴 $\hat{\omega}$ 旋转角度 $\theta$，于是定义指数坐标：

$$
\omega = \hat{\omega}\theta
$$

这是一种 3 参数表示法。

### 5.2 Rodrigues 公式

矩阵指数可以写成：

$$
R = e^{[\hat{\omega}] \theta}
= I + \sin\theta [\hat{\omega}] + (1 - \cos\theta)[\hat{\omega}]^2
$$

这条公式在课程里的意义不仅是“能算”，更重要的是：

- 旋转不再只是一个几何图形；
- 它成为由李代数元素生成的群元素。

### 5.3 例子：把轴角坐标恢复成旋转矩阵

若指数坐标为：

$$
\omega =
\begin{bmatrix}
0 \\
0 \\
\frac{\pi}{3}
\end{bmatrix}
$$

则它表示：

- 单位轴 $\hat{\omega} = (0,0,1)^T$；
- 角度 $\theta = \frac{\pi}{3}$。

利用 Rodrigues 公式：

$$
R =
\begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

代入：

$$
\cos\frac{\pi}{3} = \frac{1}{2}, \qquad
\sin\frac{\pi}{3} = \frac{\sqrt{3}}{2}
$$

得到：

$$
R =
\begin{bmatrix}
\frac{1}{2} & -\frac{\sqrt{3}}{2} & 0 \\
\frac{\sqrt{3}}{2} & \frac{1}{2} & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

这展示了“指数坐标”和“旋转矩阵”之间的来回转换。

## 6. 从姿态到位姿：$SE(3)$

### 6.1 齐次变换矩阵

刚体位姿写成：

$$
T =
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
$$

其中：

- $R \in SO(3)$ 表示姿态；
- $p \in \mathbb{R}^3$ 表示位置。

全体这样的矩阵构成：

$$
SE(3)
$$

### 6.2 课程对齐次变换的三个常见用途

这部分是课程里特别强调的：

1. 表示一个刚体配置；
2. 改变一个向量或一个坐标系的参考表达；
3. 对一个向量或一个坐标系施加位姿变换。

这三个“用法”如果在脑中不分清，后面常会把左乘、右乘、换坐标、真位移混在一起。

### 6.3 例子：一个平面位姿如何写成 $SE(3)$ 矩阵

设一个刚体在平面内：

- 绕 $z$ 轴旋转 $90^\circ$；
- 同时平移到点 $(2,1,0)$。

根据前面的旋转结果：

$$
R =
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

平移向量为：

$$
p =
\begin{bmatrix}
2 \\
1 \\
0
\end{bmatrix}
$$

所以：

$$
T =
\begin{bmatrix}
0 & -1 & 0 & 2 \\
1 & 0 & 0 & 1 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

如果作用在齐次点

$$
\bar{x} =
\begin{bmatrix}
1 \\
0 \\
0 \\
1
\end{bmatrix}
$$

上，则：

$$
T\bar{x} =
\begin{bmatrix}
2 \\
2 \\
0 \\
1
\end{bmatrix}
$$

这说明原来位于局部 $x$ 轴上的点，被先转再移，落到了世界坐标中的 $(2,2,0)$。

## 7. Twist：统一描述刚体速度

### 7.1 为什么要引入 twist

如果只用：

- 角速度 $\omega$；
- 某点线速度 $v$

来描述刚体运动，表达不够统一。课程因此引入 6 维向量：

$$
V =
\begin{bmatrix}
\omega \\
v
\end{bmatrix}
$$

这就是 twist，也常被称为 spatial velocity。

### 7.2 Screw axis 的思想

当 $\omega \neq 0$ 时，一个刚体的瞬时运动可以理解为：

- 绕某条空间直线转；
- 同时沿该直线移动。

这条直线就是 screw axis。

若已知：

- 轴上一点 $q$；
- 单位方向 $\hat{s}$；
- pitch $h$；

则 screw axis 可写为：

$$
S =
\begin{bmatrix}
\hat{s} \\
-\hat{s} \times q + h\hat{s}
\end{bmatrix}
$$

> [!important]
> 这条式子非常关键，因为它把“轴在哪里”和“沿轴有没有推进”同时编码进了下半部分。

### 7.3 se(3) 表示

twist 的矩阵形式写成：

$$
[V] =
\begin{bmatrix}
[\omega] & v \\
0 & 0
\end{bmatrix}
$$

这些矩阵构成：

$$
se(3)
$$

### 7.4 例子：从轴上一点和 pitch 算出 screw axis

设一条 screw axis 满足：

- 方向为

$$
\hat{s} =
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
$$

- 轴上一点为

$$
q =
\begin{bmatrix}
1 \\
0 \\
0
\end{bmatrix}
$$

- pitch 为

$$
h = 2
$$

根据公式：

$$
S =
\begin{bmatrix}
\hat{s} \\
-\hat{s} \times q + h\hat{s}
\end{bmatrix}
$$

先算叉乘：

$$
\hat{s} \times q =
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
\times
\begin{bmatrix}
1 \\
0 \\
0
\end{bmatrix}
=
\begin{bmatrix}
0 \\
1 \\
0
\end{bmatrix}
$$

因此：

$$
-\hat{s} \times q =
\begin{bmatrix}
0 \\
-1 \\
0
\end{bmatrix}
$$

再算：

$$
h\hat{s} =
2
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
=
\begin{bmatrix}
0 \\
0 \\
2
\end{bmatrix}
$$

于是：

$$
-\hat{s} \times q + h\hat{s} =
\begin{bmatrix}
0 \\
-1 \\
2
\end{bmatrix}
$$

最终：

$$
S =
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
-1 \\
2
\end{bmatrix}
$$

这个例子把三件事放在了一起：

- 轴的方向来自上半部分；
- 轴的位置进入下半部分的叉乘项；
- pitch 决定沿轴推进的分量。

## 8. 刚体运动的指数坐标

### 8.1 从 twist 到位姿

和旋转的情况一样，刚体位姿也可以通过指数映射得到：

$$
T = e^{[S]\theta}
$$

其中 $S\theta$ 叫刚体运动的指数坐标。

### 8.2 这一步的真正意义

它把 Chapter 3 的所有对象连成一条主线：

```mermaid
flowchart LR
    A["旋转轴与角度"] --> B["so(3)"]
    B --> C["SO(3)"]
    C --> D["位姿矩阵 T"]
    A --> E["screw axis S"]
    E --> F["se(3)"]
    F --> D
```

也就是说：

- 旋转是刚体运动的一个特例；
- $SO(3)$ 到 $SE(3)$ 的扩展，核心就是把旋转推广成 screw motion。

### 8.3 例子：由 screw axis 指数映射得到刚体位姿

继续使用上一节得到的：

$$
S =
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
-1 \\
2
\end{bmatrix}
$$

令：

$$
\theta = \frac{\pi}{2}
$$

则旋转部分是绕 $z$ 轴转 $90^\circ$：

$$
R =
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

平移部分使用：

$$
p =
\left(
I\theta + (1-\cos\theta)[\omega] + (\theta-\sin\theta)[\omega]^2
\right)v
$$

其中

$$
\omega =
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix},
\qquad
v =
\begin{bmatrix}
0 \\
-1 \\
2
\end{bmatrix}
$$

因为：

$$
\sin\frac{\pi}{2}=1,\qquad \cos\frac{\pi}{2}=0
$$

所以：

$$
p =
\left(
I\frac{\pi}{2} + [\omega] + \left(\frac{\pi}{2}-1\right)[\omega]^2
\right)v
$$

逐项计算后可得：

$$
p =
\begin{bmatrix}
1 \\
-1 \\
\pi
\end{bmatrix}
$$

因此：

$$
T =
\begin{bmatrix}
0 & -1 & 0 & 1 \\
1 & 0 & 0 & -1 \\
0 & 0 & 1 & \pi \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

这就是一个完整的“拧螺丝”结果：

- 姿态转了 $90^\circ$；
- 同时沿螺旋轴方向前进了 $\pi$。

## 9. Wrench：统一描述力与力矩

第 `3.4` 节把力学对象也写成 6 维向量。

一般写作：

$$
F =
\begin{bmatrix}
\tau \\
f
\end{bmatrix}
$$

其中：

- $f$ 是力；
- $\tau$ 是力矩。

wrench 和 twist 的对应关系非常重要，因为后面速度运动学、静力学、动力学都会用这个 6 维统一表达。

### 9.1 例子：把一个力和一个力矩合成 wrench

设末端受到：

- 力

$$
f =
\begin{bmatrix}
3 \\
0 \\
0
\end{bmatrix}
$$

- 力矩

$$
\tau =
\begin{bmatrix}
0 \\
0 \\
2
\end{bmatrix}
$$

则对应 wrench 为：

$$
F =
\begin{bmatrix}
\tau \\
f
\end{bmatrix}
=
\begin{bmatrix}
0 \\
0 \\
2 \\
3 \\
0 \\
0
\end{bmatrix}
$$

它表示：

- 下半部分是“推/拉”的线力；
- 上半部分是“扭”的力矩；
- 课程把这两者统一打包，是为了和 twist 形成速度-力的配对框架。

## 10. 本章最重要的几个结构对应

| 几何对象 | 李代数表示 | 李群表示 |
| --- | --- | --- |
| 旋转 | $so(3)$ | $SO(3)$ |
| 刚体运动 | $se(3)$ | $SE(3)$ |

以及：

| 物理量 | 统一向量形式 |
| --- | --- |
| 刚体速度 | twist |
| 刚体受力 | wrench |

## 11. 为什么第 3 章是全书核心

> [!important]
> 第 3 章不是单独的一章“表示法介绍”，而是后面所有机器人运动学和动力学公式的语言基础。

没有这一章，后面 Chapter 4 的 PoE 就只是公式；  
有了这一章，PoE 才是“由 screw axis 生成末端位姿”的自然结论。

## 12. 听课提醒

### 12.1 不要只背群名

你需要真的分清：

- $SO(3)$ 是姿态；
- $SE(3)$ 是位姿；
- twist 是速度；
- wrench 是力。

### 12.2 不要把 $v$ 理解成“单纯平移方向”

在 screw axis 里，下半部分并不只是“沿哪个方向平移”，它还包含轴相对原点的位置关系。

### 12.3 不要把矩阵乘法当成纯代数技巧

这里的矩阵乘法每一步都有几何意义。听课时一定要问自己：

- 我现在是在换坐标表达，还是在做真实位移？
- 这个对象属于 $SO(3)$、$SE(3)$、$so(3)$ 还是 $se(3)$？

## 13. 与前后章节的关系

- 它建立在 [[03-第2章 Configuration Space/第2章 Configuration Space：构型空间]] 对 configuration 的讨论之上。
- 它直接服务于 [[05-第4章 Forward Kinematics/第4章 Forward Kinematics：正运动学]] 中的 PoE 公式。

## 14. 本章串联例子总结

> [!example]
> 这一章的例子实际上是一条连续链：
> - 先从绕 $z$ 轴的旋转矩阵开始；
> - 再看角速度如何给点产生线速度；
> - 接着把姿态升级成位姿矩阵；
> - 再把单关节写成 screw axis；
> - 最后用指数映射得到完整位姿。

如果这条链你能自己重算一遍，Chapter 3 的主干就抓住了。
