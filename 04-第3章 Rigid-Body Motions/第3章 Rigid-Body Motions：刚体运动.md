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
