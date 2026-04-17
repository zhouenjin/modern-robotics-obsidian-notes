---
tags:
  - modern-robotics
  - so3
  - rotation
---

# 旋转群 SO3 与 so3

## 1. `SO(3)` 是什么

`SO(3)` 是所有三维旋转矩阵的集合。矩阵 `R` 属于 `SO(3)` 当且仅当：

\[
R^T R = I,\qquad \det(R)=1
\]

直觉上，`SO(3)` 里的元素表示“物体姿态怎么转过去了”。

## 2. `so(3)` 是什么

`so(3)` 是 `SO(3)` 对应的李代数。它的元素是 `3×3` 反对称矩阵，通常由向量 `ω` 通过 hat 映射得到：

\[
[\omega] =
\begin{bmatrix}
0 & -\omega_3 & \omega_2 \\
\omega_3 & 0 & -\omega_1 \\
-\omega_2 & \omega_1 & 0
\end{bmatrix}
\]

其中

\[
\omega =
\begin{bmatrix}
\omega_1 \\
\omega_2 \\
\omega_3
\end{bmatrix}
\]

## 3. 轴角表示

若旋转轴单位向量为 `\hat{ω}`，旋转角度为 `θ`，则指数坐标是：

\[
\hat{\omega}\theta \in \mathbb{R}^3
\]

矩阵形式则是：

\[
[\hat{\omega}]\theta \in so(3)
\]

## 4. 从 `so(3)` 到 `SO(3)`

通过指数映射得到真正的旋转矩阵：

\[
R = e^{[\hat{\omega}]\theta}
\]

Rodrigues 公式：

\[
R = I + \sin\theta[\hat{\omega}] + (1-\cos\theta)[\hat{\omega}]^2
\]

## 5. 典型例子

绕 `z` 轴旋转 `90°`：

\[
\hat{\omega} =
\begin{bmatrix}
0\\0\\1
\end{bmatrix},
\qquad
\theta = \frac{\pi}{2}
\]

\[
[\hat{\omega}] =
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
\]

所以

\[
R=
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
\]

表示 `x` 轴方向被转到 `y` 轴方向。

## 6. 你要牢牢记住的点

> [!important]
> `SO(3)` 里放的是“结果姿态”。
> `so(3)` 里放的是“生成这个姿态的无穷小旋转方向和角度信息”。

## 相关页面

- [[02-旋转与刚体运动/刚体运动 SE3 与 se3]]
- [[04-指数映射与位姿/指数映射：从 so3 到 SO3，从 se3 到 SE3]]
