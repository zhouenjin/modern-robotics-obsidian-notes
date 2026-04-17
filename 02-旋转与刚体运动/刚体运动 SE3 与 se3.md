---
tags:
  - modern-robotics
  - se3
  - rigid-body-motion
---

# 刚体运动 SE3 与 se3

## 1. `SE(3)` 是什么

`SE(3)` 表示三维空间中的刚体位姿变换，通常写成齐次变换矩阵：

\[
T =
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
\]

其中：

- `R ∈ SO(3)` 表示姿态
- `p ∈ R^3` 表示位置

## 2. `se(3)` 是什么

`se(3)` 是 `SE(3)` 对应的李代数。它的元素通常写成：

\[
[V] =
\begin{bmatrix}
[\omega] & v \\
0 & 0
\end{bmatrix}
\]

其中：

- `ω` 是角速度部分
- `v` 是线速度部分

## 3. Twist 的形式

一个 twist 写成：

\[
V=
\begin{bmatrix}
\omega \\
v
\end{bmatrix}
\in \mathbb{R}^6
\]

如果它被单位化并解释为一条几何轴，通常记为：

\[
S=
\begin{bmatrix}
S_\omega \\
S_v
\end{bmatrix}
\]

满足：

- 若有转动，则 `||S_ω|| = 1`
- 若纯平移，则 `S_ω = 0` 且 `||S_v|| = 1`

## 4. `SE(3)` 和 `SO(3)` 的关系

你可以把 `SE(3)` 看成 `SO(3)` 的增强版：

- `SO(3)` 只管“怎么转”
- `SE(3)` 同时管“怎么转 + 怎么平移”

## 5. 图里的右半边到底在讲什么

图中“Rigid-Body Motions”这一栏，实际上在讲下面这条链：

\[
S\theta \to [S]\theta \in se(3) \to e^{[S]\theta} \in SE(3)
\]

也就是：

1. 先用螺旋轴和运动参数写出指数坐标 `Sθ`
2. 再写成 `se(3)` 中的矩阵 `[S]θ`
3. 最后做指数映射，得到真实的位姿变换 `T`

## 6. 关键区别

> [!important]
> `ω` 只描述转的方向。
> `v` 不是单纯的“平移方向”，它还包含“这根轴离原点多远”的信息。

## 相关页面

- [[03-Twist与Screw/Twist、Screw Axis 与 Pitch]]
- [[04-指数映射与位姿/指数映射：从 so3 到 SO3，从 se3 到 SE3]]
