---
tags:
  - modern-robotics
  - twist
  - screw
---

# Twist、Screw Axis 与 Pitch

## 1. Twist 是什么

Twist 是刚体瞬时运动的 6 维表示：

\[
V=
\begin{bmatrix}
\omega \\
v
\end{bmatrix}
\]

它描述的是某一时刻刚体“正在怎么动”。

## 2. Screw Axis 是什么

当 `V` 被单位化并解释为一条几何轴时，我们把它记成：

\[
S=
\begin{bmatrix}
\omega \\
v
\end{bmatrix}
\]

它表示一个 screw motion，可以理解成：

- 绕某条空间直线旋转
- 同时沿同一条直线平移

这条空间直线就叫 screw axis。

## 3. Pitch 是什么

Pitch `h` 定义为：

\[
h = \frac{\omega^T v}{\omega^T \omega}
\]

若 `||ω|| = 1`，则

\[
h = \omega^T v
\]

它表示“每转 `1 rad`，沿轴前进多少长度”。

## 4. 三种典型情况

### 4.1 纯旋转

\[
h = 0,\qquad v = -\omega \times q
\]

表示绕一条经过 `q`、方向为 `ω` 的轴纯转动。

### 4.2 真正的螺旋运动

\[
h \neq 0,\qquad v = -\omega \times q + h\omega
\]

表示绕轴转的同时，沿轴前进。

### 4.3 纯平移

\[
\omega = 0,\qquad ||v|| = 1
\]

这时没有旋转轴，只有平移方向。

## 5. 最容易混淆的点

> [!warning]
> 看到 `ω` 和 `v` 不共线，不代表它就不是 screw motion。  
> 因为 `v` 中还藏着“轴不经过原点”的偏置信息。

> [!warning]
> 看到“绕 `z` 轴转、沿 `y` 方向有速度”，不一定真的是“沿 `y` 平移”。  
> 它可能只是某条偏置轴旋转时，在原点参考下表现出来的线速度项。

## 6. 一句话记忆

> [!important]
> `ω` 告诉你轴的方向，`v` 告诉你轴在哪里、有没有沿轴推进。

## 相关页面

- [[03-Twist与Screw/公式推导：v = -ω × q + hω]]
- [[05-例题/例题：偏置轴纯转动]]
- [[05-例题/例题：真正的拧螺丝运动]]
