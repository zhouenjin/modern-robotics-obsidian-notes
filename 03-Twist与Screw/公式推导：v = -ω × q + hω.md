---
tags:
  - modern-robotics
  - derivation
  - screw
---

# 公式推导：v = -ω × q + hω

## 1. 目标

要理解的公式是：

\[
S=
\begin{bmatrix}
\omega \\
v
\end{bmatrix},
\qquad
v = -\omega \times q + h\omega
\]

其中：

- `ω`：螺旋轴方向
- `q`：螺旋轴上任意一点
- `h`：pitch

## 2. 从刚体速度场出发

刚体上任一点 `x` 的瞬时速度满足：

\[
\dot{x} = \omega \times x + v
\]

这句话的意思是：

- `ω × x` 是旋转带来的线速度
- `v` 是 twist 中的偏置项

## 3. 对轴上的点施加几何条件

若点 `q` 位于 screw axis 上，那么它不会绕开轴乱跑，它的速度只能沿轴方向：

\[
\dot{q} = h\omega
\]

这里 `hω` 表示：

- 方向沿轴
- 大小等于“每 `1 rad` 前进 `h`”

另一方面，由速度场公式：

\[
\dot{q} = \omega \times q + v
\]

两式相等：

\[
\omega \times q + v = h\omega
\]

移项得到：

\[
v = -\omega \times q + h\omega
\]

这就是公式来源。

## 4. 公式的几何拆解

\[
v = -\omega \times q + h\omega
\]

可以拆成两部分：

- `-ω × q`：由于旋转轴不经过原点而产生的偏置项
- `hω`：沿着螺旋轴前进的部分

## 5. 为什么换轴上另一个点也没事

设轴上另一点为：

\[
q' = q + \lambda \omega
\]

则

\[
-\omega \times q' + h\omega
= -\omega \times (q + \lambda\omega) + h\omega
\]

\[
= -\omega \times q - \lambda(\omega \times \omega) + h\omega
\]

因为

\[
\omega \times \omega = 0
\]

所以仍然得到：

\[
v = -\omega \times q + h\omega
\]

这说明：`q` 取轴上的任何一点都可以。

## 6. 反过来怎么从 `S = (ω, v)` 看出 pitch

把 `v` 在 `ω` 方向上投影：

\[
h = \frac{\omega^T v}{\omega^T \omega}
\]

若 `||ω|| = 1`，则：

\[
h = \omega^T v
\]

这表示 `v` 中平行于轴方向的那一部分，就是沿轴推进的量。

## 7. 小例子

设

\[
\omega =
\begin{bmatrix}
0\\0\\1
\end{bmatrix},
\quad
q =
\begin{bmatrix}
1\\0\\0
\end{bmatrix},
\quad
h=2
\]

则

\[
\omega \times q =
\begin{bmatrix}
0\\1\\0
\end{bmatrix}
\]

因此

\[
-\omega \times q =
\begin{bmatrix}
0\\-1\\0
\end{bmatrix}
\]

以及

\[
h\omega =
\begin{bmatrix}
0\\0\\2
\end{bmatrix}
\]

相加得

\[
v=
\begin{bmatrix}
0\\-1\\2
\end{bmatrix}
\]

所以

\[
S=
\begin{bmatrix}
0\\0\\1\\0\\-1\\2
\end{bmatrix}
\]

## 8. 一句话总结

> [!important]
> `v = -ω × q + hω` 的本质是：  
> twist 的线速度部分 = 轴偏置带来的项 + 沿轴推进带来的项。

## 相关页面

- [[03-Twist与Screw/Twist、Screw Axis 与 Pitch]]
- [[05-例题/例题：真正的拧螺丝运动]]
