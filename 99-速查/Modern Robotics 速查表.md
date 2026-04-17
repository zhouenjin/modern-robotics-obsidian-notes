---
tags:
  - modern-robotics
  - cheatsheet
---

# Modern Robotics 速查表

## 一、最常用对象

| 名称 | 形式 | 含义 |
| --- | --- | --- |
| 旋转轴 | `ω` | 旋转方向 |
| 轴角 | `ωθ` | 旋转的指数坐标 |
| twist | `V=(ω,v)` | 刚体瞬时运动 |
| screw axis | `S=(ω,v)` | 单位化 twist |
| 位姿 | `T=[R p; 0 1]` | 刚体位置与姿态 |

## 二、最常用公式

### 旋转

\[
[\omega] =
\begin{bmatrix}
0 & -\omega_3 & \omega_2 \\
\omega_3 & 0 & -\omega_1 \\
-\omega_2 & \omega_1 & 0
\end{bmatrix}
\]

\[
R = e^{[\hat{\omega}]\theta}
= I + \sin\theta[\hat{\omega}] + (1-\cos\theta)[\hat{\omega}]^2
\]

### 刚体运动

\[
[S] =
\begin{bmatrix}
[\omega] & v \\
0 & 0
\end{bmatrix}
\]

\[
v = -\omega \times q + h\omega
\]

\[
h = \frac{\omega^T v}{\omega^T \omega}
\]

\[
T = e^{[S]\theta}
=
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
\]

\[
p=
\left(
I\theta + (1-\cos\theta)[\omega] + (\theta-\sin\theta)[\omega]^2
\right)v
\]

## 三、三类题怎么秒判

| 现象 | 判断 |
| --- | --- |
| `ω = 0` | 纯平移 |
| `ω ≠ 0` 且 `h = 0` | 偏置轴纯转动 |
| `ω ≠ 0` 且 `h ≠ 0` | 真正的 screw motion |

## 四、标准做题流程

1. 写出 `ω`、`v` 或 `q`、`h`
2. 用 `v = -ω × q + hω` 建立几何联系
3. 求 `h`
4. 必要时求轴上一点 `q`
5. 再做指数映射求 `T`

## 五、最易错点

> [!warning]
> `p` 不等于 `vθ`，除非是纯平移。

> [!warning]
> `v` 不只是平移方向，它还编码了螺旋轴相对原点的位置。

## 六、串联页面

- [[00-首页]]
- [[03-Twist与Screw/公式推导：v = -ω × q + hω]]
- [[05-例题/例题：真正的拧螺丝运动]]
