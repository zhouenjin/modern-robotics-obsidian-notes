---
tags:
  - modern-robotics
  - exponential-map
  - se3
---

# 指数映射：从 so3 到 SO3，从 se3 到 SE3

## 1. 左边：从 `so(3)` 到 `SO(3)`

若旋转轴为单位向量 `\hat{ω}`，角度为 `θ`，则：

\[
R = e^{[\hat{\omega}]\theta}
\]

使用 Rodrigues 公式：

\[
R = I + \sin\theta[\hat{\omega}] + (1-\cos\theta)[\hat{\omega}]^2
\]

## 2. 右边：从 `se(3)` 到 `SE(3)`

若 screw axis 为

\[
S=
\begin{bmatrix}
\omega\\
v
\end{bmatrix}
\]

则

\[
[S] =
\begin{bmatrix}
[\omega] & v \\
0 & 0
\end{bmatrix}
\]

指数映射为：

\[
T = e^{[S]\theta}
=
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
\]

其中

\[
R = e^{[\omega]\theta}
\]

\[
p =
\left(
I\theta + (1-\cos\theta)[\omega] + (\theta-\sin\theta)[\omega]^2
\right)v
\]

## 3. 纯平移的特殊情形

若 `ω = 0`，则是纯平移：

\[
T=
\begin{bmatrix}
I & v\theta \\
0 & 1
\end{bmatrix}
\]

其中 `v` 是单位平移方向，`θ` 是平移距离。

## 4. 图里为什么要并排写

这是因为右边整套公式是左边的自然推广：

- 左边只处理旋转
- 右边处理旋转加平移

可以直接对应着记：

| 旋转 | 刚体运动 |
| --- | --- |
| `ωθ` | `Sθ` |
| `[ω]θ` | `[S]θ` |
| `e^[ [ω]θ ] = R` | `e^[ [S]θ ] = T` |

## 5. 做题模板

1. 写出 `ω`、`v`、`S`
2. 先算 `R = e^[ [ω]θ ]`
3. 再算 `p`
4. 拼成 `T = [R p; 0 1]`

## 6. 最常见易错点

> [!warning]
> `p` 一般不等于 `vθ`。  
> 只有纯平移时才有 `p = vθ`。  
> 一旦存在旋转，`p` 必须通过完整公式计算。

## 相关页面

- [[02-旋转与刚体运动/旋转群 SO3 与 so3]]
- [[05-例题/例题：真正的拧螺丝运动]]
