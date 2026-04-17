---
tags:
  - modern-robotics
  - chapter-5
  - jacobian
  - statics
---

# 第5章 Velocity Kinematics and Statics：速度运动学与静力学

## 1. 本章目标

第 4 章已经解决了正运动学：

$$
\theta \longmapsto T(\theta)
$$

第 5 章继续问两个更直接和控制相关的问题：

1. 如果关节速度 $\dot{\theta}$ 已知，末端 twist 是多少？
2. 如果末端受到 wrench，关节需要承担多大的力和力矩？

本章的核心对象就是 **Jacobian**。  
它把第 2 章的关节空间、第 3 章的 twist 和 wrench、第 4 章的正运动学全部联通起来。

## 2. 官方小节结构

- `5.1.1` Space Jacobian
- `5.1.2` Body Jacobian
- `5.2` Statics of Open Chains
- `5.3` Singularities
- `5.4` Manipulability

## 3. 贯穿例子设定：继续使用平面 2R 机械臂

为了让第 4 章和第 5 章无缝连接，我们继续使用同一台平面 2R 机械臂：

- 连杆长度：$l_1 = 1,\; l_2 = 1$
- 关节变量：$\theta_1,\theta_2$
- 两个关节都是绕 $z$ 轴的转动关节
- 零位形时，两根连杆都沿 $x$ 轴正方向伸直

第 4 章已经得到：

$$
M =
\begin{bmatrix}
1 & 0 & 0 & 2 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

以及空间 screw axes：

$$
S_1 =
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
0 \\
0
\end{bmatrix},
\qquad
S_2 =
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
-1 \\
0
\end{bmatrix}
$$

这组数据会贯穿整章。

## 4. Jacobian 的本质

Jacobian 不是“为了求导而求导”的对象，它在机器人学里的地位更像一个接口矩阵：

$$
V = J(\theta)\dot{\theta}
$$

它把：

- 关节速度 $\dot{\theta}$
- 末端 twist $V$

联系在一起。

同样，它还满足静力关系：

$$
\tau = J^T(\theta) F
$$

它又把：

- 末端 wrench $F$
- 关节力矩 $\tau$

联系在一起。

> [!important]
> 所以 Jacobian 是“速度接口”和“力接口”的统一核心。

## 5. Space Jacobian

### 5.1 定义

space Jacobian 把末端 twist 在 space frame 中表达出来：

$$
V_s = J_s(\theta)\dot{\theta}
$$

其中：

- $V_s$ 是末端 twist 的空间表达；
- $J_s(\theta)$ 是 space Jacobian；
- $\dot{\theta}$ 是关节速度向量。

### 5.2 结构公式

对串联开链机构，space Jacobian 的列向量可以写成：

$$
J_s(\theta) =
\begin{bmatrix}
S_1 &
\text{Ad}_{e^{[S_1]\theta_1}} S_2 &
\cdots &
\text{Ad}_{e^{[S_1]\theta_1}\cdots e^{[S_{n-1}]\theta_{n-1}}} S_n
\end{bmatrix}
$$

几何含义是：

- 第一列永远是第一个关节的空间 screw axis；
- 第 $i$ 列是第 $i$ 个关节 axis 在前面关节运动之后、仍然用空间坐标系表达出来的结果。

### 5.3 例子：2R 机械臂的 space Jacobian

对 2R 机械臂：

$$
J_s(\theta) =
\begin{bmatrix}
S_1 & \text{Ad}_{e^{[S_1]\theta_1}} S_2
\end{bmatrix}
$$

第一列直接是：

$$
S_1 =
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
0 \\
0
\end{bmatrix}
$$

第二列要把 $S_2$ 经过第一个关节的旋转重新表达。

因为第一个关节只是绕原点绕 $z$ 轴旋转，所以：

- 上半部分角速度方向仍然是 $z$ 轴；
- 下半部分线速度项会随 $\theta_1$ 一起转。

原来：

$$
v_2 =
\begin{bmatrix}
0 \\
-1 \\
0
\end{bmatrix}
$$

旋转矩阵是：

$$
R_z(\theta_1) =
\begin{bmatrix}
\cos\theta_1 & -\sin\theta_1 & 0 \\
\sin\theta_1 & \cos\theta_1 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

所以新的线速度项为：

$$
R_z(\theta_1)v_2
=
\begin{bmatrix}
\cos\theta_1 & -\sin\theta_1 & 0 \\
\sin\theta_1 & \cos\theta_1 & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
0 \\
-1 \\
0
\end{bmatrix}
=
\begin{bmatrix}
\sin\theta_1 \\
-\cos\theta_1 \\
0
\end{bmatrix}
$$

因此：

$$
J_s(\theta) =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
1 & 1 \\
0 & \sin\theta_1 \\
0 & -\cos\theta_1 \\
0 & 0
\end{bmatrix}
$$

这个矩阵的解释是：

- 第一列是肩关节对末端 twist 的贡献；
- 第二列是肘关节 axis 在空间坐标系里的当前表达。

### 5.4 用具体数值代入一次

取：

$$
\theta_1 = \frac{\pi}{6}
$$

则：

$$
\sin\theta_1 = \frac{1}{2}, \qquad \cos\theta_1 = \frac{\sqrt{3}}{2}
$$

代入得到：

$$
J_s\left(\frac{\pi}{6}, \theta_2\right) =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
1 & 1 \\
0 & \frac{1}{2} \\
0 & -\frac{\sqrt{3}}{2} \\
0 & 0
\end{bmatrix}
$$

如果再令关节速度：

$$
\dot{\theta} =
\begin{bmatrix}
2 \\
1
\end{bmatrix}
$$

则末端空间 twist 为：

$$
V_s = J_s \dot{\theta}
$$

逐行计算：

$$
V_s =
\begin{bmatrix}
0 \\
0 \\
1 \cdot 2 + 1 \cdot 1 \\
0 \cdot 2 + \frac{1}{2}\cdot 1 \\
0 \cdot 2 - \frac{\sqrt{3}}{2}\cdot 1 \\
0
\end{bmatrix}
=
\begin{bmatrix}
0 \\
0 \\
3 \\
\frac{1}{2} \\
-\frac{\sqrt{3}}{2} \\
0
\end{bmatrix}
$$

解释：

- 末端角速度是 $3$ rad/s；
- 同时末端在空间系中还有线速度分量 $\left(\frac{1}{2},-\frac{\sqrt{3}}{2},0\right)$。

## 6. Body Jacobian

### 6.1 定义

body Jacobian 把末端 twist 在 body frame 中表达出来：

$$
V_b = J_b(\theta)\dot{\theta}
$$

与 space Jacobian 的区别不在“是不是同一个运动”，而在于：

- 一个用空间坐标系表达；
- 一个用末端坐标系表达。

### 6.2 结构公式

对串联机械臂：

$$
J_b(\theta) =
\begin{bmatrix}
\text{Ad}_{e^{-[B_n]\theta_n}\cdots e^{-[B_2]\theta_2}} B_1 &
\cdots &
\text{Ad}_{e^{-[B_n]\theta_n}} B_{n-1} &
B_n
\end{bmatrix}
$$

### 6.3 2R 机械臂的 body screw axes

第 4 章已经得到零位形下的 body screw axes：

$$
B_1 =
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
2 \\
0
\end{bmatrix},
\qquad
B_2 =
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
1 \\
0
\end{bmatrix}
$$

### 6.4 例子：2R 机械臂的 body Jacobian

对 2R 机构：

$$
J_b(\theta) =
\begin{bmatrix}
\text{Ad}_{e^{-[B_2]\theta_2}}B_1 & B_2
\end{bmatrix}
$$

我们直接从几何上理解第一列。

在 body frame 中：

- 第 2 关节轴通过点 $(-1,0,0)$；
- 第 1 关节轴通过点 $(-2,0,0)$。

当第 2 关节转过角度 $\theta_2$ 时，第 1 关节轴相对 body frame 的位置会绕点 $(-1,0)$ 旋转。

原始相对向量是：

$$
\begin{bmatrix}
-1 \\
0
\end{bmatrix}
$$

转过 $-\theta_2$ 后变成：

$$
\begin{bmatrix}
-\cos\theta_2 \\
\sin\theta_2
\end{bmatrix}
$$

因此新的轴上一点为：

$$
q_1' =
\begin{bmatrix}
-1 \\
0
\end{bmatrix}
+
\begin{bmatrix}
-\cos\theta_2 \\
\sin\theta_2
\end{bmatrix}
=
\begin{bmatrix}
-1-\cos\theta_2 \\
\sin\theta_2
\end{bmatrix}
$$

对应纯转动 screw axis 的下半部分为：

$$
v_1' = -\hat{z}\times q_1'
=
\begin{bmatrix}
\sin\theta_2 \\
1+\cos\theta_2 \\
0
\end{bmatrix}
$$

所以：

$$
J_b(\theta) =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
1 & 1 \\
\sin\theta_2 & 0 \\
1+\cos\theta_2 & 1 \\
0 & 0
\end{bmatrix}
$$

### 6.5 用具体数值代入一次

取：

$$
\theta_2 = \frac{\pi}{3}
$$

则：

$$
\sin\theta_2 = \frac{\sqrt{3}}{2},\qquad
\cos\theta_2 = \frac{1}{2}
$$

所以：

$$
J_b =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
1 & 1 \\
\frac{\sqrt{3}}{2} & 0 \\
\frac{3}{2} & 1 \\
0 & 0
\end{bmatrix}
$$

若仍取：

$$
\dot{\theta} =
\begin{bmatrix}
2 \\
1
\end{bmatrix}
$$

则：

$$
V_b = J_b \dot{\theta}
=
\begin{bmatrix}
0 \\
0 \\
3 \\
\sqrt{3} \\
4 \\
0
\end{bmatrix}
$$

解释：

- 这是同一个末端运动；
- 只不过现在速度被表达在末端自身参考系里。

## 7. 用平面位置 Jacobian 再看一遍速度映射

虽然 Chapter 5 的正式语言是 twist Jacobian，但对平面 2R 机械臂，位置 Jacobian 仍然非常有助于理解。

末端位置满足：

$$
x = l_1\cos\theta_1 + l_2\cos(\theta_1+\theta_2)
$$

$$
y = l_1\sin\theta_1 + l_2\sin(\theta_1+\theta_2)
$$

对时间求导：

$$
\dot{x} =
\left(
-l_1\sin\theta_1 - l_2\sin(\theta_1+\theta_2)
\right)\dot{\theta}_1
- l_2\sin(\theta_1+\theta_2)\dot{\theta}_2
$$

$$
\dot{y} =
\left(
l_1\cos\theta_1 + l_2\cos(\theta_1+\theta_2)
\right)\dot{\theta}_1
+ l_2\cos(\theta_1+\theta_2)\dot{\theta}_2
$$

因此：

$$
\begin{bmatrix}
\dot{x} \\
\dot{y}
\end{bmatrix}
=
J_p(\theta)
\begin{bmatrix}
\dot{\theta}_1 \\
\dot{\theta}_2
\end{bmatrix}
$$

其中位置 Jacobian 为：

$$
J_p(\theta) =
\begin{bmatrix}
-l_1\sin\theta_1 - l_2\sin(\theta_1+\theta_2) &
-l_2\sin(\theta_1+\theta_2) \\
l_1\cos\theta_1 + l_2\cos(\theta_1+\theta_2) &
l_2\cos(\theta_1+\theta_2)
\end{bmatrix}
$$

对 $l_1=l_2=1$，若取：

$$
\theta_1=\frac{\pi}{6},\qquad \theta_2=\frac{\pi}{3}
$$

则：

$$
\theta_1+\theta_2=\frac{\pi}{2}
$$

于是：

$$
J_p =
\begin{bmatrix}
-\frac{3}{2} & -1 \\
\frac{\sqrt{3}}{2} & 0
\end{bmatrix}
$$

再取：

$$
\dot{\theta}=
\begin{bmatrix}
2 \\
1
\end{bmatrix}
$$

则：

$$
\begin{bmatrix}
\dot{x} \\
\dot{y}
\end{bmatrix}
=
\begin{bmatrix}
-\frac{3}{2} & -1 \\
\frac{\sqrt{3}}{2} & 0
\end{bmatrix}
\begin{bmatrix}
2 \\
1
\end{bmatrix}
=
\begin{bmatrix}
-4 \\
\sqrt{3}
\end{bmatrix}
$$

这个结果很好地告诉你：

- Jacobian 的每一列表示某个关节单独单位速度时对末端速度的贡献；
- Jacobian 乘以关节速度，就是这些贡献的线性叠加。

## 8. Statics of Open Chains

### 8.1 静力关系

Chapter 5 中最重要的静力公式是：

$$
\tau = J^T F
$$

其中：

- $F$ 是末端 wrench；
- $\tau$ 是关节力和关节力矩。

这条关系可以从功率守恒直觉理解：

$$
\dot{\theta}^T \tau = V^T F
$$

这里右边的

$$
V^T F
$$

就是 Chapter 3 里讲的瞬时机械功率：

$$
V^T F = \omega^T\tau + v^T f
$$

它在不同坐标系中必须保持不变。

而又有：

$$
V = J\dot{\theta}
$$

因此：

$$
\dot{\theta}^T \tau = \dot{\theta}^T J^T F
$$

对任意 $\dot{\theta}$ 成立，所以：

$$
\tau = J^T F
$$

### 8.2 例子：由末端力求关节力矩

继续使用 2R 机械臂的位置 Jacobian。  
在平面位置问题里，若只考虑末端平面力

$$
f =
\begin{bmatrix}
f_x \\
f_y
\end{bmatrix}
$$

则静力关系可写成：

$$
\tau = J_p^T f
$$

取前面同样的姿态：

$$
J_p =
\begin{bmatrix}
-\frac{3}{2} & -1 \\
\frac{\sqrt{3}}{2} & 0
\end{bmatrix}
$$

设末端受到一个向上的力：

$$
f =
\begin{bmatrix}
0 \\
10
\end{bmatrix}
$$

则：

$$
\tau = J_p^T f
=
\begin{bmatrix}
-\frac{3}{2} & \frac{\sqrt{3}}{2} \\
-1 & 0
\end{bmatrix}
\begin{bmatrix}
0 \\
10
\end{bmatrix}
=
\begin{bmatrix}
5\sqrt{3} \\
0
\end{bmatrix}
$$

也就是：

$$
\tau_1 = 5\sqrt{3},\qquad \tau_2 = 0
$$

概念解释：

- 在这个构型下，向上的末端力全部由肩关节提供力矩；
- 肘关节在这个特定方向的受力投影下不需要额外扭矩。

## 9. Singularities

### 9.1 什么是奇异位形

当 Jacobian 降秩时，机器人进入 singularity。  
几何上意味着：

- 某些方向上的末端速度无法通过关节速度产生；
- 或某些末端力方向会要求无穷大的关节力矩。

### 9.2 例子：2R 机械臂的奇异位形

对位置 Jacobian：

$$
J_p(\theta) =
\begin{bmatrix}
-l_1\sin\theta_1 - l_2\sin(\theta_1+\theta_2) &
-l_2\sin(\theta_1+\theta_2) \\
l_1\cos\theta_1 + l_2\cos(\theta_1+\theta_2) &
l_2\cos(\theta_1+\theta_2)
\end{bmatrix}
$$

求行列式：

$$
\det J_p
=
\left(
-l_1\sin\theta_1 - l_2\sin(\theta_1+\theta_2)
\right) l_2\cos(\theta_1+\theta_2)
- \left(
-l_2\sin(\theta_1+\theta_2)
\right)\left(
l_1\cos\theta_1 + l_2\cos(\theta_1+\theta_2)
\right)
$$

整理后得到：

$$
\det J_p = l_1 l_2 \sin\theta_2
$$

因此奇异条件是：

$$
\sin\theta_2 = 0
$$

也就是：

$$
\theta_2 = 0 \quad \text{或} \quad \theta_2 = \pi
$$

几何解释：

- 当两根杆完全伸直或完全折叠时，末端瞬时可运动方向减少；
- Jacobian 失去满秩；
- 机器人在某些方向上“动不了”或者“非常难动”。

### 9.3 数值检验

若 $\theta_2 = 0$，则两杆共线。  
此时位置 Jacobian 两列会线性相关，因此秩降到 1。

这意味着：

- 末端只能沿某条切向方向瞬时运动；
- 无法在平面任意方向独立地生成速度。

## 10. Manipulability

### 10.1 直觉

manipulability 讨论的是：

> 在当前构型下，机器人“容易”往哪些方向动，或“容易”往哪些方向施力。

这通常用 manipulability ellipsoid 和 force ellipsoid 来可视化。

### 10.2 2R 机械臂的位置 manipulability

对正方 Jacobian，常用标量 manipulability 指标是：

$$
w = \sqrt{\det(JJ^T)}
$$

对 2R 平面位置 Jacobian，由于它是 $2\times2$，所以：

$$
w = |\det J_p|
$$

而前面已经求得：

$$
\det J_p = l_1 l_2 \sin\theta_2
$$

因此：

$$
w = l_1 l_2 |\sin\theta_2|
$$

对 $l_1=l_2=1$：

$$
w = |\sin\theta_2|
$$

### 10.3 例子：比较两种构型的 manipulability

#### 构型 A

取：

$$
\theta_2 = \frac{\pi}{2}
$$

则：

$$
w = \left|\sin\frac{\pi}{2}\right| = 1
$$

说明 manipulability 较高。

#### 构型 B

取：

$$
\theta_2 = 0
$$

则：

$$
w = |\sin 0| = 0
$$

说明 manipulability 退化到零，也就是奇异位形。

### 10.4 例子：算一个 manipulability ellipsoid 的矩阵

取姿态：

$$
\theta_1 = 0,\qquad \theta_2 = \frac{\pi}{2}
$$

此时：

$$
J_p =
\begin{bmatrix}
-1 & -1 \\
1 & 0
\end{bmatrix}
$$

因此：

$$
J_p J_p^T =
\begin{bmatrix}
2 & -1 \\
-1 & 1
\end{bmatrix}
$$

求特征值：

$$
\det
\left(
\begin{bmatrix}
2 & -1 \\
-1 & 1
\end{bmatrix}
- \lambda I
\right)
= 0
$$

即：

$$
\det
\begin{bmatrix}
2-\lambda & -1 \\
-1 & 1-\lambda
\end{bmatrix}
= (2-\lambda)(1-\lambda)-1=0
$$

展开：

$$
\lambda^2 - 3\lambda + 1 = 0
$$

所以：

$$
\lambda_{1,2} = \frac{3 \pm \sqrt{5}}{2}
$$

对应椭球主轴长度与 $\sqrt{\lambda_i}$ 成正比。

这个例子说明：

- 即使在非奇异构型，机器人在不同方向上的速度能力也不一样；
- manipulability ellipsoid 正是在可视化这种方向性差异。

## 11. 第 5 章和前四章怎么串起来

```mermaid
flowchart LR
    A["Chapter 2\njoint space / C-space"] --> B["Chapter 4\nT(θ)"]
    B --> C["Chapter 5\nV = J(θ) θdot"]
    C --> D["Chapter 5\nτ = J(θ)^T F"]
```

你可以把前五章看成逐层推进：

1. Chapter 2：先知道机器人配置怎么描述；
2. Chapter 3：再知道刚体位姿、速度、力怎么表示；
3. Chapter 4：再把关节变量映射到末端位姿；
4. Chapter 5：最后把关节速度映射到末端速度，把末端力映射回关节力矩。

## 12. 听课提醒

### 12.1 Jacobian 的每一列都要有几何感觉

不要把 Jacobian 当成一张表。  
每一列都表示“某个关节单独以单位速度运动时，末端 twist 会怎样变化”。

### 12.2 space Jacobian 和 body Jacobian 不要混

两者描述的是同一个真实运动，但参考表达不同：

- $J_s$ 对应 $V_s$；
- $J_b$ 对应 $V_b$。

### 12.3 奇异位形不只是“行列式为零”

行列式为零只是检测手段。  
真正要理解的是：机器人在那个构型下失去了某些方向的瞬时运动能力。

### 12.4 manipulability 和奇异性是同一件事的不同视角

- singularity 关注“退化”；
- manipulability 关注“方向性能力”；
- 二者都由 Jacobian 决定。

## 13. 本章串联例子总结

> [!example]
> 这一章的贯穿例子可以浓缩为一条完整链：
> 1. 先由第 4 章的 screw axes 写出 $J_s$；
> 2. 再改写成 $J_b$；
> 3. 用位置 Jacobian 算末端速度；
> 4. 再用 $J^T$ 把末端力映射成关节力矩；
> 5. 最后用 $\det J$ 和 $\det(JJ^T)$ 讨论奇异性与 manipulability。

如果这条链你能自己重算一遍，第 5 章就真正打通了。

## 14. 与前后章节的关系

- 前置章节：[[05-第4章 Forward Kinematics/第4章 Forward Kinematics：正运动学]]
- 总导航：[[01-总览与方法/课程地图与使用说明]]
- 附录：[[99-附录与速查/符号约定、公式写法与章节速查]]
