---
tags:
  - modern-robotics
  - chapter-2
  - c-space
---

# 第2章 Configuration Space：构型空间

## 1. 本章目标

第 2 章要解决的问题可以概括成一句话：

> 一个机器人系统的“状态”到底应该怎么几何化地描述？

这里的状态不是控制论里更广义的状态，而是 **configuration**，也就是机器人所有刚体构件在空间中的位置与姿态的整体描述。

## 2. 官方小节结构

- `2.1` Degrees of Freedom of a Rigid Body
- `2.2` Degrees of Freedom of a Robot
- `2.3.1` Configuration Space Topology
- `2.3.2` Configuration Space Representation
- `2.4` Configuration and Velocity Constraints
- `2.5` Task Space and Workspace

## 3. 核心概念一：刚体的自由度

### 3.1 平面刚体

平面中的刚体有 3 个自由度：

- 两个平移；
- 一个转动。

因此平面刚体的 configuration 可以用 3 个变量描述。

### 3.2 空间刚体

三维空间中的刚体有 6 个自由度：

- 三个位置自由度；
- 三个姿态自由度。

这件事非常关键，因为它直接成为后面 Chapter 3 中 $SE(3)$ 的背景。

### 3.3 例子：平面 2R 机械臂为什么是 2 自由度

考虑贯穿例子中的平面 2R 机械臂。它有两个转动关节，因此最自然的 configuration 写成：

$$
q =
\begin{bmatrix}
\theta_1 \\
\theta_2
\end{bmatrix}
$$

这里每个关节只提供一个独立变量，所以系统自由度就是 2。

这件事的概念意义是：

- 你不需要记录每个点的位置；
- 只要知道能唯一确定整体几何状态的最小变量即可；
- 对这个系统，$\theta_1,\theta_2$ 就足够了。

## 4. 核心概念二：机器人的自由度

机器人不是单个刚体，而是多个刚体通过关节连接而成，因此：

- 单个刚体本身有固有自由度；
- 关节通过约束减少相对运动自由度。

开放链里，一个一自由度的 revolute joint 或 prismatic joint 通常只保留一维相对运动。

### 4.1 Grubler 公式的思想

Grubler 公式不是魔法，它本质上是：

$$
\text{系统自由度} = \text{所有构件的自由度总和} - \text{约束数}
$$

在空间机构里常写成：

$$
\text{dof} = 6(N - 1 - J) + \sum_{i=1}^{J} f_i
$$

其中：

- $N$ 是 link 数，按惯例包含 ground；
- $J$ 是 joint 数；
- $f_i$ 是第 $i$ 个关节提供的自由度。

> [!warning]
> 这个公式对很多常见机构很好用，但对特殊退化结构、冗余约束和奇异构型要保持警惕。课程中也强调它常常给出的是方便的计数起点，而不是对所有结构都无条件可靠的最终结论。

### 4.2 例子：用 Grubler 公式数平面 2R 机械臂的自由度

对一个平面机构，常用版本是：

$$
\text{dof} = 3(N - 1 - J) + \sum_{i=1}^{J} f_i
$$

对平面 2R 机械臂：

- $N = 3$，因为包含 ground、link 1、link 2；
- $J = 2$；
- 两个关节都是转动关节，所以 $f_1 = f_2 = 1$。

代入得到：

$$
\text{dof} = 3(3 - 1 - 2) + (1 + 1)
$$

先算括号：

$$
3 - 1 - 2 = 0
$$

因此：

$$
\text{dof} = 3 \cdot 0 + 2 = 2
$$

这个计算说明：

- 两个关节确实给了系统两个独立运动变量；
- 它和我们直观上写出 $q = (\theta_1,\theta_2)$ 是一致的。

## 5. 核心概念三：Configuration Space

Configuration space, C-space，不是“关节角的表格”，而是：

> 机器人所有可能 configuration 构成的集合。

这是一种非常重要的思维升级：

- 以前你可能只把机器人看成机械臂本体；
- 现在要把“所有可能姿态的集合”看成一个几何空间。

### 5.1 C-space 的维数

C-space 的维数等于系统自由度。

因此：

- 平面刚体的 C-space 维数是 3；
- 空间刚体的 C-space 维数是 6；
- 一个 6R 串联机械臂的 C-space 在一般情形下维数是 6。

### 5.2 C-space 的拓扑

课程在 `2.3.1` 里特别强调 topology，是因为“维数相同”不代表“空间长得一样”。

典型例子：

- 平面 $E^2$ 和球面 $S^2$ 都是二维，但拓扑不同；
- 一个转角变量 $\theta$ 常被画成区间，但其真实拓扑是圆 $S^1$。

对机器人来说，这意味着：

- 你用坐标表示的图像只是某种展开；
- 真实的 configuration space 可能是环面、球面、乘积空间等。

例如，一个平面 2R 机械臂常用 $(\theta_1, \theta_2)$ 表示，但其拓扑更自然地写作：

$$
S^1 \times S^1
$$

这在几何上是一个二维环面。

### 5.3 例子：为什么不是 $\mathbb{R}^2$，而是 $S^1 \times S^1$

如果你把 $\theta_1,\theta_2$ 都画在区间 $[-\pi,\pi)$ 上，看起来像一个二维方形区域。  
但这只是坐标展开，不是真实拓扑。

原因是每个角变量都满足：

$$
\theta_i \equiv \theta_i + 2\pi
$$

例如：

$$
\theta_1 = -\pi \quad \text{和} \quad \theta_1 = \pi
$$

表示的是同一个关节方向。

所以二维方形的左右边必须粘起来，上下边也必须粘起来，最终得到的是一个环面，而不是普通平面。

这个例子想说明：

- 坐标图像可能是方形；
- 真实 configuration space 却是环面；
- 这就是 topology 和 coordinate representation 的区别。

## 6. 核心概念四：Configuration Space 的表示

课程在 `2.3.2` 里区分了两种思想：

### 6.1 最小坐标显式表示

优点：

- 参数少；
- 常常直观；
- 便于局部计算。

缺点：

- 容易出现坐标奇异；
- 有时一个局部图无法覆盖整个空间。

### 6.2 嵌入到高维空间的隐式表示

思路是把配置空间看成更高维欧氏空间中的一个约束曲面。

优点：

- 表达统一；
- 常和后续矩阵表示兼容；
- 更适合连接到 Chapter 3 的旋转矩阵和齐次变换。

缺点：

- 参数不是最少；
- 需要额外约束条件。

> [!important]
> 这正是 Modern Robotics 这门课的重要风格：  
> 它经常不追求“最少参数”，而更偏向“结构清楚、几何清楚、和后续运算统一”的表示法。

### 6.3 例子：同一个圆周上的姿态可以怎样表示

考虑平面中一个刚体只允许绕原点转动。它的 configuration 只有一个角度 $\theta$。

最小表示可以写成：

$$
\theta
$$

这是 1 维参数，最省变量。

但也可以写成二维嵌入形式：

$$
(\cos\theta,\sin\theta)
$$

这时变量有两个，但它们满足约束：

$$
x^2 + y^2 = 1
$$

也就是说，真实 configuration 是平面中的单位圆。

这个例子展示了：

- 最小表示：变量少，但有周期拼接问题；
- 高维嵌入：变量多，但几何结构更清楚。

## 7. 核心概念五：约束

### 7.1 Holonomic constraints

holonomic constraint 是 **构型约束**，可以写成 configuration 的方程。

一般形式：

$$
g(q) = 0
$$

它直接限制机器人能处于哪些 configuration。

### 7.2 Nonholonomic constraints

nonholonomic constraint 不是 configuration 本身的约束，而是 **速度层面的约束**。

常写为：

$$
A(q)\dot{q} = 0
$$

这类约束通常不能直接积分成仅关于 $q$ 的约束式。

经典直觉例子是轮式机器人：

- 可以向前滚；
- 不能侧向滑。

这不是“不能处在某个位置”，而是“不能以某种速度方向运动”。

### 7.3 Pfaffian constraints

Pfaffian 约束本质上就是线性的速度约束写法：

$$
A(q)\dot{q} = 0
$$

它是后续移动机器人和受约束系统分析的重要接口。

### 7.4 例子：差速小车的非完整约束

考虑平面移动机器人构型：

$$
q =
\begin{bmatrix}
x \\
y \\
\phi
\end{bmatrix}
$$

其中：

- $(x,y)$ 是车体中心位置；
- $\phi$ 是朝向。

若小车不能侧滑，则其瞬时速度在车体横向方向上必须为 0。这个约束可写成：

$$
-\sin\phi \, \dot{x} + \cos\phi \, \dot{y} = 0
$$

把它写成矩阵形式：

$$
\begin{bmatrix}
-\sin\phi & \cos\phi & 0
\end{bmatrix}
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\phi}
\end{bmatrix}
= 0
$$

这就是一个标准的 Pfaffian 约束。

为什么它是 nonholonomic？

因为它约束的是速度方向，而不是某个 configuration 方程 $g(q)=0$。  
小车可以到达很多位置，但到达这些位置的路径方向受限制。

## 8. 核心概念六：Task Space 与 Workspace

这部分很容易和 C-space 混淆，所以必须分清。

### 8.1 C-space

- 描述的是机器人本体的 configuration；
- 维数等于自由度；
- 关注“机器人自身能处于哪些状态”。

### 8.2 Task Space

- 描述任务最自然发生的空间；
- 不一定等同于 C-space；
- 也不一定等同于完整末端位姿空间。

例如：

- 若任务只是末端位置跟踪，task space 可能只取末端位置；
- 若任务是完整抓取定位，task space 可能是末端位姿空间。

### 8.3 Workspace

workspace 是末端可达集合的描述。

常见理解：

- 末端所有可达位置的集合；
- 或在更强语境下，末端所有可达位姿的集合。

> [!warning]
> workspace 不是 configuration space 的同义词。  
> C-space 是“机器人状态空间”，workspace 是“末端可达集合”。

### 8.4 例子：2R 机械臂的 C-space 和 workspace 不是一回事

继续使用长度 $l_1 = l_2 = 1$ 的平面 2R 机械臂。

它的 C-space 是：

$$
S^1 \times S^1
$$

但它的末端位置满足：

$$
x = l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2)
$$

$$
y = l_1 \sin\theta_1 + l_2 \sin(\theta_1 + \theta_2)
$$

代入 $l_1 = l_2 = 1$：

$$
x = \cos\theta_1 + \cos(\theta_1 + \theta_2)
$$

$$
y = \sin\theta_1 + \sin(\theta_1 + \theta_2)
$$

再看末端到原点的距离：

$$
r^2 = x^2 + y^2
$$

展开可得：

$$
r^2 = 1^2 + 1^2 + 2 \cos\theta_2 = 2 + 2\cos\theta_2
$$

因此：

$$
r = \sqrt{2 + 2\cos\theta_2}
$$

因为 $\cos\theta_2 \in [-1,1]$，所以：

$$
r \in [0,2]
$$

这说明它的末端位置 workspace 是一个以原点为中心、半径从 0 到 2 的圆盘，而不是二维环面。

这个例子清楚地说明了：

- C-space 描述关节状态；
- workspace 描述末端可达位置；
- 两者维数可能相同，但几何意义完全不同。

## 9. 这一章最重要的认知转变

### 9.1 从“机器人本体”转向“配置集合”

真正的难点不是公式，而是观念：

- 机器人不再只是图上的机械结构；
- 它还是一个 configuration manifold。

### 9.2 从“坐标”转向“空间”

坐标只是表示空间的方法，不是空间本身。

这为后面 Chapter 3 引入：

- $SO(3)$；
- $SE(3)$；
- 指数坐标；
- 李群表示

做好了思想准备。

## 10. 本章与前后章的关系

- 它承接 [[02-第1章 Preview/第1章 Preview：课程全景]] 中“先表示 configuration 再谈运动”的路线。
- 它为 [[04-第3章 Rigid-Body Motions/第3章 Rigid-Body Motions：刚体运动]] 提供了“为什么需要统一表示刚体配置”的背景。
- 它也为 [[05-第4章 Forward Kinematics/第4章 Forward Kinematics：正运动学]] 提供了“关节空间与末端空间不是一回事”的基础。

## 11. 听课提醒

> [!tip]
> 听这一章时，不要只盯着自由度公式。真正值得记住的是：
> - C-space 是一个空间，不只是一个向量；
> - topology 和 representation 要分开；
> - holonomic 与 nonholonomic 的区别在“约束落在哪一层”。

## 12. 本章串联例子总结

> [!example]
> 如果把这一章所有例子串起来，你会得到一条清晰链路：
> - 2R 机械臂告诉你如何数自由度；
> - 角变量周期性告诉你为什么 C-space 是 $S^1 \times S^1$；
> - 差速小车告诉你什么叫速度约束；
> - 末端位置圆盘告诉你 workspace 和 C-space 不是同一个空间。

这四个点放在一起，基本就是 Chapter 2 的骨架。
