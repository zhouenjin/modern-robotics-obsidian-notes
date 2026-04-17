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
