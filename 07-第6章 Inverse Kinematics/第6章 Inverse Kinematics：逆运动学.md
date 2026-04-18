---
tags:
  - modern-robotics
  - chapter-6
  - inverse-kinematics
---

# 第6章 Inverse Kinematics：逆运动学

## 1. 本章目标

第 4 章解决的是：

> 已知关节变量 $\theta$，求末端位姿 $T(\theta)$。

第 6 章反过来问：

> 已知希望末端达到的位姿，怎么反求关节变量 $\theta$？

这就是逆运动学。

最核心的问题写成：

$$
\text{已知 } T_{sd}, \quad \text{求 } \theta \text{ 使得 } T(\theta)=T_{sd}
$$

或者如果末端用最小坐标 $x$ 描述，则写成：

$$
\text{已知 } x_d, \quad \text{求 } \theta \text{ 使得 } f(\theta)=x_d
$$

所以这一章的任务不是建立新几何对象，而是：

- 把 [[05-第4章 Forward Kinematics/第4章 Forward Kinematics：正运动学]] 里的正运动学反过来用；
- 把 [[06-第5章 Velocity Kinematics and Statics/第5章 Velocity Kinematics and Statics：速度运动学与静力学]] 里的 Jacobian 拿来做迭代更新；
- 建立“解析求解”和“数值求解”两条路线。

## 2. 官方小节结构

- Inverse Kinematics of Open Chains
- `6.2` Numerical Inverse Kinematics (Part 1 of 2)
- `6.2` Numerical Inverse Kinematics (Part 2 of 2)

官方第 6 章的重点很集中，基本就围绕两件事展开：

1. 逆运动学问题本身是什么；
2. Newton-Raphson 数值迭代怎么做。

## 3. 这一章为什么会接在第 4、5 章后面

这一章的逻辑链非常清楚：

```mermaid
flowchart LR
    A["第4章\n已知 theta 求 T(theta)"] --> B["第5章\n已知 theta-dot 求末端 twist"]
    B --> C["第6章\n已知目标位姿 反求 theta"]
```

你可以把它理解成：

- 第 4 章给你函数
  $$
  T(\theta)
  $$
- 第 5 章给你它的一阶线性近似工具
  $$
  J(\theta)
  $$
- 第 6 章就用这两个东西去做反求。

## 4. 逆运动学到底在解决什么

### 4.1 正运动学和逆运动学的方向相反

正运动学：

$$
\theta \longmapsto T(\theta)
$$

逆运动学：

$$
T_{sd} \longmapsto \theta
$$

所以你以后做题时，先问自己：

- 已知的是关节变量，还是末端目标？
- 求的是末端位姿，还是关节变量？

如果是“已知末端，求关节”，就是逆运动学。

### 4.2 为什么逆运动学比正运动学难

因为正运动学通常直接代公式即可，而逆运动学会遇到三类典型情况：

1. 无解；
2. 多解；
3. 初值敏感、局部收敛。

所以逆运动学不是简单“把正运动学公式倒过来”。

## 5. 解析逆运动学和数值逆运动学

### 5.1 解析法

解析逆运动学的想法是：

> 直接通过几何关系、三角关系或代数消元，把 $\theta$ 解出来。

优点：

- 解快；
- 能看到多解结构；
- 如果有闭式解，做题很漂亮。

缺点：

- 很多机器人根本不好推；
- 结构稍复杂就会很麻烦；
- 不是所有机构都有漂亮解析解。

### 5.2 数值法

数值逆运动学的想法是：

> 不直接写闭式解，而是从一个初值出发，反复迭代逼近目标。

这一章重点讲的就是 Newton-Raphson。

## 6. 最小坐标版数值逆运动学

### 6.1 问题写法

先假设末端用一个最小坐标向量 $x$ 表示。

正运动学写成：

$$
x = f(\theta)
$$

逆运动学要求：

$$
f(\theta_d)=x_d
$$

等价写成求根问题：

$$
x_d - f(\theta_d)=0
$$

### 6.2 Newton-Raphson 的核心思想

在当前猜测值 $\theta_i$ 附近，把 $f(\theta)$ 做一阶近似：

$$
f(\theta_i + \Delta \theta)\approx f(\theta_i)+J(\theta_i)\Delta\theta
$$

如果希望更新后更接近目标 $x_d$，就让：

$$
x_d \approx f(\theta_i)+J(\theta_i)\Delta\theta
$$

移项得到：

$$
x_d-f(\theta_i)\approx J(\theta_i)\Delta\theta
$$

记误差为

$$
e_i = x_d - f(\theta_i)
$$

则有：

$$
e_i \approx J(\theta_i)\Delta\theta
$$

如果 $J$ 可逆：

$$
\Delta\theta = J(\theta_i)^{-1} e_i
$$

于是更新公式就是：

$$
\theta_{i+1} = \theta_i + J(\theta_i)^{-1}(x_d-f(\theta_i))
$$

### 6.3 为什么会引出伪逆

真实机器人里，常常遇到：

- $J$ 不是方阵；
- $J$ 在奇异位形附近不可逆；
- 冗余机器人有很多解；
- 欠驱动机器人无法精确达到目标。

所以课程里改成统一写法：

$$
\Delta\theta = J(\theta_i)^\dagger e_i
$$

更新公式变成：

$$
\theta_{i+1} = \theta_i + J(\theta_i)^\dagger \big(x_d-f(\theta_i)\big)
$$

### 6.4 数值逆运动学算法

1. 选初值 $\theta_0$
2. 计算误差
   $$
   e_i = x_d - f(\theta_i)
   $$
3. 如果误差足够小，停止
4. 否则计算
   $$
   \Delta\theta = J(\theta_i)^\dagger e_i
   $$
5. 更新
   $$
   \theta_{i+1} = \theta_i + \Delta\theta
   $$
6. 重复

## 7. 位姿矩阵版数值逆运动学

机器人末端更自然的表示是：

$$
T_{sb}(\theta)\in SE(3)
$$

所以课程第二部分把“误差”改成“位姿误差对应的 twist”。

### 7.1 当前位姿和目标位姿

- 目标位姿：
  $$
  T_{sd}
  $$
- 当前猜测关节值 $\theta_i$ 下的末端位姿：
  $$
  T_{sb}(\theta_i)
  $$

### 7.2 为什么会出现 $T_{bd}$

在 body frame 里写误差最自然，所以先算：

$$
T_{bd} = T_{sb}(\theta_i)^{-1} T_{sd}
$$

它表示：

> 目标 frame `{d}` 相对当前末端 frame `{b}` 的位姿

### 7.3 为什么会出现矩阵对数

我们不是想直接看“差了一个矩阵”，而是想问：

> 还差一个什么 twist，沿它运动一小步，就能从当前位姿逼近目标位姿？

这时用矩阵对数：

$$
[V_b] = \log(T_{bd})
$$

再转成 6 维向量：

$$
V_b = \operatorname{se3ToVec}\big(\log(T_{bd})\big)
$$

这个 $V_b$ 就扮演最小坐标算法里误差 $e_i$ 的角色。

### 7.4 为什么用 body Jacobian

因为误差 twist 是在当前末端 `{b}` 里表达的，所以更新时配套使用 body Jacobian：

$$
J_b(\theta_i)
$$

于是更新公式变成：

$$
\theta_{i+1}
=
\theta_i + J_b(\theta_i)^\dagger V_b
$$

### 7.5 停止条件

课程里会把 $V_b$ 分成：

- 角速度部分 $\omega_b$
- 线速度部分 $v_b$

如果两者都足够小，就认为已经收敛：

$$
\|\omega_b\| < \epsilon_\omega,
\qquad
\|v_b\| < \epsilon_v
$$

## 8. 逆速度运动学

还可以问一个更“瞬时”的问题：

> 如果我不求关节位置，只要求某个期望末端 twist，关节速度怎么求？

这就是逆速度运动学。

如果期望 twist 和 Jacobian 在同一个坐标系表达，则：

$$
\dot{\theta} = J^\dagger V_d
$$

它和第 5 章的正向关系正好相对：

$$
V = J\dot{\theta}
\qquad\Longleftrightarrow\qquad
\dot{\theta} = J^\dagger V_d
$$

## 9. 一组最重要的概念区分

### 9.1 正运动学 vs 逆运动学

- 正运动学：已知 $\theta$ 求 $T$
- 逆运动学：已知目标 $T$ 求 $\theta$

### 9.2 解析逆运动学 vs 数值逆运动学

- 解析法：直接写闭式解
- 数值法：不断迭代逼近

### 9.3 坐标误差 vs 位姿误差

- 坐标误差：$x_d-f(\theta_i)$
- 位姿误差：$\operatorname{se3ToVec}(\log(T_{sb}^{-1}T_{sd}))$

### 9.4 Jacobian 逆 vs 伪逆

- 可逆时：$J^{-1}$
- 一般情况：$J^\dagger$

### 9.5 收敛到某个解 vs 找到全部解

数值逆运动学通常只会：

> 从某个初值出发，收敛到附近的一个解

它不保证给你全部解。

## 10. 一条最重要的做题流程

### 10.1 如果末端用最小坐标表示

1. 定义误差
   $$
   e_i = x_d - f(\theta_i)
   $$
2. 线性化
   $$
   e_i \approx J(\theta_i)\Delta\theta
   $$
3. 用伪逆求更新量
   $$
   \Delta\theta = J(\theta_i)^\dagger e_i
   $$
4. 更新
   $$
   \theta_{i+1}=\theta_i+\Delta\theta
   $$

### 10.2 如果末端用位姿矩阵表示

1. 计算当前位姿
   $$
   T_{sb}(\theta_i)
   $$
2. 计算 body-frame 误差位姿
   $$
   T_{bd}=T_{sb}(\theta_i)^{-1}T_{sd}
   $$
3. 计算误差 twist
   $$
   V_b=\operatorname{se3ToVec}(\log(T_{bd}))
   $$
4. 用 body Jacobian 更新
   $$
   \theta_{i+1}=\theta_i+J_b(\theta_i)^\dagger V_b
   $$
5. 检查
   $$
   \|\omega_b\|,\ \|v_b\|
   $$
   是否足够小

## 11. 详细例子 1：2R 平面机械臂的解析逆运动学

设 2R 平面机械臂长度为：

$$
l_1=l_2=1
$$

末端目标位置是：

$$
(x,y)=(1,1)
$$

正运动学是：

$$
x = l_1\cos\theta_1 + l_2\cos(\theta_1+\theta_2)
$$

$$
y = l_1\sin\theta_1 + l_2\sin(\theta_1+\theta_2)
$$

### 11.1 先解 $\theta_2$

由余弦定理：

$$
x^2+y^2 = l_1^2+l_2^2+2l_1l_2\cos\theta_2
$$

代入数值：

$$
1^2+1^2 = 1^2+1^2+2\cos\theta_2
$$

得到：

$$
2 = 2 + 2\cos\theta_2
$$

所以：

$$
\cos\theta_2=0
$$

于是：

$$
\theta_2=\frac{\pi}{2}
\quad \text{或} \quad
\theta_2=-\frac{\pi}{2}
$$

### 11.2 再解 $\theta_1$

先取

$$
\theta_2=\frac{\pi}{2}
$$

则

$$
\theta_1 = \operatorname{atan2}(y,x) - \operatorname{atan2}(l_2\sin\theta_2,\ l_1+l_2\cos\theta_2)
$$

代入：

$$
\theta_1
=
\operatorname{atan2}(1,1)-\operatorname{atan2}(1,1)
=0
$$

得到一组解：

$$
(\theta_1,\theta_2)=\left(0,\frac{\pi}{2}\right)
$$

再取

$$
\theta_2=-\frac{\pi}{2}
$$

则

$$
\theta_1
=
\operatorname{atan2}(1,1)-\operatorname{atan2}(-1,1)
=
\frac{\pi}{2}
$$

得到另一组解：

$$
(\theta_1,\theta_2)=\left(\frac{\pi}{2},-\frac{\pi}{2}\right)
$$

这个例子最重要的作用是说明：

- 逆运动学通常多解；
- 同一个目标末端位置，可以有不同关节构型。

## 12. 详细例子 2：数值逆运动学的最小坐标版

设：

$$
f(\theta)=\theta^2
$$

目标是：

$$
x_d=2
$$

所以要求解：

$$
\theta^2 = 2
$$

真解是：

$$
\theta=\pm\sqrt{2}
$$

### 12.1 选初值

取：

$$
\theta_0=1
$$

误差：

$$
e_0 = x_d-f(\theta_0)=2-1^2=1
$$

Jacobian：

$$
J(\theta)=\frac{df}{d\theta}=2\theta
$$

所以：

$$
J(\theta_0)=2
$$

更新量：

$$
\Delta\theta_0 = J(\theta_0)^{-1} e_0 = \frac{1}{2}
$$

更新后：

$$
\theta_1=1.5
$$

### 12.2 再迭代一步

误差：

$$
e_1 = 2-1.5^2 = -0.25
$$

Jacobian：

$$
J(\theta_1)=3
$$

更新量：

$$
\Delta\theta_1 = \frac{-0.25}{3}\approx -0.0833
$$

所以：

$$
\theta_2 \approx 1.4167
$$

已经很接近：

$$
\sqrt{2}\approx 1.4142
$$

这个例子说明：

- 数值逆运动学不是一下求出解；
- 而是利用 Jacobian 的局部线性近似一步步逼近。

## 13. 本章最该记住的公式

$$
f(\theta_d)=x_d
$$

$$
x_d-f(\theta_i)\approx J(\theta_i)\Delta\theta
$$

$$
\theta_{i+1}=\theta_i+J(\theta_i)^\dagger \big(x_d-f(\theta_i)\big)
$$

$$
T_{bd}=T_{sb}(\theta_i)^{-1}T_{sd}
$$

$$
V_b=\operatorname{se3ToVec}\big(\log(T_{bd})\big)
$$

$$
\theta_{i+1}=\theta_i+J_b(\theta_i)^\dagger V_b
$$

$$
\dot{\theta}=J^\dagger V_d
$$

## 14. 本章最容易混的点

> [!warning]
> 逆运动学不是把正运动学公式“直接倒过来写”。

> [!warning]
> 数值逆运动学通常只找到一个局部解，不会自动给你全部解。

> [!warning]
> 初值不同，可能收敛到不同解，也可能不收敛。

> [!warning]
> 用位姿矩阵版算法时，误差不是简单做 $T_{sd}-T_{sb}$，而是要先算相对位姿，再取矩阵对数。

> [!warning]
> 误差 twist 如果在 body frame 表达，就应该配 body Jacobian。

## 15. 和前后章节的关系

- 它建立在 [[05-第4章 Forward Kinematics/第4章 Forward Kinematics：正运动学]] 的 $T(\theta)$ 之上。
- 它依赖 [[06-第5章 Velocity Kinematics and Statics/第5章 Velocity Kinematics and Statics：速度运动学与静力学]] 的 Jacobian 和伪逆思想。
- 它为后面的运动控制打基础。

## 16. 复习时最推荐的顺序

1. 先看本章第 4 节，确认“逆运动学到底在求什么”
2. 再看第 6 节，理解最小坐标版 Newton-Raphson
3. 再看第 7 节，理解为什么位姿误差要写成 twist
4. 最后看第 13 节，把核心公式压缩记住

## 17. 相关入口

- 上一章：[[06-第5章 Velocity Kinematics and Statics/第5章 Velocity Kinematics and Statics：速度运动学与静力学]]
- 前三章总表：[[01-总览与方法/前三章公式、概念与符号总表]]
- 前五章复习地图：[[01-总览与方法/前五章复习地图]]
- 附录速查：[[99-附录与速查/符号约定、公式写法与章节速查]]
