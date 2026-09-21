# 控制与推力建模方法

## 摘要

本项目的 `px4ctrl` 采用级联几何位置控制。外环在 ENU 世界系中由位置、速度和加速度参考构造期望合力；该合力方向与给定偏航组合为期望姿态四元数和归一化集体推力。默认模式将四元数和推力通过 MAVLink `SET_ATTITUDE_TARGET` 发送给 PX4；可选 $SO(3)$ 模式则使用精确群对数姿态误差生成机体系角速度设定点。PX4 负责内部角速度、力矩和电机控制。

因此，它不是直接输出力矩 $M$ 的全状态 $SO(3)$ 力矩控制器，但在 `use_bodyrate_ctrl=true` 时已实现 $SO(3)$ 姿态误差到角速度的几何外环。准确名称是：**以 PX4 角速度内环为执行层的级联几何位置-$SO(3)$ 姿态控制器**。

实现对应 [controller.py](../px4ctrl/px4ctrl/controller.py)、[fsm.py](../px4ctrl/px4ctrl/fsm.py) 与 [sim.yaml](../px4ctrl/config/sim.yaml)。本文记录的是当前代码实际实现的方法，不把 PX4 内部控制器误写成本项目自行实现的控制律。

## 坐标系与符号

- $W$：世界系，采用 ENU，坐标为 $(E,N,U)$。
- $B$：机体系，采用 FLU，坐标为 $(F,L,U)$。
- $R$：从机体系到世界系的旋转矩阵，$R$ 属于 $SO(3)$。
- $p,v,a$：当前位置、速度、加速度；下标 $d$ 表示参考值。
- $g=9.81\,\mathrm{m/s^2}$：重力常数，世界系单位上向量为 $e_3=(0,0,1)^T$。
- $q$：四元数。外环输出先保持 ENU/FLU，再由链路层统一转换为 PX4 的 NED/FRD。

四旋翼的平移动力学可写为：

$$
 m \ddot{p} = f R e_3 - m g e_3 + d,
$$

其中 $m$ 是质量，$f$ 是集体推力，$d$ 汇集了空气动力、模型误差和外扰。项目不显式使用质量求控制量，而是直接将归一化推力映射到加速度。

## 外环位置与速度控制

定义误差：

$$
 e_p = p_d-p, \qquad e_v=v_d-v.
$$

当前控制器的总期望加速度是：

$$
 a_c = a_d + K_v e_v + K_p e_p + g e_3.
$$

其中 $K_p=\operatorname{diag}(6,6,5)$，$K_v=\operatorname{diag}(4,4,5)$，来自 [sim.yaml](../px4ctrl/config/sim.yaml)。$a_d$ 是轨迹前馈加速度；平滑航点通过五次多项式同时提供 $p_d,v_d,a_d$，避免将远距离航点作为位置阶跃送入控制器。

在理想姿态跟踪、未触发限幅、推力模型准确时，$f R e_3$ 与 $m a_c$ 对齐，闭环误差近似满足：

$$
 \ddot{e}_p + K_v \dot{e}_p + K_p e_p = 0.
$$

这说明 $K_p$ 决定位置恢复强度，$K_v$ 提供阻尼。该结论仅在姿态内环足够快、未发生倾角饱和且参考平滑时成立。

## 从合力方向构造期望姿态

令 $a_c=(a_x,a_y,a_z)^T$，以实测偏航 $\psi$ 构造横滚 $\phi$ 与俯仰 $\theta$。项目采用 Z-Y-X 欧拉顺序：

$$
 R_d = R_z(\psi) R_y(\theta) R_x(\phi).
$$

机体 $z$ 轴应对齐 $a_c$。当前实现使用精确反解：

$$
 \sin(\phi)=\frac{a_x\sin(\psi)-a_y\cos(\psi)}{\lVert a_c\rVert},
$$

$$
 	heta=\operatorname{atan2}\left(a_x\cos(\psi)+a_y\sin(\psi),a_z\right).
$$

因此，横滚与俯仰并非小角度近似。使用实测偏航而非期望偏航的原因是横向加速度方向必须对应当前可实现的机体系方向；当前任务保持固定偏航，故该选择不会引入显著偏航过渡误差。

随后以 Z-Y-X 顺序合成四元数 $q_d$。若 IMU 与里程计姿态同时有效，控制器还会应用姿态补偿：

$$
 q_{\mathrm{cmd}}=q_{\mathrm{imu}}q_{\mathrm{odom}}^{-1}q_d.
$$

它用于补偿 IMU 姿态和里程计姿态之间的测量偏差；若任一姿态不可用，则直接使用 $q_d$。

## 倾角限幅与可实现性

配置给定 $|\phi|,|\theta| \leq 25^\circ$。悬停附近，该限制对应近似最大水平加速度：

$$
 a_{h,\max} \approx g\tan(25^\circ) \approx 4.6\,\mathrm{m/s^2}.
$$

严格地说，$a_c$ 中同时包含位置反馈、速度反馈和加速度前馈。因此轨迹约束不能只限制 $a_d$；必须为 $K_p e_p+K_v e_v$ 预留裕量。此前 $0.8 m/s, 1.0 m/s^2$ 的航段在起步阶段持续饱和，已证明只限制前馈不足。当前默认航段限制为 $0.25 m/s, 0.25 m/s^2$。

控制器在限幅前记录 `tilt_saturated`。在 `CMD_CTRL` 中，该标记持续超过 $0.5 s$ 时，状态机将期望点重置到当前实测位置并切换到 `AUTO_HOVER`。这是一项可实现性保护，而不是控制性能指标。

## 推力命令与倾角补偿

定义推力到机体轴加速度映射：

$$
 a_B^z = u T_a,
$$

其中 $u$ 是归一化推力，$T_a$ 是 `thr2acc`。由悬停比例 `hover_percentage` 初始化：

$$
 T_a=\frac{g}{u_hover}.
$$

倾斜时，推力在世界竖直方向的有效比例为：

$$
 c_t=\cos(\phi)\cos(\theta).
$$

故当前推力指令为：

$$
 u=\frac{a_z}{T_a c_t}.
$$

当 `tilt_compensation=true` 时，$c_t$ 生效；若关闭则取 $c_t=1$。该补偿避免飞行器在横向机动时因竖直推力投影减少而掉高。发送前 $u$ 被截断到 $[0,1]$。

## 带遗忘因子的在线推力辨识

真机可将 `online_estimate` 打开，对模型 $a_B^z=u T_a$ 进行标量递推最小二乘。用延迟配对后的样本 $(u_k,a_{B,k}^z)$，当前实现采用：

$$
 \gamma_k=\frac{1}{\rho^2+u_k P_{k-1}u_k},
$$

$$
 L_k=\gamma_k P_{k-1}u_k,
$$

$$
 T_{a,k}=T_{a,k-1}+L_k(a_{B,k}^z-u_kT_{a,k-1}),
$$

$$
 P_k=\frac{(1-L_ku_k)P_{k-1}}{\rho^2}, \qquad \rho^2=0.998.
$$

推力历史按 $35--45 ms$ 延迟窗口与当前 IMU 比力配对。量测必须是 FLU 机体系的比力 $a_B^z$，悬停时约为 $+g$；不能使用世界系竖直加速度。为防止符号、坐标或初始标定错误导致辨识发散，$T_a$ 被约束在初值的 `0.5--2.0` 倍范围内，越界更新被拒绝并记录。

## 状态机与安全层

控制率由状态机包裹，状态包括 `AUTO_TAKEOFF`、`AUTO_HOVER`、`CMD_CTRL` 和 `AUTO_LAND`。

- 里程计超时：停止控制，不能继续使用陈旧反馈。
- 制导指令超过 `0.5 s` 未更新：从 `CMD_CTRL` 转入当前点悬停。
- 持续倾角饱和：$0.5 s$ 后转入当前点悬停。
- 脚本正常结束、异常或一次 `Ctrl-C`：请求降落并等待上锁。

这些机制不改变名义控制率，但定义了视觉量测失效、估计器停更或轨迹不可实现时的闭环退化行为。

## SO(3) 角速度外环

当 `use_bodyrate_ctrl=true` 时，控制器不再把 $q_{\mathrm{cmd}}$ 直接交给 PX4 姿态环，而是令当前姿态为 $R$、补偿后的期望姿态为 $R_d$，定义：

$$
R_e=R^T R_d,
\qquad
e_R=\operatorname{Log}(R_e),
\qquad
e_{\Omega} = \Omega - R^T R_d \Omega_d.
$$

其中 $\operatorname{Log}:SO(3)\rightarrow\mathbb{R}^3$ 是主值旋转群对数：若误差旋转的轴角表示为 $(n,\theta)$，则 $\operatorname{Log}(R_e)=\theta n$，取 $\theta\in[-\pi,\pi]$。实现直接由单位误差四元数 $q_e=(v,w)$ 计算：

$$
e_R=\frac{2\operatorname{atan2}(\lVert v\rVert,w)}{\lVert v\rVert}v.
$$

这不是小角度近似。与 $\frac12(R_d^TR-R^TR_d)^\vee=\sin(\theta)n$ 相比，群对数在大角度时不会将误差从 $\theta$ 压缩为 $\sin(\theta)$。$e_{\Omega}$ 表示当前机体系角速度与运输到当前机体系后的期望角速度之差。

设世界系期望偏航速率为 $\dot{\psi}_d$，则先得到期望机体系角速度 $\Omega_d$，再运输到当前机体系：

$$
\Omega_{ff}=R_e\Omega_d.
$$

发送给 PX4 的 FLU 机体系角速度为：

$$
\Omega_{cmd}=\operatorname{sat}_{\omega_{\max}}\left(\Omega_{ff}+K_R e_R-k_{\Omega}e_{\Omega}\right).
$$

其中 $K_R=\operatorname{diag}(KAngR,KAngP,KAngY)$，$k_{\Omega}$ 是 `so3.rate_damping`，$\omega_{\max}$ 是 `so3.max_bodyrate`。当前仿真和真机配置均采用 $\omega_{\max}=3\,\mathrm{rad/s}$，这是发送前的安全限幅，不是小角度假设。

## 与直接力矩 SO(3) 控制的区别

经典全状态 $SO(3)$ 几何控制进一步在刚体动力学上构造力矩：

$$
 M=-k_R e_R-k_{\Omega}e_{\Omega}+\Omega \times J\Omega
 -J\left(\hat{\Omega}R^T R_d\Omega_d-R^T R_d\dot{\Omega}_d\right).
$$

其关键是：显式使用转动惯量 $J$、输出力矩 $M$ 或电机分配命令，并对整个刚体旋转闭环给出稳定性分析。

本项目没有这一步。$SO(3)$ 模式输出 $\Omega_{\mathrm{cmd}}$ 与 $u$，PX4 的内部角速度控制器完成力矩/电机闭环；默认模式仍输出 $q_{\mathrm{cmd}}$ 与 $u$。因此论文中应表述为“采用 PX4 角速度内环的级联几何位置-$SO(3)$ 姿态控制”，而不要称为“本文实现了直接力矩的完整 $SO(3)$ 控制器”。

## 与 6D 估计和 EKF 的接口

视觉 6D 算法与 EKF 应只替换外环的参考来源，而不改动 `px4ctrl` 的控制出口。建议估计器输出：目标共享 ENU 位置、速度、姿态、角速度、协方差、置信度和时间戳。制导器将估计目标状态转成 tracker 的 $(p_d,v_d,a_d,\psi_d)$，再交给现有 `CMD_CTRL`。

闭环接入分为影子评估、半闭环和全闭环。若量测超时、协方差过大或创新异常，应停止更新制导命令，使现有新鲜度保护将飞行器转入悬停。这样视觉误差、EKF 误差与底层控制误差可以分别记录和分析。

## 复现参数与限制

当前仿真默认值为 $K_p=\operatorname{diag}(6,6,5)$、$K_v=\operatorname{diag}(4,4,5)$、最大倾角 $25^\circ$、控制任务频率 $20\,\mathrm{Hz}$、`hover_percentage=0.2893`。$SO(3)$ body-rate 模式默认关闭，必须先通过仿真和单机低高度验收；在线 RLS 在仿真默认关闭，真实无人机启用严格窗口辨识时应使用 $50\,\mathrm{Hz}$。

本文方法的前提是 PX4 角速度内环稳定、姿态估计可用、坐标变换正确且推力模型在有效范围内。若后续研究需要直接比较直接力矩 $SO(3)$ 控制与 PX4 角速度内环方案，应新增直接力矩接口、惯量模型、力矩分配及对应的台架和飞行验证；这是一项独立控制器研发工作，不应与当前 6D 视觉/EKF 接入混为一谈。
