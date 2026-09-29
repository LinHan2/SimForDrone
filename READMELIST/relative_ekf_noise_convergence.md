# 双机相对 Bearing-Box EKF 与控制工具链

## 状态与预测

`tracking/relative_ekf.py` 迁移自 `Airsim2box/src/estimation/ekf_relative.py`。共享 ENU 下状态为

$$
x=[\delta p,\ \delta v,\ a_t,\ \alpha]^T,
\qquad \delta p=p_t-p_o,
$$

其中 $a_t$ 是目标绝对世界加速度，$\alpha$ 是 bearing-box 的尺度。按论文式 (34)，以观测机
世界加速度 $u_k=a_{o,k}$ 为已知输入，离散系统写为

$$
x_{k+1}=F_kx_k+B_ku_k+w_k,
$$

$$
F_k=
\begin{bmatrix}
I_3 & \Delta t I_3 & \tfrac12\Delta t^2 I_3 & 0_{3\times1}\\
0_{3\times3} & I_3 & \Delta t I_3 & 0_{3\times1}\\
0_{3\times3} & 0_{3\times3} & I_3 & 0_{3\times1}\\
0_{1\times3} & 0_{1\times3} & 0_{1\times3} & 1
\end{bmatrix},
\qquad
B_k=
\begin{bmatrix}
-\tfrac12\Delta t^2 I_3\\
-\Delta t I_3\\
0_{3\times3}\\
0_{1\times3}
\end{bmatrix}.
$$

因此 $w_k\sim\mathcal N(0,Q)$，其中

$$
Q=\operatorname{diag}(\sigma_p^2I_3,\ \sigma_v^2I_3,\ \sigma_a^2I_3,\ \sigma_\alpha^2).
$$

过程噪声标准差采用 Airsim2box 已调参数：$\sigma_p=0.002\,m$、
$\sigma_v=0.1095\,m/s$、$\sigma_a=0.2449\,m/s^2$、$\sigma_\alpha=0.0949\,m$。
不使用论文外的 jerk 状态或垂直加速度伪先验。

## 量测

由 3D box 八角点经 Lemma 1 得到相机系归一化位置 $\bar p_c$，转到世界系得到
$\bar T_{w,k}=R_{c,k}^w\bar p_{c,k}$。令 $B_k=[b_{1,k}\ b_{2,k}]$ 为 $h_k$ 在 $S^2$ 切空间的
正交基，即 $B_k^TB_k=I_2$、$B_k^Th_k=0$，且 $B_kB_k^T=I_3-h_kh_k^T$。代码选择与 $h_k$ 最不平行
的世界坐标轴并将其正交投影得到 $b_{1,k}$，再以 $b_{2,k}=h_k\times b_{1,k}$ 构造基。联合伪线性量测为

$$
z_k=H_kx_k+\epsilon_k,
\qquad
z_k=
\begin{bmatrix}
0_{3\times1}\\
B_k^Tg_{ENU}
\end{bmatrix},
$$

$$
H_k=
\begin{bmatrix}
I_3 & 0_{3\times3} & 0_{3\times3} & -\bar T_{w,k}\\
0_{2\times3} & 0_{2\times3} & B_k^T & 0_{2\times1}
\end{bmatrix},
\qquad
R=\operatorname{diag}(\sigma_{bb}^2I_3,\ \sigma_h^2I_2).
$$

第一行即 $\delta p-\alpha\bar T_w=0$，第二行即 $B_k^Ta_t=B_k^Tg_{ENU}$。该五维量测与原本
$P_h a_t=P_hg_{ENU}$ 的秩 $2$ 约束等价，但没有冗余 measurement 行。其中 $h$ 是目标推力方向，
$g_{ENU}=(0,0,-9.81)^T$。实现按 bearing-box 后 tilt 的顺序使用 Joseph 协方差更新、PSD
投影、状态限幅和遗忘因子 $1.001$。量测标准差为 $0.102\,m$ 与 $0.03\,m/s^2$。

当前 $R_{tilt}=\sigma_h^2I_2$ 使用已调的固定方差。网络接入切空间协方差 $\Sigma_\xi$ 后，可改为
$R_{tilt}=\|\hat a_t-g_{ENU}\|^2\Sigma_\xi$；在没有经过标定的网络不确定度前，不启用该状态相关权重。

当前 `relative-ekf` 的共享位置输入只用于联调时构造 $\bar T_w$ 和初始化 $\alpha$，并不等同于
真实的 bearing-box 前端。接入相机前必须将其替换为八角点检测、相机外参与 Lemma 1 计算。

## 控制与验收

当前一键实验的 tracker 使用 **truth** 目标状态；相对 EKF 仍是只读 shadow，不向 tracker
发送控制参考。将来接入经验证的相对估计后，才能以
$p_t=p_o+\delta p$、$v_t=v_o+\delta v$、$a_t$ 替换这一输入。
target 轨迹的解析加速度 `reference_a` 只用于离线比较，不以遥测速率差分伪造加速度真值。

### 位置与三轴姿态参考

令 $p_t^W,v_t^W,a_t^W$ 为共享 ENU 中的目标状态，$p_o^W$ 为 tracker 共享位置，
$p_o^L$ 为其 PX4 local ENU 位置，$d^W$ 为起飞后锁存的世界系相对站位。
`PositionTrackerV0` 生成的 tracker local 参考为

$$
p_d^L=p_o^L+(p_t^W+d^W-p_o^W),\qquad v_d=v_t^W,\qquad a_d=a_t^W.
$$

几何偏航 $\psi_g=\operatorname{atan2}(p_{t,N}^W-p_{o,N}^W,\,
p_{t,E}^W-p_{o,E}^W)$ 指向目标。飞行入口从 tracker 当前实测偏航 $\psi_0$ 起步，
每周期按最短角路径限制偏航参考的变化：

$$
\Delta\psi_k=\operatorname{atan2}(\sin(\psi_g-\psi_{k-1}),\cos(\psi_g-\psi_{k-1})),
\qquad
\psi_k=\psi_{k-1}+\operatorname{clip}(\Delta\psi_k,-\omega_{\max}\Delta t_k,\omega_{\max}\Delta t_k).
$$

默认 $\omega_{\max}=0.35\,\mathrm{rad/s}$；这是**偏航参考**限速，不是实测机体角速度保证。
`LinearControl` 用位置/速度反馈和加速度前馈计算含重力的世界系期望加速度：

$$
a_c=a_d+K_v(v_d-v_o)+K_p(p_d^L-p_o^L)+g e_3,
\quad K_p=\operatorname{diag}(6,6,5),\quad K_v=\operatorname{diag}(4,4,5).
$$

其中 $a_c$ 的水平分量包括位置、速度和前馈三项，不能仅限幅 $a_d$。
令 $a_z^+=\max(a_{c,z},0.1g)$（非有限值也回退到 $0.1g$），
$a_h=(a_{c,x},a_{c,y})$，$\theta_{\max}$ 为倾角预算。当前实现对**水平向量整体**限幅：

$$
h_{\max}=a_z^+\tan\theta_{\max},\qquad
	ilde a_h=a_h\min\left(1,\frac{h_{\max}}{\|a_h\|}\right),
$$

在 $\|a_h\|=0$ 时直接取 $\tilde a_h=0$。然后用**同一期望偏航** $\psi_k$ 反解
roll $\phi$ 和 pitch $\theta$，并合成姿态，而不是以实测偏航反解倾角却以期望偏航合成姿态：

$$
\phi=\arcsin\frac{\tilde a_x\sin\psi_k-\tilde a_y\cos\psi_k}
{\sqrt{\tilde a_x^2+\tilde a_y^2+a_{c,z}^2}},\qquad
	heta=\operatorname{atan2}(\tilde a_x\cos\psi_k+\tilde a_y\sin\psi_k,\,a_z^+),
\qquad R_d=R_z(\psi_k)R_y(\theta)R_x(\phi).
$$

tracker 默认 $\theta_{\max}=20^\circ$（[run_tracker.py](../tracking/tracking/run_tracker.py)
的 `--max-tilt-deg`）；target 保持 [sim.yaml](../px4ctrl/config/sim.yaml) 的 35° 配置。
当 $a_{c,z}>0$ 时，向量限幅使组合倾角
$\arccos(\cos\phi\cos\theta)\leq\theta_{\max}$；垂向指令异常时的除数下限
仅用于数值保护，不应解读为无条件飞行安全保证。`tilt_saturated` 记录限幅发生，
不会自动切回悬停。

姿态输出为四元数 $q_d$；IMU 和里程计姿态均有效时，指令四元数为
$q_{\mathrm{cmd}}=q_{\mathrm{imu}}q_{\mathrm{odom}}^{-1}q_d$，否则直接使用 $q_d$。
默认姿态加归一化推力经 MAVLink 发给 PX4，机体角速度/力矩内环由 PX4 执行。
推力以**原始垂向指令**计算：

$$
u=\frac{a_{c,z}}{T_a\,s},\qquad
s=\cos\phi\cos\theta,\qquad
T_a=\frac{g}{\texttt{hover\_percentage}},
$$

其中 $s$ 是启用倾角补偿且取有限正值时的系数；关闭补偿或系数无效时取 $s=1$。实际下发前
FSM 将 $u$ 限制到 $[0,1]$。倾角限幅限制参考姿态幅度，不等于实测姿态限幅、
连续可见性保障或图像反馈控制；需用 tracker 真实机载画面检查轨迹过程。

运行前先重启 Isaac/PX4 场景。完整 bearing-box 前端接入前，飞行仅验证状态通道、数值稳定性和
PX4 控制接口，不能宣称视觉闭环或论文可观测性验收完成。