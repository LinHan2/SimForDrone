# Yaw-Invariant Tilt-Constrained Target Motion Estimator

## 1. 问题定义与贡献边界

考虑非合作目标无人机 $T$ 与观测无人机 $O$。所有状态在共享 ENU 世界系 $W$ 中表示，

$$
g^W=(0,0,-9.81)^T\ \mathrm{m/s^2}.
$$

类别级无人机姿态网络常因机体绕自身 $z$ 轴的外观对称出现大 yaw 歧义。该歧义会损害完整 $SO(3)$
姿态评价，却不必损害目标平移动力学估计。本方法的贡献不是以 tilt 凭空创造新观测信息，而是：

1. 将感知到估计的姿态表示由 $SO(3)$ 降为与平移动力学相关的商空间 $SO(3)/SO(2)\simeq S^2$；
2. 用 $S^2$ 切空间的两个独立坐标表达已有的 rank-$2$ 推力轴约束；
3. 将网络 tilt 不确定度以正定 $2\times2$ 协方差传播到 Kalman 量测噪声；
4. 通过常加速度状态传播的交叉协方差，使 tilt 对加速度的约束改善未来的速度和位置预测。

因此，本文不能声称二维表示比 Zhang 的 $P_h$ 约束具有更大的物理可观子空间；两者等价。改进是
**最小表示、无冗余量测坐标、yaw 对称不变接口和可校准的二维不确定度建模**。

## 2. 非合作多旋翼动力学与 yaw 不变表示

目标的平移动力学写为

$$
a_t=g^W+\lambda h,\qquad \lambda=f_t/m_t,\qquad h\in S^2,
$$

其中 $m_t$、总推力 $f_t$ 及其比值 $\lambda$ 对非合作目标均未知。当前 ENU/FLU 实现中，推力轴为

$$
h=R_te_3,\qquad e_3=(0,0,1)^T.
$$

若网络产生 body-yaw 对称解 $R'_t=R_tR_z(\gamma)$，则

$$
h(R'_t)=R_tR_z(\gamma)e_3=R_te_3=h(R_t).
$$

这对任意 $\gamma$ 严格成立。这里的 yaw 指绕 **目标自身 body-$z$ 轴** 的对称变换，不是任意
世界系 Euler yaw 的变化。因此估计器只消费 $h$，完整 $R_t$ 仍可保留给 6D pose benchmark。

进一步令右乘小旋转误差为 $\hat R=R\exp(\delta\theta^\wedge)$，有

$$
\delta h\approx -R[e_3]_\times\delta\theta.
$$

因为 $[e_3]_\times e_3=0$，纯 body-yaw 误差位于该 Jacobian 的零空间。全姿态可以存在 $90^\circ$
或 $180^\circ$ 对称误差，同时动态相关的 $h$ 仍准确。

## 3. 从投影矩阵到二维切空间量测

定义投影矩阵 $P_h=I_3-hh^T$。由 $P_hh=0$，未知推力幅值被消去：

$$
P_h(a_t-g^W)=0.
$$

$P_h$ 的特征值为 $(1,1,0)$，因此其秩为 $2$。这不是数值缺陷：沿 $h$ 的加速度大小
$h^T(a_t-g^W)=\lambda$ 未知，物理上只能获得垂直于 $h$ 的两个独立约束。

令 $B(h)=[b_1\ b_2]\in\mathbb R^{3\times2}$ 为 $T_hS^2$ 的正交基，满足

$$
B^TB=I_2,\qquad B^Th=0,\qquad BB^T=P_h.
$$

实现选择与 $h$ 最不平行的世界坐标轴 $c=\arg\min_{e_i}|h^Te_i|$，然后计算

$$
b_1=\frac{c-h(h^Tc)}{\|c-h(h^Tc)\|},\qquad b_2=h\times b_1.
$$

于是 $P_h(a_t-g^W)=0$ 等价于最小形式

$$
B(h)^T a_t=B(h)^Tg^W.
$$

$B^T\in\mathbb R^{2\times3}$ 是满行秩，而不是“满 rank 3”。它删除的是一个冗余 measurement
coordinate，不增加单帧可观测信息。

## 4. 双目距离对 bearing-box 尺度的约束

双目相机能提供目标参考点相对相机的 metric 距离 $\rho$，但其深度精度通常随距离、纹理和视差退化。
因此不应直接写 $\alpha=\rho$：bearing-box 的 $\bar T_w$ 是由框几何导出的归一化/无尺度向量，未必是
单位向量，故 $\alpha$ 不一定等于 Euclidean range。

保留原状态 $x=[\delta p,\delta v,a_t,\alpha]$，新增独立的非线性立体 range 量测：

$$
z_\rho=\|\delta p\|+\nu_\rho,\qquad \nu_\rho\sim\mathcal N(0,\sigma_\rho^2).
$$

在当前预测 $\hat{\delta p}^-$ 处一阶线性化，令 $\hat\rho^-=\|\hat{\delta p}^-\|$，则

$$
H_\rho=
\begin{bmatrix}
(\hat{\delta p}^-/\hat\rho^-)^T&0_{1\times3}&0_{1\times3}&0
\end{bmatrix},
\qquad
r_\rho=z_\rho-\hat\rho^-.
$$

range 量测直接校正的是 $\delta p$ 的径向分量；它不会直接测量 $\alpha$。但 bearing-box 更新已建立
$\delta p$ 与 $\alpha$ 的交叉协方差，因此 range 更新会间接减少尺度不确定性。这比用不可靠深度硬覆盖
$\alpha$ 更适合双目误差随环境变化的实际情况。

当前实现的 `stereo_range_m` 和 `stereo_range_std` 均为可选项，默认关闭。双目前端可按每帧有效深度
提供距离；无效视差、遮挡或过远目标应令 `stereo_range_m=None`，只执行 bearing-box 与 tilt 更新。

## 5. 十维相对 EKF 与顺序量测更新

滤波状态严格沿用 Zhang bearing-box 模型：

$$
x=[\delta p^T,\ \delta v^T,\ a_t^T,\ \alpha]^T\in\mathbb R^{10},
\qquad \delta p=p_t-p_o,\quad \delta v=v_t-v_o.
$$

以观测机世界加速度 $u_k=a_{o,k}$ 为已知输入，常加速度预测为

$$
x_{k+1}=F_kx_k+G_ku_k+w_k,
$$

$$
F_k=
\begin{bmatrix}
I_3&\Delta tI_3&\tfrac12\Delta t^2I_3&0\\
0&I_3&\Delta tI_3&0\\
0&0&I_3&0\\
0&0&0&1
\end{bmatrix},
\qquad
G_k=
\begin{bmatrix}
-\tfrac12\Delta t^2I_3\\
-\Delta tI_3\\
0\\0
\end{bmatrix}.
$$

每帧按两个独立的量测源顺序更新；在线性、高斯且两类噪声独立时，这与堆叠联合更新等价，但更便于
诊断和单源失效处理。

**Bearing-box 更新。** 完整前端应由八角点和 Lemma 1 生成 $\bar T_w$：

$$
z_{bb}=0_3,\qquad
H_{bb}=[I_3\ \ 0_{3\times3}\ \ 0_{3\times3}\ \ -\bar T_w].
$$

它约束 $\delta p-\alpha\bar T_w=0$，主要直接更新相对位置和尺度。

**双目 range 更新。** 当该帧有可信的 $z_\rho$ 时，在 bearing-box 更新后、tilt 更新前执行上述
$H_\rho$ 一维 EKF 更新。该顺序让 range 通过 $\\delta p$--$\alpha$ 交叉协方差校正尺度；若 range 缺失，
该步骤完全跳过。

**Tilt 更新。** 对网络输出的单位推力轴 $\hat h$ 构造 $B=B(\hat h)$：

$$
z_h=B^Tg^W,\qquad
H_h=[0_{2\times3}\ \ 0_{2\times3}\ \ B^T\ \ 0_{2\times1}].
$$

其残差为

$$
r_h=B^T(g^W-\hat a_t).
$$

该量测直接约束 $a_t$；预测中的 $p$--$a$ 和 $v$--$a$ 交叉协方差会通过 Kalman gain 间接修正
$\delta p$、$\delta v$，并可能在长期通过 bearing-box 的交叉协方差影响 $\alpha$。协方差更新使用
Joseph form 与 PSD 投影。

## 6. Tilt 量测协方差 $R_h$

### 当前默认：经验固定加速度约束噪声

当前 `RelativeTargetEKF` 的默认值为

$$
\sigma_{\mathrm{constraint}}=0.03\ \mathrm{m/s^2},\qquad
R_h=\sigma_{\mathrm{constraint}}^2I_2=0.0009I_2\ \mathrm{m^2/s^4}.
$$

该量是 **$B^T(a_t-g)$ 残差的经验标准差**，来自 Airsim2box 已调参数；它不是网络 tilt 的角度标准差。
默认路径保持这个标定，不依赖网络是否能输出置信度。

### 可选方向噪声传播

当构造滤波器时显式提供 `tilt_direction_std=\sigma_{\mathrm{tilt}}`，代码启用各向同性的切空间方向
不确定度传播。把 $\sigma_{\mathrm{tilt}}$ 视为小角度量级（rad，无量纲），则

$$
R_h=\max(\|\hat a_t-g^W\|,10^{-6})^2\sigma_{\mathrm{tilt}}^2I_2.
$$

实现发生在 bearing-box 更新之后、tilt 更新之前，因此 $\hat a_t$ 是该时刻当前状态，而非保存的
独立 prediction snapshot。下限 $10^{-6}\ \mathrm{m/s^2}$ 仅避免零矩阵，并不增加物理信息。

未来网络若输出经标定的非各向同性切空间协方差 $\Sigma_\xi\succ0$，应替换为

$$
R_h=\|\hat a_t-g^W\|^2\Sigma_\xi.
$$

在没有校准过的 $\Sigma_\xi$ 前，不应把该公式用于飞行参数。

### 数值例子

以下例子均为 $R_h$，单位是 $\mathrm{m^2/s^4}$；矩阵的平方根才具有 $\mathrm{m/s^2}$ 单位。

| 场景 | 参数 | 计算 | $R_h$ |
| --- | --- | --- | --- |
| 默认标定 | $\sigma_{\mathrm{constraint}}=0.03\ \mathrm{m/s^2}$ | $0.03^2$ | $0.0009I_2$ |
| 悬停、方向噪声模式 | $\hat a_t=0$, $\sigma_{\mathrm{tilt}}=0.02\ \mathrm{rad}$ | $(9.81\times0.02)^2$ | $0.03849444I_2$ |
| $2\ \mathrm{m/s^2}$ 横向机动、方向噪声模式 | $\hat a_t=(2,0,0)^T$, $\sigma_{\mathrm{tilt}}=0.02\ \mathrm{rad}$ | $(\sqrt{2^2+9.81^2}\times0.02)^2$ | $0.04009444I_2$ |
| $2\ \mathrm{m/s^2}$ 向上机动、方向噪声模式 | $\hat a_t=(0,0,2)^T$, $\sigma_{\mathrm{tilt}}=0.02\ \mathrm{rad}$ | $(11.81\times0.02)^2$ | $0.05579044I_2$ |

方向噪声模式的数值明显大于默认固定值，**不能据此判断哪一个更优**：两者描述的不是同一标定量。
启用该模式前必须通过带标注的 tilt 误差或闭环残差重新校准 $\sigma_{\mathrm{tilt}}$。

### 当前离线调参结果

使用 `tracking.evaluate_relative_ekf` 的确定性恒距圆周场景：半径 $6\ \mathrm{m}$、角速度
$0.35\ \mathrm{rad/s}$、相对位置代理噪声 $0.05\ \mathrm{m}$、时长 $30\ \mathrm{s}$。该场景让
$\delta p=\alpha\bar T$ 严格成立，并以 $h=\operatorname{normalize}(a_t-g^W)$ 构造物理一致推力轴；
它直接以仿真真值位置、速度、加速度和姿态生成 observer 输入，再按指定标准差注入受控扰动。因此当前
阶段验证 observer，不验证真实 3D box 前端、图像检测、网络姿态或控制器。

零噪声真值位姿基准下，默认固定 $R_h$ 的 RMSE 为：位置 $0.089\ \mathrm{m}$、速度
$0.030\ \mathrm{m/s}$、加速度 $0.0028\ \mathrm{m/s^2}$。把 tilt 约束近似禁用后，误差分别为
$1.259\ \mathrm{m}$、$0.417\ \mathrm{m/s}$、$0.248\ \mathrm{m/s^2}$。该对照验证当前坐标约定、
常加速度传播和二维 tilt 更新能够在理想观测下显著改善状态估计；它不是图像闭环指标。

| 注入 tilt 噪声 | no-tilt 加速度 RMSE | 推荐方向标准差 | 启用 tilt 后加速度 RMSE | 结论 |
| --- | --- | --- | --- | --- |
| $0.5^\circ$ | $0.719\ \mathrm{m/s^2}$ | $0.05\ \mathrm{rad}$ | $0.099\ \mathrm{m/s^2}$ | 启用 |
| $2.0^\circ$ | $0.719\ \mathrm{m/s^2}$ | $0.07\ \mathrm{rad}$ | $0.483\ \mathrm{m/s^2}$ | 启用，但需保守放大 |
| $5.0^\circ$ | $0.719\ \mathrm{m/s^2}$ | $0.07\ \mathrm{rad}$ | $3.327\ \mathrm{m/s^2}$ | 门控/拒绝 tilt |

推荐值大于注入误差，说明当前常加速度模型与 bearing 代理仍存在有效模型失配。因此这些值只能作为
离线 observer 的起点，不能直接写入飞行默认参数。实际感知链路接入后，应按网络每帧置信度门控 tilt：
高置信度帧使用经标定的 $R_h$，低置信度帧跳过 tilt 更新而保留 bearing-box 及预测。

## 7. 可观测性、收敛性与不变性

将 $P_h$ 换成 $B^T$ 不改变可观子空间，因为

$$
\operatorname{Row}(P_h)=\operatorname{Row}(B^T).
$$

有限时间窗口内，完整状态只有在 bearing 变化、时间激励和相对加速度满足条件时才可能恢复。Zhang 的
条件 $P_h(a_t-a_o)\ne0$ 等价于 $B^T(a_t-a_o)\ne0$；单帧 tilt 仍无法确定沿 $h$ 的推力加速度。

对时变系统，若有限窗口观测 Gramian 存在一致正下界，且过程噪声、量测噪声有正定有界上下界，则
线性 Kalman 滤波协方差有界；无噪声误差可指数衰减，有噪声时只能主张均方有界。因为

$$
h(RR_z(\gamma))=h(R),
$$

body-yaw 对称歧义不改变 $H_h$ 所张成的子空间及其可观性秩。不同合法切空间基若满足
$B'=BQ$, $Q\in SO(2)$，也仅改变二维坐标，不改变物理约束。

## 8. 实现状态与验证

- [relative_ekf.py](../tracking/tracking/relative_ekf.py) 使用确定性 $S^2$ 切空间基和二维 tilt 更新；
  默认配置沿用 Airsim2box 标定；`tilt_direction_std` 与 `stereo_range_std` 均为可选 API，默认关闭。
- [test_estimation.py](../tracking/tests/test_estimation.py) 覆盖：$R$ 与 $RR_z(\gamma)$ 的 yaw 不变性、
  $B^TB=I_2$、$B^Th=0$、$BB^T=P_h$、二维更新与冗余三维 $P_h$ 更新的后验等价性，以及 $R_h$ 的
  默认与方向噪声缩放。
- 同一测试还验证双目 range 更新校正相对距离，并经 bearing-box 交叉协方差缩小 $\alpha$ 误差；
  `TargetMeasurement.stereo_range_m` 已可将未来双目距离送入该更新。
- [test_relative_ekf_evaluation.py](../tracking/tests/test_relative_ekf_evaluation.py) 固化低噪声 tilt 的
  可量化收益、零噪声真值位姿基线，以及高噪声 tilt 应被门控而非强制融合的边界。
- 当前共享相对位置仅是 bearing-box 方向的仿真代理。真实视觉闭环仍需要八角点检测、相机内外参与
  Lemma 1 的 $\bar p_c$ 计算；在此之前，实验只能宣称 observer 数值验证，不能宣称完整视觉
  bearing-box 验证。