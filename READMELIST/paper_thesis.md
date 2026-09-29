# 论文中心思想与实验收敛计划

> 状态：规划稿（2026-09-28）。目标不是"做得更多"，而是收敛到一个别人会记住的中心结论。
> 与 [双机跟踪实施规划](uav_visual_tracking_project_plan.md) 的关系：本文件回答
> **"论文要证明什么"**，实施规划回答 **"工程上怎么落地"**。

---

## 1. 定位：从"加法型"转向"新认识"

如果最终论文的结论只是"6D 更准 + 加了 h + EKF 更准 + 跟踪更准"，那它仍是非常完整的
加法型系统工作，更自然地对应 TMECH / TASE / RA-L——这并不差，但不足以冲 T-RO。

Zhang 等人工作的强处不是实验数量，而是一个清晰结论：
3D detection 中原本未被充分利用的信息可以改变目标运动的可观测条件；对 MAV，
姿态—加速度耦合可以解除对 observer 高阶运动的要求。

因此现在最重要的不是"还能加什么实验"，而是**砍掉不能支撑核心科学结论的实验**。

---

## 2. 核心结论（候选）

$$
\boxed{\text{完整 } SO(3) \text{ 姿态精度并不等价于闭环机器人任务价值。}}
$$

对于具有结构对称性的类别级 UAV，真正与短时平移动力学和 pursuit 相关的是

$$
h \in S^2,
$$

而不是完整姿态 $R \in SO(3)$。

由此形成的论证链：

$$
\text{category-level perception}
\;\rightarrow\;
h \text{ uncertainty}
\;\rightarrow\;
\text{motion prediction uncertainty}
\;\rightarrow\;
\text{closed-loop tracking performance}.
$$

若能对这条链**理论上证明、实验上验证**，则不再是 Zhang estimator 的"小修小补"。

---

## 3. 论文问题重述

- 传统问题：How accurately can we estimate 6D pose?
- 本工作的问题：Which components of category-level 6D pose actually matter for a
  dynamic robotic task?
- 答案候选：完整姿态用于几何理解，而 quotient attitude $h$ 才直接决定短时平移运动信息；
  这种 task-relevant representation 在 unseen UAV 上更容易泛化，并真正改善闭环 pursuit。

---

## 4. 三个理论命题

1. **Yaw / symmetry invariance**：
   对具有 $C_4$ 旋转对称的类别级 UAV，$d_{SO(3)}$ 可以很大而短时动力学不变；
   yaw 误差不进入运动预测与跟踪误差的敏感方向。
2. **Finite-horizon information / prediction gain**：
   由 $p(t+\tau)=p+\tau v+\frac12\tau^2 a$，加速度不确定度对未来位置预测协方差的贡献
   近似随 $\tau^4$ 增长；$h$ 约束提供的加速度信息因此对预测有超线性收益。
3. **Estimation error → tracking error bound**：

   $$
   \|e_{\rm track}\|
   \le
   c_p\|\tilde p\| + c_v\|\tilde v\| + c_h \sin\frac{\theta_h}{2},
   $$

   其中 $\theta_{\rm yaw}$ **根本不出现**。真实实验若观察到 $E_R$ 与跟踪性能相关性一般、
   而 $E_h$ 与预测/跟踪误差高度相关，即与该 bound 互相印证。

---

## 5. 四组核心实验（每组只回答一个科学问题）

### Q1：类别级感知真的泛化吗？

- 设置：Seen UAV vs Unseen UAV；对比现有方法与本方法。
- 指标：$R$、$t$、ADD/ADD-S，以及本方法的 $E_h$。
- 要证明的不是 unseen 不掉点，而是 **generalization gap 明显更小**。

### Q2：为什么需要 $h$，而不是只看完整姿态？

- 三个输入通路：$\hat R \rightarrow \hat h_R$、$\hat h_{\rm direct}$、$h_{\rm GT}$。
- 指标：$E_h$，并按高/低对称性、seen/unseen 拆分。
- 理想形态：$E_R^{\rm unseen}$ 大幅上升，而 $E_h^{\rm unseen}$ 上升很少。
- 关键图：$d_{SO(3)}$ 与 downstream motion error 的相关性**弱于** $d_{S^2}$。

### Q3：$h$ 到底有没有改善运动预测？

- 三个 estimator：position-only vs $R\rightarrow h$ vs direct $h$ + uncertainty。
- 指标：$RMSE_a$、$RMSE_v$、$E_{\rm pred}(100/300/500\,\mathrm{ms})$。
- 轨迹只保留四种：straight/CV（负对照，预期无优势）、circle、S-turn、rapid maneuver。
- 预期规律：**机动越强，$h$ 的价值越大**——这个规律比"所有数据集都提升 12.6%"有意义得多。

### Q4：它最终真的帮助无人机追踪了吗？（杀手实验）

- 同一 observer、同一控制器、同一 target trajectory，只换 estimator：
  position-only vs pose-derived $h$ vs ours。
- 指标：tracking RMSE、peak tracking error、maneuver response lag、prediction error。
- 仿真做完整消融；真机不需要所有组合全飞。
- 真机重点证明链路：
  onboard RGB/stereo → network → estimator → tracker → PX4 **可以实时闭环**。

---

## 6. 收敛后的工作量边界

$$
\text{4 组核心实验} + \text{3 个理论命题} + \text{1 个真实双机闭环}
$$

- 每组实验必须明确回答一个 scientific question，否则砍掉。
- 不追求几十张表；一张能表达"机动越强 h 价值越大"或
  "$E_h$ 与跟踪误差高度相关而 $E_R$ 相关性一般"的图，价值高于十个平均提升数字。

---

## 7. 期刊定位预期

- 现有基础（类别级 UAV pose 精度与 unseen 泛化已显著优于现有方法）是一块较硬的
  perception contribution，不是从零拼系统。
- 若 4 组实验 + 3 个命题 + 真机闭环全部成立：具备冲击 T-RO 的"中心思想"。
- 务实预期：即使最终只证明"更好的类别级姿态确实改善目标状态预测与追踪"，
  也是扎实的 TMECH / TASE / RA-L 级工作。

---

## 8. 与当前工程进度的衔接（待办映射）

| 论文实验 | 当前工程状态 | 缺口 |
|---|---|---|
| Q1 泛化 | Drone6D unseen 测试已有结果 | 补 $E_h$ 指标与高/低对称性拆分 |
| Q2 representation | 消融已有 w/ vs w/o global | 需三通路对比与相关性图 |
| Q3 运动预测 | `relative-ekf` 9 维 EKF 已接入影子通路 | 三 estimator 对比 + 机动轨迹集 |
| Q4 闭环 | 真值双机跟踪保守八字已通过 | 视觉前端替换真值量测 + 只换 estimator 的消融 |
| 真机 | `vision/` 未接、相机外参未链路化 | onboard 推理、实时闭环验收 |

详见 [双机跟踪实施规划](uav_visual_tracking_project_plan.md) 与 [项目进度](progress.md)。
