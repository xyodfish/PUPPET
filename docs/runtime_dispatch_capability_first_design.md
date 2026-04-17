# Runtime Dispatch 设计（Capability-First）

## 1. 目标
本设计面向通用 `PrimitiveFrame -> ControlIntent` 编排，不绑定具体场景（主从臂只是其中一种）。

关键目标：
1. 先识别 `group`，再识别控制语义（笛卡尔/关节/增量/速度）。
2. 再从可用 pipeline 插件中选择执行实现。
3. `whole_body` 与 `single_chain` 等能力不匹配场景必须硬约束拦截，不允许仅靠优先级“误选”。
4. 支持多 device 与同 device 多控制方式并存。

---

## 2. 四阶段调度模型

单周期调度按以下顺序执行：

1. **Group Detect**  
从 `PrimitiveFrame`（`meta.body_group/entity/tags/context`）判定候选 group。

2. **Semantics Detect**  
对每个 group 判定控制语义：
- `joint_absolute`
- `joint_delta`
- `cartesian_absolute`
- `cartesian_delta`
- `cartesian_velocity`（或 `cartesian_rate`）
- `velocity`

3. **Capability Filter（硬约束）**  
对每个 pipeline 进行能力匹配检查（必须通过）：
- `supported_groups`
- `supported_semantics`
- `required_inputs`
- `scope`（`whole_body` / `sub_group`）

4. **Priority Select（软策略）**  
在通过硬约束的候选中按优先级与健康度选赢家。

> 规则：**Hard Constraint > Priority**。  
能力不匹配直接淘汰，优先级再高也不能被选。

---

## 3. 核心约束（你强调的点）

### 3.1 whole_body 与 single_chain 的关系
- `single_chain_ik` 仅支持 `sub_group`（例如 `left_arm/right_arm`）。
- `whole_body` 请求下，`single_chain_ik` 必须在 Capability Filter 阶段被拒绝。
- `whole_body` 只能从 `whole_body` scope 插件中选（如 `gmr`、后续 `wbc_ik`）。

### 3.2 scope 约束
- `scope=whole_body`：覆盖全身耦合目标。
- `scope=sub_group`：仅覆盖局部链/局部组。
- 默认策略：`whole_body_exclusive=true`  
  whole_body 赢家存在时，屏蔽子组赢家（避免冲突控制）。

### 3.3 fallback 约束
- whole_body 失败回退必须仍在 whole_body scope 内。
- sub_group 失败回退必须在 sub_group scope 内。
- 禁止跨 scope 降级（例如 whole_body 失败直接降到 single_chain）。

---

## 4. Pipeline 能力声明（Registry）

每个插件注册时声明自身能力：

```yaml
pipeline_registry:
  - pipeline_id: gmr_pipeline
    plugin: gmr
    enabled: true
    priority: 120
    capability:
      scope: whole_body
      supported_groups: [whole_body]
      supported_semantics: [cartesian_absolute, cartesian_delta]
      required_inputs: [poses]

  - pipeline_id: single_chain_ik_pipeline
    plugin: single_chain_ik
    enabled: true
    priority: 200
    capability:
      scope: sub_group
      supported_groups: [left_arm, right_arm]
      supported_semantics: [cartesian_absolute, cartesian_delta]
      required_inputs: [poses, ee_entity]
```

---

## 5. Group 与语义分发策略

建议新增 dispatch 配置（概念示例）：

```yaml
dispatch_policy:
  whole_body_exclusive: true

group_policies:
  - body_group: whole_body
    owner_policy: fixed
    owner_candidates: [mocap_fullbody]
    semantics_candidates: [cartesian_absolute]
    candidate_pipelines: [gmr_pipeline, wbc_ik_pipeline]

  - body_group: right_arm
    owner_policy: hybrid
    owner_candidates: [master_arm, vr_main]
    semantics_candidates: [joint_absolute, joint_delta, cartesian_delta]
    candidate_pipelines: [single_chain_ik_pipeline, newton_chain_ik_pipeline, direct_pass_pipeline]
```

---

## 6. 多 device 与同 device 多模式统一处理

### 6.1 多 device
- `owner_policy` 选择每个 group 的当前主 source（fixed/priority/freshness/hybrid）。
- 选中 owner 后再进入语义判定与 pipeline 选择。

### 6.2 同 device 多模式
- 同一个 source 对同 group 可切换语义（关节直通/关节增量/笛卡尔增量/速度）。
- 通过 `Semantics Detect` 判定当前语义，不在插件内部硬编码场景。
- 语义确定后进入 Capability Filter，自动映射可执行 pipeline。

### 6.3 笛卡尔速度输入（你补充的场景）
- 典型例子：轮腿机器人通过手柄给“腿部高度目标速度”（`dz/dt`）。
- 该输入不应直接当作 `cartesian_delta`，应走 `cartesian_velocity` 语义。

标准处理链建议：
1. 读取机器人当前状态（例如当前足端/机身高度，来自状态估计或反馈）。
2. 将速度指令按周期积分为目标位姿：
   - `x_target = x_current + v_cmd * dt`
3. 施加限幅/边界（高度上下限、最大步长、加速度限）。
4. 形成 `cartesian_absolute` 目标。
5. 再交给 IK 类插件（`single_chain_ik` / `wbc_ik` / `newton_ik`）求解关节命令。

实现上建议新增中间层：
- `StatefulTargetGenerator`（有状态目标生成器）
  - 输入：`cartesian_velocity`
  - 状态：`x_current / x_target_last`
  - 输出：`cartesian_absolute`

这样可以把“速度积分与约束”从 IK 插件剥离，保持 IK 插件只做求解。

---

## 7. 冲突与稳定性策略

1. 同 group 单赢家：一个周期只允许一个 WinnerPlan 写入。  
2. 并列优先级 tie-break：
   - 新鲜度
   - 近期成功率
   - 固定字典序（可复现）
3. 切换去抖：
   - `min_dwell_ms`
   - 模式切换守卫（例如速度阈值）
4. 状态回退：
   - `last_good_plan` 短时保持，避免抖动。

---

## 8. 与现有代码的落地建议（增量）

### Phase 1（最小可用）
1. pipeline 增加 capability 元信息（先支持 `scope/supported_groups/supported_semantics`）。
2. 在 `RetargetingPipeline` 前加入 Capability Filter。
3. 引入 `whole_body_exclusive` 与 scope 内 fallback 规则。

### Phase 2
1. 增加 `Semantics Detect` 独立模块。
2. 增加 `owner_policy=hybrid` 与 source 打分。
3. 增加 `StatefulTargetGenerator`，支持 `cartesian_velocity -> cartesian_absolute`。

### Phase 3
1. 引入 pipeline 健康度与自动降级图。
2. 完善运行时观测（每组 winner、filter 原因、回退次数）。

---

## 9. 预期收益
1. 防止 whole_body 错误落到 single_chain（硬约束保证）。  
2. 主从臂/VR/mocap 统一到同一调度框架，不再场景特化。  
3. 新增插件（wbc_ik/newton_ik）只需注册能力，无需改主流程分支。  
4. 决策可解释（每次淘汰与胜出都有明确原因）。
