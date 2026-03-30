# PID 整定工具

当前工程提供了基于辨识模型的离线 PID 整定工具：

- [tools/pid_tune.py](../tools/pid_tune.py)

它读取 [tools/sysid_identify.py](../tools/sysid_identify.py) 生成的 `summary.json`，然后：

1. 选择一个模型
2. 按固件当前的速度环 PID 形式仿真闭环
3. 自动搜索三组风格不同的 `kp/ki/kd`
4. 输出参数、指标和闭环响应图

当前默认会生成：

- `conservative`
- `balanced`
- `aggressive`

三组候选参数。

默认推荐风格是：

- `conservative`

## 固件一致性

这个脚本按当前固件里的真实 PID 形式建模：

- 位置式 PID
- `error = ref - feedback`
- `integral += error * dt`
- `derivative = (error - previous_error) / dt`
- 积分限幅
- 输出限幅

也就是说，它和当前固件中的 [pid.c](../modules/algorithm/pid.c) 是一致的。

## 输入

输入是系统辨识输出的：

- `summary.json`

该文件里会包含：

- `recommended_model`
- `dt_s`
- 每个候选模型的参数

## 默认整定目标

当前脚本默认仿真一个简单的速度阶跃：

- 预留 `0.1 s` 零输入
- 阶跃到 `10 rad/s`
- 保持 `0.8 s`
- 再回到 `0`
- 观察 `0.6 s`

默认限幅和固件保持一致：

- `integral_limit = 3000`
- `output_limit = 10000`

## 用法

### 1. 使用推荐模型

```bash
.venv/bin/python tools/pid_tune.py /tmp/sysid_identify/summary.json
```

### 2. 手动指定模型

```bash
.venv/bin/python tools/pid_tune.py /tmp/sysid_identify/summary.json --model first_order_delay
```

### 3. 调整阶跃幅值

```bash
.venv/bin/python tools/pid_tune.py /tmp/sysid_identify/summary.json --step-radps 12
```

### 4. 调整积分限幅和输出限幅

```bash
.venv/bin/python tools/pid_tune.py /tmp/sysid_identify/summary.json --integral-limit 3000 --output-limit 10000
```

### 5. 指定输出目录

```bash
.venv/bin/python tools/pid_tune.py /tmp/sysid_identify/summary.json -o /tmp/pid_tune
```

### 6. 只生成部分风格

```bash
.venv/bin/python tools/pid_tune.py /tmp/sysid_identify/summary.json --styles conservative balanced
```

## 输出内容

脚本会生成：

- `pid_summary.json`
  - 三组候选 `kp/ki/kd`
  - 推荐风格
  - 用到的模型参数
  - 主要闭环指标
- `pid_step_response.png`
  - 参考速度
  - 三组候选闭环响应
  - 三组候选控制输出

## 当前优化方式

脚本内部使用：

- `scipy.optimize.differential_evolution`

来搜索 `kp/ki/kd`。

搜索目标不是单一指标，而是一个综合代价函数，主要考虑：

- `IAE`
- `ITAE`
- 超调
- 稳态误差
- 回零阶段残留
- 饱和比例
- 控制输出变化幅度

不同风格的代价函数权重不同：

- `conservative`
  - 更重罚超调、饱和、抖动、回零残留
- `balanced`
  - 取折中
- `aggressive`
  - 更允许快一点的响应

所以它给出的不是理论唯一最优解，而是三组工程风格不同的候选初值。

## 如何使用整定结果

得到推荐参数后，建议这样用：

1. 先从 `conservative` 开始
2. 把对应的 `kp/ki/kd` 写到调试器全局变量里
3. 先做单轮或架空验证
4. 再上整车验证 `vx / vy / wz`
5. 如果车很稳但偏肉，再试 `balanced`
6. 如果还想更激进，再看 `aggressive`

也就是说：

- 这三个输出本质上都是 **初值**
- 默认推荐 `conservative`
- 不是“无需实车验证的最终答案”
