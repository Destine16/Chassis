# 系统辨识工具

当前工程提供了一套最小可用的离线系统辨识工具链：

1. 板上运行单轮 `PRBS` 实验并把数据记录到 RAM
2. 用 `gdb` dump 导出 `meta` 和 `buffer`
3. 用 [tools/sysid_dump_to_csv.py](../tools/sysid_dump_to_csv.py) 转成 `csv`
4. 用 [tools/sysid_identify.py](../tools/sysid_identify.py) 拟合低阶模型、出图、计算拟合度

## 当前支持的模型

当前脚本会比较以下候选模型：

- `first_order`
  - 一阶模型
- `first_order_delay`
  - 一阶模型 + 纯滞后
- `second_order_delay`
  - 二阶串联惯性模型 + 纯滞后

当前这三类模型都通过 `scipy.optimize.least_squares` 拟合。
其中二阶模型因为参数更多，运行时间会明显长于一阶模型。

推荐顺序仍然是：

1. 先看一阶
2. 如果明显不够，再看一阶 + 滞后
3. 还不够时再看二阶

## 依赖

当前脚本只依赖：

- `.venv/bin/python`
- `numpy`
- `matplotlib`

当前项目内虚拟环境 `.venv` 已经安装：

- `numpy`
- `scipy`
- `matplotlib`
- `pandas`
- `control`
- `statsmodels`
- `sympy`

所以后续命令统一建议使用 `.venv/bin/python`。

## 输入数据

脚本读取的是 [tools/sysid_dump_to_csv.py](../tools/sysid_dump_to_csv.py) 导出的 `csv`。

数据列为：

- `sample_idx`
- `t_s`
- `u_raw`
- `speed_radps`

其中：

- `u_raw`
  - 实际发给 `C620` 的原始电流命令
- `speed_radps`
  - 当前辨识轮子的实际输出轴角速度

## 推荐实验方式

最推荐准备两组独立实验：

- 第一组：用于辨识
- 第二组：用于验证

如果暂时只有一组数据，也可以先只跑训练集，但那样不能真正验证模型泛化能力。

## 用法

### 1. 只有一组数据

```bash
.venv/bin/python tools/sysid_identify.py /tmp/sysid.csv
```

### 2. 有训练集和验证集

```bash
.venv/bin/python tools/sysid_identify.py /tmp/sysid_train.csv -v /tmp/sysid_validate.csv
```

### 3. 指定输出目录

```bash
.venv/bin/python tools/sysid_identify.py /tmp/sysid_train.csv -v /tmp/sysid_validate.csv -o /tmp/sysid_identify
```

### 4. 调整最大滞后搜索范围

```bash
.venv/bin/python tools/sysid_identify.py /tmp/sysid_train.csv -v /tmp/sysid_validate.csv --max-delay-ms 160
```

## 输出内容

脚本会在输出目录中生成：

- `summary.json`
  - 各个候选模型的参数和拟合指标
- `train_comparison.png`
  - 训练集的实际输出与模型输出对比图
- `validate_comparison.png`
  - 验证集的实际输出与模型输出对比图
- `best_model_residuals.png`
  - 推荐模型的残差图

终端还会打印：

- 每个模型的训练集拟合度
- 每个模型的验证集拟合度
- 推荐模型

## 拟合指标

当前脚本会计算：

- `RMSE`
- `MAE`
- `R²`
- `fit %`

其中 `fit %` 定义为：

```text
fit% = 100 * (1 - ||y - y_hat|| / ||y - mean(y)||)
```

这个值越高越好。

## 推荐模型策略

脚本默认按以下逻辑选推荐模型：

1. 优先看验证集 `fit %`
2. 如果没有验证集，就看训练集 `fit %`
3. 若两个模型拟合度非常接近（差值不超过约 `1%`），优先选更简单的模型

也就是说：

- 不会无条件偏向更高阶模型
- 更强调“简单但足够好”

## 结果怎么看

如果一阶模型已经能做到：

- 曲线基本跟上
- 残差没有明显结构
- 验证集拟合度也还不错

那通常就没必要继续升阶。

如果出现这些情况，再考虑升到更复杂的模型：

- 一阶明显跟不上
- 延迟感很强
- 残差有明显规律
- 验证集上二阶明显优于一阶

## 后续用途

当前工具的直接输出是“辨识模型”，不是 PID 参数。

后续可以继续基于 `summary.json`：

- 做离线仿真
- 设计 PID 初值
- 扫描 `kp/ki/kd`
- 选择一组更稳妥的候选参数
