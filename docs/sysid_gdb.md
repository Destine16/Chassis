# 单轮 PRBS 系统辨识与 GDB 导出

这份文档描述当前底盘工程里单轮 `PRBS` 系统辨识模式的使用方式，以及如何通过 `gdb` 直接从板上 RAM 导出实验数据。

适用前提：
- 当前工程已编译并烧录成功
- 使用 `J-Link GDB Server` 或兼容的 `gdb remote` 链路
- 轮子已架空
- 建议遥控器保持 `SAFE`

## 目标

当前辨识对象是：

```text
u_raw -> wheel_speed_radps
```

也就是：
- 输入：发给单个轮子的原始电流命令 `u_raw`
- 输出：该轮实际输出轴角速度 `wheel_speed_radps`

辨识模式下会绕过正常底盘闭环控制，只激励一个轮子，其他三个轮子保持 `0`。

## 默认 PRBS 参数

- 幅值：`+2500 / -2500`
- bit 时间：`40 ms`
- bit 数：`127`
- 总时长：约 `5.08 s`
- 采样频率：`1000 Hz`

## 数据缓冲

实验数据写入 `CCMRAM`：
- 缓冲区符号：`g_sysid_buffer`
- 元数据符号：`g_sysid_meta`

样本结构：

```c
typedef struct {
    int16_t u_raw;
    int16_t reserved;
    float   speed_radps;
} sysid_sample_t;
```

说明：
- `u_raw` 记录的是“上一拍已经实际发出的电流命令”
- `speed_radps` 记录的是同一条样本对应拍的轮速反馈
- `reserved` 当前未使用，只做显式对齐

元数据结构：

```c
typedef struct {
    uint32_t sample_hz;
    uint32_t sample_count;
    uint32_t capacity;
    uint32_t wheel_id;
    int32_t  amplitude_raw;
    uint32_t bit_period_ms;
    uint32_t total_bits;
} sysid_meta_t;
```

## 调试器控制变量

以下全局变量可直接在 `gdb` 或 Ozone 中修改：

- `g_sysid_arm`
- `g_sysid_running`
- `g_sysid_hold_zero`
- `g_sysid_wheel_id`
- `g_sysid_amplitude_raw`
- `g_sysid_bit_period_ms`
- `g_sysid_total_bits`

语义：
- `g_sysid_arm = 1`：启动一次实验
- `g_sysid_running = 1`：实验正在进行
- `g_sysid_hold_zero = 1`：实验结束后继续保持四轮零输出
- `g_sysid_hold_zero = 0`：释放零输出保持，恢复正常底盘控制

轮子编号：
- `0 = front_left`
- `1 = front_right`
- `2 = rear_left`
- `3 = rear_right`

## GDB 使用流程

下面示例假设：
- 当前工作目录在工程根目录
- ELF 文件为 `build/Debug/Chassis.elf`
- `J-Link GDB Server` 运行在 `localhost:2331`

### 1. 连接与烧录

```gdb
file build/Debug/Chassis.elf
target extended-remote localhost:2331

monitor halt
monitor reset halt
load
```

### 2. 配置一次实验

下面以左前轮为例：

```gdb
set var g_sysid_wheel_id = 0
set var g_sysid_amplitude_raw = 2500
set var g_sysid_bit_period_ms = 40
set var g_sysid_total_bits = 127
set var g_sysid_hold_zero = 0
set var g_sysid_arm = 1
continue
```

### 3. 判断实验是否结束

实验跑完后：

```gdb
monitor halt
p g_sysid_running
p g_sysid_hold_zero
p g_sysid_meta
```

正常情况下：
- `g_sysid_running = 0`
- `g_sysid_hold_zero = 1`
- `g_sysid_meta.sample_count` 大约为 `5080`

### 4. 导出数据

```gdb
set $n  = g_sysid_meta.sample_count
set $sz = sizeof(g_sysid_buffer[0])

dump binary memory /tmp/sysid_meta.bin &g_sysid_meta ((char*)&g_sysid_meta) + sizeof(g_sysid_meta)
dump binary memory /tmp/sysid_buf.bin  &g_sysid_buffer[0] ((char*)&g_sysid_buffer[0]) + ($n * $sz)
```

导出后：
- `/tmp/sysid_meta.bin`
- `/tmp/sysid_buf.bin`

就是这次实验的原始数据。

### 5. 恢复正常底盘控制

导出完以后，如果要恢复正常控制：

```gdb
set var g_sysid_hold_zero = 0
continue
```

## 转成 CSV

工程里已经带了一个本地解析脚本：

- [tools/sysid_dump_to_csv.py](../tools/sysid_dump_to_csv.py)

使用方式：

```bash
python3 tools/sysid_dump_to_csv.py /tmp/sysid_meta.bin /tmp/sysid_buf.bin -o /tmp/sysid.csv
```

脚本会：
- 读取 `g_sysid_meta`
- 按 `sample_hz` 还原时间轴
- 输出 `sample_idx,t_s,u_raw,speed_radps`
- 在 CSV 头部附带一段元数据

## 时间轴恢复

当前不单独存每条样本时间戳。

离线处理时直接用：

```text
t = sample_idx / sample_hz
```

当前默认：

```text
sample_hz = 1000
```

所以：

```text
t = sample_idx / 1000.0
```

## 推荐操作习惯

- 先确认遥控器在 `SAFE`
- 先做一轮默认参数实验
- 导出后先检查：
  - `sample_count`
  - `u_raw` 是否确实在 `+2500/-2500` 切换
  - `speed_radps` 是否响应正常
- 一切正常后，再去改：
  - `wheel_id`
  - `amplitude_raw`
  - `bit_period_ms`
  - `total_bits`

## 备注

- 如果以后需要更长实验时间，优先改 `APP_CFG_SYSID_SAMPLE_CAPACITY`
- 如果以后需要更长记录时间但 RAM 紧张，可考虑把 `speed_radps` 改成定点存储
- 这份文档适合人手操作，也适合让 `Codex CLI` 直接按步骤执行
