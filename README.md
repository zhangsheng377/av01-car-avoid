# 小学生 Arduino 超声波避障小车

> 一年级小学生动手项目：Arduino Uno 超声波避障小车。
> 采用**极坐标扇形直方图（VFH）避障** + **点云轮廓匹配的航向闭环校准**。

本仓库是对应真实机器（真机）调试的小车固件源码与设计文档。
项目在 `\nas\张腾予\小学\一年级\一年级上\小车\超声波避障小车\` 下迭代，
此仓库为该目录的版本化备份，便于云端保存、版本管理与协作。

## 目录结构

```
.
├── README.md                          # 本说明
├── 航向闭环校准设计方案.md             # 架构级设计：点云轮廓匹配航向闭环校准
└── sketch_sep26a/                     # Arduino sketch 目录（目录名必须与 .ino 一致）
    ├── sketch_sep26a.ino              # 主程序（完整小车固件）
    ├── 编译与调参说明.md               # 编译验证、内存占用、参数调优
    └── sketch_sep26a_20260926.bak     # 历史备份
```

## 功能概览

- **极坐标 VFH 避障**：超声波舵机扫描 9 档（rel −80°~+80°）写入 12 个 15° 物理扇区，
  以 `sector[]` 直方图表达周围占用，`secOffset` 表示车头朝向。
- **分层状态机**：SCAN / FORWARD / DECEL / TURN / VERIFY / CAPREF / CAPNEW / DISENGAGE / HALT。
- **航向闭环校准**：转向后停车→快速扫一圈干净轮廓→与转前参考轮廓做相关匹配
  （对 `secOffset` 滑动 0..11 求最小误差）→用实测转角 `bestShift` 重建 `secOffset`，防漂移。
- **三类卡死统一脱离**：
  - B1 前方空却转向（决策层修复）
  - B2 全方向堵死 + 舵机盲区（兜底脱离 + 盲区排除）
  - B3 物理卡住转不动（`turnStep!=0 && reliable && bestShift==0` → 持续倒车脱离）

## 引脚接线

| 功能 | 引脚 |
|---|---|
| 超声波 Trig | 12 |
| 超声波 Echo | 13 |
| 舵机 | A0（`attach(A0,700,2400)`，10~170°） |
| 电机 ENA / ENB | 5 / 6 |
| 电机 IN1 / IN2 / IN3 / IN4 | 3 / 4 / 2 / 7 |

## 如何编译

环境：Arduino Uno（`arduino:avr:uno`）+ `Servo@1.3.0` 库，使用 `arduino-cli`。

> 注意：sketch 目录名必须与 `.ino` 文件名一致，且**避免在 UNC 网络路径下编译**，
> 请先复制 sketch 目录到本地临时路径再编译。

```powershell
# 1) 复制 sketch 目录到本地临时路径
Copy-Item -Recurse "\\nas\张腾予\小学\一年级\一年级上\小车\超声波避障小车\sketch_sep26a" "$env:TEMP\sketch_sep26a"

# 2) 编译
& "C:\Users\43587\arduino-cli\arduino-cli.exe" compile `
  --fqbn arduino:avr:uno `
  --build-path "$env:TEMP\car_build2" `
  "$env:TEMP\sketch_sep26a"
```

编译结果（exit code 0）：程序 4472 B（13% Flash），全局变量 **122 B**（5% SRAM，远低于预算）。

## 参数调优

常用可调参数见《编译与调参说明.md》§6，关键参数：

| 参数 | 初值 | 说明 |
|---|---|---|
| `WALL_STOP` | 12 | 护栏急停距离 cm |
| `TURN_DIST` | 25 | 接近→停车转向 cm |
| `SLOW_DIST` | 40 | 减速门限 cm |
| `FREESPACE` | 20 | 转向后验证可通行距离 cm |
| `MAXTURN` | 2 | 连续转向上限，超限进脱离 |
| `DISENGAGE_MS` | 600 | 脱离模式倒车时长 ms |
| `DISENGAGE_MAX` | 3 | 脱离触发上限，超限倒车加倍 |
| `MATCH_MIN_PAIRS` | 5 | 轮廓匹配有效重叠对下限 |
| `MATCH_MAX_PAIR_DIFF` | 3 | 每对平均允许差 cm |
| `MATCH_MARGIN` | 2 | 最佳 vs 次佳误差差（峰尖锐度） |

## 真机实测已知问题（待继续调优）

真机暴露两个缺陷，由团队继续修复：

1. **卡死后仍无脱困（后退）动作**：需复查 `enterDisengage()` 触发条件，
   特别是轮廓匹配 B3 检测（`turnStep!=0 && reliable && bestShift==0`）在真机为何不生效，
   修复使卡死必退。
2. **探测距离太近、快撞上才转弯**：`TURN_DIST=25 / SLOW_DIST=40 / WALL_STOP=12` 偏近，
   且探测量程 / 舵机扫描角 / 正前方扇区更新可能不足，需增大探测与转向距离阈值、修正盲区，
   使车提前转弯不撞墙。

对应修复后的代码与调参说明会随迭代更新至本仓库。