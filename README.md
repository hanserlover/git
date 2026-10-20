# OB_ROBOT 四轮麦轮小车（STM32F103RC）

STM32F103RCT6 + TB6612 双驱动片 + 四路编码器的麦轮底盘工程。
底层已能直行（速度闭环 PI + PWM 前馈），本仓库在闭环基础上补齐了
**前进 / 后退 / 左移 / 右移 / 原地旋转** 的完整运动层。

## 工程结构

```
OB_ROBOT(3)(1)/OB_ROBOT(1)/OB_ROBOT/   ← 原始 Keil MDK 工程（三层嵌套来自压缩包，未改动）
├── Core/            CubeMX 生成的 HAL 初始化（main.c 只做初始化 + App_Run）
├── Drivers/         ST 官方 HAL / CMSIS
├── MDK-ARM/
│   ├── OB_ROBOT.uvprojx      Keil 工程文件
│   └── User/                 自己写的应用层（见下表）
├── OB_ROBOT.ioc             CubeMX 工程
└── AGENTS.md                本工程的编码约定（中断里只做采样/控制，不做 OLED/打印）
```

`MDK-ARM/User/` 模块：

| 文件 | 职责 |
| --- | --- |
| `motor.c/.h` | TB6612 驱动，四轮引脚映射与极性 |
| `encoder.c/.h` | 编码器计数（LF/RF 硬件定时器，LR/RR 软件 EXTI） |
| `wheel_speed.c/.h` | 10 ms 轮速采样、PPC 标定、一阶滤波 |
| `wheel_speed_control.c/.h` | 四轮速度 PI + PWM 前馈闭环（**支持四轮带符号目标**） |
| `move.c/.h` | 底盘运动层：麦轮逆运动学、加减速斜坡、超时保护、前/后/左/右/旋转 |
| `motion_test.c/.h` | 按键 + OLED 的四向运动测试界面 |
| `speed_control_test.c/.h` | 原来的单速 PI 调参界面（保留，默认不调用） |
| `debug_telemetry.c/.h` | USART1 115200 上传 VOFA+ FireWater 波形 |
| `app.c/.h` | 10 ms 按键扫描、各模块非阻塞调度 |

## 编译

Keil MDK-ARM（ARMCC V5.06）打开
`OB_ROBOT(3)(1)/OB_ROBOT(1)/OB_ROBOT/MDK-ARM/OB_ROBOT.uvprojx`，
选择 `OB_ROBOT` target 后 Build。

## 运动层用法

速度单位统一是 **0.1 cm/s**（100 = 10 cm/s），正值表示车轮物理前进。
全部接口都是非阻塞的：只在 10 ms 定时中断里做斜坡与闭环，调用后立即返回。

```c
Move_Init();                       /* 已在 App_Init() 中调用 */

Move_Forward(100);                 /* 前进 10 cm/s */
Move_Backward(100);                /* 后退 */
Move_Left(100);                    /* 左平移 */
Move_Right(100);                   /* 右平移 */
Move_RotateLeft(80);               /* 原地左旋 */
Move_RotateRight(80);              /* 原地右旋 */
Move_Halt();                       /* 斜坡减速到 0 后关闭闭环 */
Move_Stop();                       /* 立即断 PWM（急停） */

/* 任意方向组合：vx 前进为正，vy 左移为正，wz 左旋为正 */
Move_Drive(vx_x10, vy_x10, wz_x10);

/* 定时运动：跑 ms 毫秒后自动减速停车 */
Move_DriveFor(vx_x10, vy_x10, wz_x10, 1500);

/* 距离闭环：走满 10000 = 1.00m 后自动减速停车（停车前预扣减速距离） */
Move_DriveDistance(100, 10000);

/* 本条距离指令已走行程（0.01cm），给界面显示用 */
int32_t trip = Move_GetTravelX100();
```

所有速度指令都会经过加减速斜坡（`MOVE_ACCEL_X10_PER_TICK`），
反转时目标先过零点再反向，避免电流冲击和齿轮打齿。

## 按键测试（1 米直行）

上电后 OLED 显示直行测试界面，**只用一个中间键**：

| 操作 | 动作 |
| --- | --- |
| 按一下 CENTER | 以 10 cm/s 直行，走满 **1.00 m** 自动减速停车 |
| 跑动中再按 CENTER | 中途停车（斜坡减速） |

屏幕上四个量：

| 显示 | 含义 |
| --- | --- |
| `RUN / DONE / READY` | 状态；DONE 时后面是实际走过的距离 |
| `TRIP +43.2/100.0` | 本次行程 / 目标（cm），四轮行程平均值 |
| `LF:... RF:...` | 左前/右前轮实测速度（cm/s） |
| `DEV:+0.2` | 左右两侧累计行程差（cm），正=左侧走得多（车往右偏） |

判断标准：

- **能走直**：`DEV` 起来一下就被纠回 0 附近，全程不超过 ±1cm，`DONE` 落点在 100cm 左右（±2cm 属正常）。
- **起步偏**：前 20cm 内 `DEV` 迅速变大且纠不回来 → 调 `WHEEL_FF_FRICTION_PWM`。
- **匀速慢偏**：`DEV` 持续单向增长 → 加大 `MOVE_STRAIGHT_KP`。
- **落点差太多**：`DONE` 明显不是 100cm → 查 `wheel_ppc_x100[]` 标定。

速度和距离都在 `motion_test.c` 顶部两个宏里改：
`MOTION_TEST_SPEED_X10`、`MOTION_TEST_DISTANCE_X100`。

其它方向（后退/横移/旋转）的运动接口仍然在 `move.c` 里，
换测试项目时直接调用 `Move_Backward()` / `Move_Left()` / `Move_RotateLeft()` 等即可。

## 速度闭环（增量式 PI + 前馈 + 摩擦补偿）

`wheel_speed_control.c`，10ms 一次，四轮各自独立但结构完全一致：

```
前馈    FF = 速度*45/10 + sign(速度)*120        <- 线性项 + 静摩擦补偿
PI      u[k] = u[k-1] + Kp*(e[k]-e[k-1])/10 + Ki*e[k]/1000 + (FF[k]-FF[k-1])
刹车    目标为 0 时改成 u = -速度*30/10（阻尼刹车，不溜车、不倒车）
限幅    ±1500 PWM
```

增量式（速度式）PI 没有独立积分状态，所以不存在积分饱和，也不会有
"某个轮子积分憋满、另一个还空着"的左右不对称；目标一变输出立刻跟上。

## 走不直怎么调（从快到慢，逐级排查）

| 现象 | 该调什么 | 位置 |
| --- | --- | --- |
| 一松开就起步，车头立刻被拽偏 | `WHEEL_FF_FRICTION_PWM`（四轮同步破静摩擦） | `wheel_speed_control.c` |
| 匀速直线，车身慢慢往一边偏 | `MOVE_STRAIGHT_KP` / `MOVE_STRAIGHT_DEADBAND_X100`（行程纠偏） | `move.c` |
| 行程纠偏一直顶到上限还偏 | 加大 `MOVE_STRAIGHT_WZ_LIMIT_X10`，或查下面两项 | `move.c` |
| 某个轮子稳态就跟不上 | `wheel_trim_x10[]` 配平（0.1cm/s，正值=该轮更快） | `wheel_speed_control.c` |
| 走了 2m 后距离明显对不上 | 重标 PPC：`wheel_ppc_x100[]` | `wheel_speed.c` |
| 停车往前溜 / 停不住 | `WHEEL_BRAKE_GAIN_X10`（越大刹得越狠） | `wheel_speed_control.c` |
| 高速蛇行、抖动 | 降 `WHEEL_PI_KP_X10`，或加大 `wheel_speed.c` 里的低通（现在 1:4） | 两处 |

调法建议：**先只调一个参数、每次改一点、跑 3m 记结果**。
行程纠偏是位置闭环，能吃掉稳态差速，所以配平表尽量调小，
让纠偏去兜底，这样换地面、换电量都不容易跑偏。

## 仓库说明

- 仓库地址：<https://github.com/hanserlover/git>
- `.zip` 原始压缩包、Keil 编译输出、`*.uvoptx` 等本机配置都已忽略，
  只保留源码、工程配置和文档。
- `.history_backup/` 存放导入前的原始 git 历史快照（bundle），已在忽略列表中。

## Git 备忘

| 命令 | 说明 |
| --- | --- |
| `git status` | 查看工作区状态 |
| `git log --oneline --graph` | 查看提交历史 |
| `git add -A` | 暂存全部变更 |
| `git commit -m "msg"` | 提交 |
| `git push` | 推送到远端 |
