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
| `heading.c/.h` | 编码器里程计 + **航向角环**（外环），四方向直线保持 |
| `move.c/.h` | 底盘运动层：麦轮逆运动学、加减速斜坡、距离闭环、航向保持、前/后/左/右/旋转 |
| `motion_test.c/.h` | 按键 + OLED 的四方向 1m 直行测试界面 |
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

/* 距离闭环：沿指令方向走满 10000 = 1.00m 后自动减速停车
 * （中途预扣减速距离，横移也能用；最后一个参数是距离） */
Move_DriveDistance(100, 0, 10000);      /* 前进 1m */
Move_DriveDistance(0, 100, 10000);      /* 左移 1m */

/* 本条距离指令已走行程（0.01cm），给界面显示用 */
int32_t trip = Move_GetTravelX100();
```

所有速度指令都会经过加减速斜坡（`MOVE_ACCEL_X10_PER_TICK`），
反转时目标先过零点再反向，避免电流冲击和齿轮打齿。
纯平移指令会自动锁定当前航向，由角度环保持直线（见下文）。

## 按键测试（四方向各走 1 米）

上电后 OLED 显示四方向直行测试界面：

| 按键 | 动作 |
| --- | --- |
| UP | 前进 1.00 m |
| DOWN | 后退 1.00 m |
| LEFT | 左移 1.00 m |
| RIGHT | 右移 1.00 m |
| CENTER | 停止（斜坡减速） |

每个方向都是「距离闭环 + 航向保持」，走满 1m 自动停车。跑动中按其它方向键
会立刻换向（斜坡会先过零）。

屏幕上四个量：

| 显示 | 含义 |
| --- | --- |
| `FWD RUN / DONE / READY` | 方向 + 状态 |
| `TRIP +43.2/100.0` | 本次沿指令方向的行程 / 目标（cm） |
| `YAW+1.2 E+0.3 ODO` | 当前航向、角度环误差（度），以及航向来源（ODO=里程计，IMU=外部陀螺） |
| `LF:... RF:...` | 跑动中显示左前/右前轮实测速度（cm/s） |

判断标准：

- **能走直**：`YAW` 起来一下就被拉回 0 附近，`E` 全程在 ±0.5° 内，`DONE` 落点 100cm 左右（±2cm 正常）。
- **起步偏**：前 20cm 内 `YAW` 猛冲且回不来 → 调 `WHEEL_FF_FRICTION_PWM`（四轮不同时破静摩擦）。
- **持续慢偏 / 蛇行**：调 `HEADING_KP`（太小纠不动，太大会摆）、`HEADING_KI`（压稳态偏差）。
- **落点差太多**：`DONE` 明显不是 100cm → 查 `wheel_ppc_x100[]` 标定。
- **四个方向不一致**（比如只有横移偏）：先确认 PPC 一致，再看 `heading.c` 的 `HEADING_TRACK_X10`（只影响比例，一般不用改）。

速度和距离都在 `motion_test.c` 顶部两个宏里改：
`MOTION_TEST_SPEED_X10`、`MOTION_TEST_DISTANCE_X100`。

## 角度环（航向保持）

`heading.c`，与速度环构成串级：**角度环（外）→ 速度环（内）**。

```
里程计（10ms，用位移不是速度）：
    d_vx  = ( LF + RF + LR + RR) / 4      前进位移
    d_vy  = ( LF - RF - LR + RR) / 4      左移位移
    d_rot = (-LF + RF - LR + RR) / 4      转动位移
    航向 += d_rot * 360 / (π * 左右轮距)

角度环（PI，误差 0.1°，输出 wz 0.1cm/s）：
    误差 = 目标航向 - 当前航向（归一化到 ±180°）
    wz   = Kp*误差 + Ki*∫误差            Kp=60/100, Ki=100/10000, 死区 0.3°, 限幅 6cm/s
```

**为什么四个方向都能走直线**：`d_rot` 只跟车身转不转有关，与往哪个方向走无关。
运动层在纯平移指令开始时锁住当前航向（`Heading_Hold()`），
角度环的 `wz` 修正叠加到指令上，前进/后退/左移用的是同一条通路；
旋转指令期间角度环让路（`Heading_Release()`），不跟人抢方向盘。

**局限（重要）**：航向来自编码器里程计，**轮子打滑时它是错的**——它测的是
轮子走过的距离差，不是车身真实转角。另外 PPC 标定如果不一致，里程计会
出现"虚假偏航"，角度环就会真的把车拐弯。所以：

- PPC 必须逐轮标定准确（见 `wheel_speed.c` 的 `wheel_ppc_x100[]`）。
- 想彻底摆脱打滑影响，得用陀螺。接口已经留好：收到一帧真实 yaw 时调用
  `Heading_InputExternalYaw(yaw_01deg, 1)`，之后航向就用外部值，
  连续 200ms 没新数据自动退回里程计；OLED 上的来源会显示 `IMU`。

**怎么验证里程计可不可信**：把 `move.c` 的 `MOVE_HEADING_HOLD` 改成 0
（关掉航向保持）跑 1m，看车实际偏了多少、跟屏幕上报的 `YAW` 差多少。
两者接近 → 里程计可信，可以把角度环打开；差很多 → 先重标 PPC。

## 陀螺（IMU601）接入进展：还没用上

目前的**角度环用的是编码器里程计，不是陀螺**。原因很具体：

- `imu601.c` 只有「发指令」的代码，姿态帧解析在本工程 git 历史里从未存在
  （全部 20 个提交里最早的 `imu601.c` 就已经是发送版）；
- 全工程唯一读 USART3 的地方是 `IMU601_Process()`，它把收到的字节**直接丢掉**；
- 硬件通路本身是好的：USART3 = PC10/PC11，115200 8N1，`usart.c` 里
  `Mode = UART_MODE_TX_RX` 且 NVIC 已使能，模块上电就在持续上报。

所以缺的只是**帧格式**。拿到它就能把角度环换成真陀螺：

### 路线 A：用内置的原始字节嗅探器抓协议（不用手册）

1. 把 `debug_telemetry.h` 里的 `DEBUG_IMU_SNIFFER` 改成 **1**，编译烧写。
   （此时 USART1/PA9 不再是 VOFA+ 波形，而是 hex 文本）
2. 串口助手 115200 连 PA9，**静止**抓 20 行；再把车**原地转 90°**，再抓 20 行。
3. 把两段 hex 贴回来，同时告诉我「静止时车头朝哪个方向」。我按字节变化反推
   帧头/长度/字段偏移/校验，然后：
   - 加 `IMU601_GetYaw()`（收帧 + 校验 + 单位换算）；
   - 在收帧处调用 `Heading_InputExternalYaw(yaw_01deg, 1)`；
   - OLED 上的来源自动变成 `IMU`，角度环就成了真陀螺闭环。

嗅探输出长这样（每行最多 12 字节，10ms 一行）：

```
[123456] AA 55 60 01 06 3F 12 00 8C
[123466] AA 55 60 01 06 3F 12 00 8C
```

### 路线 B：直接给我现成的

任一份都行，我就能省掉反推：

- 模块手册里「姿态上报」那一页（帧格式/字段定义/校验规则）；
- 你之前删掉的那版 `imu601.c`（有 `Attitude_t`、`IMU601_GetAttitude()` 的）。

注意：打开嗅探后 `app.c` 里会通过 `#if` 关掉 `IMU601_Process()`，
否则原始字节会在主循环里被提前读空。

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
| 匀速直线慢慢偏、`E` 压不下去 | `HEADING_KP` 加大；还压不下就加 `HEADING_KI` | `heading.c` |
| 航向来回摆（蛇行） | `HEADING_KP` 减小，或加大 `HEADING_DEADBAND_01DEG` | `heading.c` |
| 修正量顶到上限还是偏 | 加大 `HEADING_WZ_LIMIT_X10`，并检查是不是在打滑 | `heading.c` |
| 关掉角度环后车能走直、打开反而偏 | PPC 不一致导致的虚假偏航 → 重标 `wheel_ppc_x100[]` | `wheel_speed.c` |
| 某个轮子稳态就跟不上 | `wheel_trim_x10[]` 配平（0.1cm/s，正值=该轮更快） | `wheel_speed_control.c` |
| 走了 2m 后距离明显对不上 | 重标 PPC：`wheel_ppc_x100[]` | `wheel_speed.c` |
| 停车往前溜 / 停不住 | `WHEEL_BRAKE_GAIN_X10`（越大刹得越狠） | `wheel_speed_control.c` |
| 高速蛇行、抖动 | 降 `WHEEL_PI_KP_X10`，或加大 `wheel_speed.c` 里的低通（现在 1:4） | 两处 |

调法建议：**先只调一个参数、每次改一点、跑 1~3m 记结果**。
先确认轮速环（`LF/RF` 两个数能稳在目标上），再调角度环；
配平表（`wheel_trim_x10[]`）尽量调小，让角度环去兜稳态偏航。

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
