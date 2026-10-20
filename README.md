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
```

所有速度指令都会经过加减速斜坡（`MOVE_ACCEL_X10_PER_TICK`），
反转时目标先过零点再反向，避免电流冲击和齿轮打齿。

## 按键测试

上电后 OLED 显示四向运动测试界面：

| 按键 | 动作 |
| --- | --- |
| UP | 前进 |
| DOWN | 后退 |
| LEFT | 左平移 |
| RIGHT | 右平移 |
| CENTER | 停车 |

再按一次同方向键即可停车。想测原地旋转，把 `motion_test.c` 里
`MOTION_TEST_ROTATE_WITH_LR` 改成 1，LEFT/RIGHT 就变成原地左转/右转。

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
