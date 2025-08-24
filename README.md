# UAV 26赛季无人机代码

```
     开云台+自瞄			 棘轮向下拨一点时--打一梭子
   ↗                   ↗ 
左上角 → 开云台      右上角  →棘轮向下拨一点时时单发
   ↘                   ↘
    无力                  最高权限失能摩擦轮
拨轮只有在摩擦轮开启时才能开启 --》向上拨一点开摩擦轮
向下拨一点：由右上拨杆决定
向下波到底：连发
图传（优先级高）：
长按鼠标右键 自瞄
鼠标左键 开拨弹盘
R：开关摩擦轮
C：加弹速RPM         ctrl+c：减弹速
V：加自瞄pitch补偿  ctrl+V：减补偿
G：高弹频               ctrl+G：低弹频
左键射弹，右键自瞄
```

## 主要状态以及切换状态说明

## 1.fire 发射机构

```c
typedef enum
    {
        Disable    = 0x00, // 失能
        Enable     = 0x01, // 使能
        Ready      = 0x02, // 摩擦轮转动，允许播弹盘
        OneShoot   = 0x03, // 单发逻辑
        StartShoot = 0x04, // 连发逻辑
        Stuck      = 0x05, // 卡弹
    } Firc_State_e;
```

![fire](https://gitee.com/lzero123/typora_-img/raw/master/PicGo/fire.png)

## 2.gimbal 云台

```c
typedef enum
{
    Disable            = 0x00, // 失能
    ControlWithEncoder = 0x01, // 编码器的手动控制
    ControlWithIMU     = 0x02, // 陀螺仪的手动控制
    AutoControl        = 0x03, // 自瞄
} Gimbal_State_e;
```

![image-20250819185743367](https://gitee.com/lzero123/typora_-img/raw/master/PicGo/image-20250819185743367.png)
