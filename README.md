# UAV 26赛季无人机代码

```
遥控：
    开云台+自瞄			 连发
   ↗                   ↗ 
左上角 → 开云台      右上角  →单发
   ↘                   ↘
    无力                  最高权限失能摩擦轮
拨轮只有在摩擦轮开启时才能开启
向上拨一点：开关摩擦轮
向上波满：退弹
向下波：发射
左摇杆右下角2s：切换编码器模式
左摇杆右上角2s：切换陀螺仪模式
左摇杆左上角且右拨杆居中：加弹速
左摇杆左下角且右拨杆居中：减弹速
左摇杆左上角且右拨杆上拨：加弹频
左摇杆左下角且右拨杆上拨：减弹频

图传（优先级高）：
长按鼠标右键 自瞄
鼠标左键 开拨弹盘
R：开关摩擦轮
C：弹速RPM逻辑
V：自瞄pitch补偿逻辑
G：弹频逻辑
鼠标滚轮：逻辑加减
WS：逻辑大幅加减
AD：逻辑小幅定额加减
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

![image-20250826214631744](https://gitee.com/lzero123/typora_-img/raw/master/PicGo/image-20250826214631744.png)

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

![image-20250826215019514](https://gitee.com/lzero123/typora_-img/raw/master/PicGo/image-20250826215019514.png)
