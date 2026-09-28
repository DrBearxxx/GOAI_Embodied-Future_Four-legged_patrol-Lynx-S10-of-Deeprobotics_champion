# 唤醒接口依据

核对对象为此前从用户 G12 原厂 App 提取的 smali：

- `CommandId.SLEEP_MODE_MSG_ID`：`CommandType(functionID=2, subFuctionID=0x40, operateID=2)`，默认 `productID=0x10`。
- `MessageUtils.getMessage`：Type = `(productID << 16) | functionID`；Command = `(subFuctionID << 16) | operateID`。
- `MessageUtils.changeSleepModeMsg(false)`：Items 仅包含 `Sleep: false`。

因此请求为 Type `0x100002`、Command `0x400002`、Items `{"Sleep": false}`。不会顺带修改自动休眠配置、发送起立、改变 SDK 所有权或启用行走。

2026-09-21 通过统一 App 实际发送后，底盘返回 ErrorCode=0，随后独立 BasicStatus 确认 Sleep=false；MotionState 仍为 0，网关 enabled=false，非零速度计数为 0。实现依据实际状态而非 ACK 判断唤醒完成。
