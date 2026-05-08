# software/tools

这里存放日常调试和辅助工具。

当前工具包括：

- `get_arm_info.py`：读取机械臂名称、固件版本等信息
- `arm_hardware_reset.py`：硬件复位辅助脚本
- `arm_serial_controller.py`：串口命令调试工具
- `filtered_master_slave_teleop.py`：滤波手感优先的主从遥控工具，保留 follower 固件 slave 滤波语义
- `fast_io_teleop_pair.py`：基于 `zyarm_sdk.ZyArmTeleopPair` 的 fast_io 主从遥控工具，用于体验和数据采集链路验证
- `master_slave_remote.py`：旧入口兼容 wrapper，后续请改用 `filtered_master_slave_teleop.py`
- `joycon_sixdof_controller.py`：Joy-Con 六自由度输入解析模块，供手柄遥控脚本调用
- `JOYCON_SIXDOF_TUNING_GUIDE.md`：Joy-Con 手感、死区和灵敏度调参参考

运行这些脚本前，请先根据 `requirements.txt` 安装所需 Python 依赖。
