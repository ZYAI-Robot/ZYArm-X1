import time
import serial

# ==========================================
# 继电器控制类 
# 继电器型号：​ USB Relay PRO (TC, 8, CB, Opto)，(URP-T8B0)
# 参考手册：https://wiki.diustou.com/cn/USB_Relay_PRO
# ==========================================
class URPT8B0:
    """
        URP-T8B0 的其余通道
        CH_2         = 0x02
        ...
        CH_8         = 0x08

        其他参数修改见参考手册
    """

    # 动作指令，这里仅使用第一个通道
    ACTION_OFF   = 0x00
    ACTION_ON    = 0x01
    ACTION_QUERY = 0x02
    CH_1         = 0x01
    
    # 初始化函数
    def __init__(self, port, baudrate=115200):
        """
        初始化 USB 继电器控制类
        :param port: 串口号 (如 'COM23' 或 '/dev/ttyUSB0')
        :param baudrate: 波特率 (默认 115200)
        """
        self.port = port
        self.serial_port = None
        try:
            self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=1.0)
        except Exception as e:
            print(f"[Relay] 错误: {e}")
            raise e

    # 发送底层指令 
    def _send_command(self, channel, action):
        """
        发送底层指令到继电器
        :param channel: 通道号 (通常 0x01)
        :param action: 动作指令 (0x01=开, 0x00=关)
        """
        if not self.serial_port or not self.serial_port.is_open: return
        header = 0xA0
        checksum = (header + channel + action) & 0xFF
        cmd_bytes = bytes([header, channel, action, checksum])
        self.serial_port.write(cmd_bytes)
        self.serial_port.flush()

    # 继电器状态确认
    def get_status(self):
        """
        查询继电器当前的开关状态
        :return: True=开, False=关, None=查询失败/未知
        """
        if not self.serial_port or not self.serial_port.is_open: return None
        self.serial_port.reset_input_buffer()
        header = 0xA0; checksum = (header + self.CH_1 + self.ACTION_QUERY) & 0xFF
        self.serial_port.write(bytes([header, self.CH_1, self.ACTION_QUERY, checksum]))
        try:
            time.sleep(0.1)
            if self.serial_port.in_waiting > 0:
                resp = self.serial_port.read(self.serial_port.in_waiting)
                try: resp_str = resp.decode('utf-8', 'ignore')
                except: resp_str = ""
                if "ON" in resp_str or (len(resp)>=3 and resp[2]==0x01): return True
                if "OFF" in resp_str or (len(resp)>=3 and resp[2]==0x00): return False
            return None
        except: return None

    # 继电器开
    def power_on(self):
        """
        打开继电器通道 1
        给机械臂供电 (Power Up)
        调用内部发送函数，发送指令 [0xA0, 0x01, 0x01, Checksum]
        """
        self._send_command(self.CH_1, self.ACTION_ON)

    # 继电器关
    def power_off(self):
        """
        关闭继电器通道 1
        切断机械臂电源 (Hard Shutdown)，用于硬复位或结束测试
        调用内部发送函数，发送指令 [0xA0, 0x01, 0x00, Checksum]
        """
        self._send_command(self.CH_1, self.ACTION_OFF)

    # 串口资源释放
    def close(self):
        """
        关闭串口连接，释放操作系统资源：
        1. 释放 COM 口句柄，防止下次运行时报 'PermissionError' 或 'Access Denied'。
        2. 确保在脚本退出或异常结束时，电脑不再占用该 USB 设备。
        """
        if self.serial_port: self.serial_port.close()