import serial
import time

class CholoepusCtrl:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        初始化夹爪控制器
        :param port: 串口号 (如 'COM3' 或 '/dev/ttyUSB0')
        :param baudrate: 波特率，默认为115200
        :param timeout: 超时时间，单位秒
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.device_id = 928  # 固定ID为928
        self.serial = None
        self.connect()

    def connect(self):
        """建立串口连接"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            print(f"Connected to {self.port} at {self.baudrate} baud")
            time.sleep(2)  # 等待连接稳定
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            raise

    def _send_command(self, mode: int, value: float = 0.0, extra_value: float = 0.0):
        """发送命令"""
        command = bytearray(16)
        command[0] = 0xAA
        command[1:6] = [0x00, 0x00, 0x08, 0x00, 0x00]
        id_bytes = self.device_id.to_bytes(2, byteorder='big')
        command[6:8] = id_bytes
        command[8] = mode
        if (value < 0): value = 0
        if (value > 90): value = 90
        value_int = int(value * 100)
        value_bytes = value_int.to_bytes(2, byteorder='big', signed=True)
        command[10:12] = value_bytes
        if mode == 3:
            extra_value_int = int(extra_value * 100)
            extra_value_bytes = extra_value_int.to_bytes(2, byteorder='big', signed=True)
            command[12:14] = extra_value_bytes
        self.serial.write(command)

    def initialize(self):
        """初始化控制器"""
        self._send_command(0)

    def set_torque(self, torque: float):
        """设置扭矩"""
        self._send_command(1, torque)

    def set_angle(self, angle: float):
        """设置角度"""
        self._send_command(2, angle)

    def set_angle_with_torque_limit(self, angle: float, voltage_limit: float):
        """力位混合控制：设置角度带扭矩限制"""
        self._send_command(3, angle, voltage_limit)

    def close(self):
        """关闭串口连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial connection closed")