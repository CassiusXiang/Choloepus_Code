
import time
from CholoepusCtrl import CholoepusCtrl

# 初始化控制器
controller = CholoepusCtrl(port='/dev/tty.usbmodem92867CBE124B1')  # 替换为实际串口

controller.set_angle_with_torque_limit(90, 1)
time.sleep(10)
controller.set_angle_with_torque_limit(10, 1)
time.sleep(10)
