from CholoepusCtrl import CholoepusCtrl
import time

controller = CholoepusCtrl(port='/dev/tty.usbmodem92867CBE124B1')  # 替换为实际串口
# controller.initialize()
# controller.set_torque(1)  # 设置扭矩1.5
# time.sleep(1)
# controller.set_angle(10)  # 设置角度45度
# time.sleep(1)
controller.set_angle_with_torque_limit(40.0, 1.0)  # 设置角度30度，电压限制5
time.sleep(10)
controller.close()