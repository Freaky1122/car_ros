import pygame

DEADZONE = 0.05

def deadzone(value):
    if value < DEADZONE and value > -DEADZONE:
        value = 0
    return value


class Joystick(object):
    def __init__(self, num=0) -> None:
        
        # 初始化pygame
        pygame.init()

        # 初始化joystick模块
        pygame.joystick.init()

        # 检查是否有手柄连接
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            # 没有手柄连接
            print("Error, no joysticks connected!")
        else:
            # 使用第一个找到的手柄
            self.joystick = pygame.joystick.Joystick(num)
            self.joystick.init()
            
        self.left_x_axis= 0.0
        self.left_y_axis = 0.0
        self.right_x_axis = 0.0
        self.right_y_axis = 0.0
        self.a_button = 0.0
        self.b_button = 0.0
        self.x_button = 0.0
        self.y_button = 0.0
        self.left_bumper = 0.0
        self.right_bumper = 0.0
        self.x_direction = 0.0
        self.y_direction = 0.0

    def update(self, is_show=0):
        for event in pygame.event.get():  # 必须处理事件循环
            pass
        # -1 -> 1
        self.left_x_axis = round(self.joystick.get_axis(0), 1)
        self.left_y_axis = round(-self.joystick.get_axis(1), 1)
        self.right_x_axis = round(self.joystick.get_axis(2), 1)
        self.right_y_axis = round(-self.joystick.get_axis(3), 1)
        self.left_bumper = round(self.joystick.get_axis(5), 1)
        self.right_bumper = round(self.joystick.get_axis(4), 1)

        self.a_button = self.joystick.get_button(0)
        self.b_button = self.joystick.get_button(1)
        self.x_button = self.joystick.get_button(3)
        self.y_button = self.joystick.get_button(4)
        self.left_trigger = self.joystick.get_button(6)
        self.right_trigger = self.joystick.get_button(7)
        self.x_direction = self.joystick.get_hat(0)[0]
        self.y_direction = self.joystick.get_hat(0)[1]

        pygame.time.wait(1)  # 等待100ms
        
        if is_show:
            self.show()

    
    def show(self):
        print(f"Left  >> X-axis: {self.left_x_axis}, Y-axis: {self.left_y_axis}")
        print(f"Right >> X-axis: {self.right_x_axis}, Y-axis: {self.right_y_axis}")
        print(f"Bumper >> Left:{self.left_bumper}, Right:{self.right_bumper} ")
        print(f"Direction >> Left:{self.x_direction}, Right:{self.y_direction} ")
        print(f"A: {self.a_button}, B: {self.b_button}, X: {self.x_button}, Y: {self.y_button}")
        print(f"Trigger >> Left:{self.left_trigger}, Right:{self.right_trigger}")
        
    def close(self):
        print("JoyStick Exiting...")
        pygame.quit()
