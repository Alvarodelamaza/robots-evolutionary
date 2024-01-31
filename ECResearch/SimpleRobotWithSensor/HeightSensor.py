import random
class HeightSensor:
    def __init__(self, noise_level=0.1):
        # 假设传感器有一定的噪声水平
        self.noise_level = noise_level

    def get_height(self, robot_position):
        # 获取机器人在z轴上的位置，加上一些随机噪声来模拟现实世界的不精确性
        return robot_position.z + random.uniform(-self.noise_level, self.noise_level)

# 示例使用
sensor = HeightSensor(noise_level=0.05)
# 假设我们从仿真环境获取到机器人的位置
# robot_position = get_robot_position_from_simulation()
# height = sensor.get_height(robot_position)
# print(f"Measured Height: {height}")
