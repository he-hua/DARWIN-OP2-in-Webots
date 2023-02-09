# 注意，要先将Webots文件夹中的两个路径放入Project Structure (我使用Webots2022b)
# Webots\lib\controller\python39
# Webots\projects\robots\robotis\darwin-op\libraries\python39
# 此外，需要在webots中设置机器人的controller为<extern>，并在Edit Configurations中配置路径与环境变量

import xlrd
import xlwt
import math
import numpy as np
from controller import Robot
from managers import RobotisOp2GaitManager, RobotisOp2MotionManager


# 计算机器人关节位置误差向量的模长
def norm2(err):
    norm = 0
    for i in range(len(err)):
        norm += err[i] ** 2
    return math.sqrt(norm)


class Darwin:
    def __init__(self):
        self.robot = Robot()  # 初始化Robot类以控制机器人
        # timeStep在webots界面的WorldInfo中设置,不能小于8，否则gaitManager不资瓷
        self.timeStep = int(self.robot.getBasicTimeStep())  # 获取当前每一个仿真步所仿真时间mTimeStep
        # self.keyboard = Keyboard()  # 键盘
        # self.keyboard.enable(self.timeStep)  # 采样周期
        # 约定设备变量的首字母大写，包括LED、Accelerometer、Gyro、Motors、PositionSensors
        self.HeadLED = self.robot.getDevice('HeadLed')  # 获取头部LED灯，高版本python需要用getDevice，低版本可以用getLED，下同
        self.EyeLED = self.robot.getDevice('EyeLed')  # 获取眼部LED灯
        self.HeadLED.set(0xff0000)  # 点亮头部LED灯并设置颜色
        self.EyeLED.set(0xa0a0ff)  # 点亮眼部LED灯并设置颜色
        self.Accelerometer = self.robot.getDevice('Accelerometer')  # 获取加速度传感器
        self.Accelerometer.enable(self.timeStep)  # 激活传感器，并以timeStep为周期更新数值
        self.fup = 0
        self.fdown = 0  # 定义两个类变量，用于之后判断机器人是否摔倒
        self.Gyro = self.robot.getDevice('Gyro')  # 获取陀螺仪
        self.Gyro.enable(self.timeStep)  # 激活陀螺仪，并以timeStep为周期更新数值
        self.motorNames = ('ShoulderR', 'ShoulderL', 'ArmUpperR', 'ArmUpperL', 'ArmLowerR', 'ArmLowerL',
                           'PelvYR', 'PelvYL', 'PelvR', 'PelvL', 'LegUpperR', 'LegUpperL', 'LegLowerR',
                           'LegLowerL', 'AnkleR', 'AnkleL', 'FootR', 'FootL', 'Neck', 'Head')  # 初始化各传感器名
        self.motorNum = len(self.motorNames)  # 有的论文会忽略Head motor的动作，这里未忽略。如果不需要可删除Names的最后一个元素，或Dim-=1
        self.deltaLimit = 0.2  # 如果motor的action为位置编码的增量，需要限定其幅度
        self.PositionSensors = []  # 初始化关节角度传感器
        self.Motors = []  # 初始化motor
        self.motorMinPositions = []  # 每个motor位置上界的编码值
        self.motorMaxPositions = []
        self.motorMaxTorque = []
        # 获取各传感器并激活，以mTimeStep为周期更新数值，获取各motor及其参数
        for i in range(0, self.motorNum):
            self.PositionSensors.append(self.robot.getDevice(self.motorNames[i] + 'S'))  # motor名称后加S即为传感器名称
            self.PositionSensors[i].enable(self.timeStep)
            self.Motors.append(self.robot.getDevice(self.motorNames[i]))
            self.motorMinPositions.append(self.Motors[i].getMinPosition())
            self.motorMaxPositions.append(self.Motors[i].getMaxPosition())
            self.motorMaxTorque.append(self.Motors[i].getMaxTorque())  # 读取各motor允许的最大扭矩，先将其discount再存储
            self.Motors[i].setAvailableTorque(self.motorMaxTorque[i])  # 设置各motor运动时可以使用的最大扭矩
        self.motionManager = RobotisOp2MotionManager(self.robot)  # 初始化机器人动作组控制器
        self.gaitManager = RobotisOp2GaitManager(self.robot, "config.ini")  # 初始化机器人步态控制器

    # 按照webots中设定的timeStep，运行一个step
    def step(self):
        ret = self.robot.step(self.timeStep)
        if ret == -1:
            exit(0)

    # 等待，参数单位为s
    def wait(self, waitTime):
        startTime = self.robot.getTime()
        while startTime + waitTime >= self.robot.getTime():
            self.step()

    # 处理跌倒，目前暂时不管
    def checkFallen(self):  # 双足机器人在训练时一定经常跌倒，需要寻找一种高效的站起方法
        acc_tolerance = 60.0
        acc_step = 100  # 计数器上限
        acc = self.Accelerometer.getValues()  # 通过加速度传感器获取三轴的对应值
        if acc[1] < 512.0 - acc_tolerance:  # 面朝下倒地时y轴的值会变小
            self.fup += 1  # 计数器加1
        else:
            self.fup = 0  # 计数器清零
        if acc[1] > 512.0 + acc_tolerance:  # 背朝下倒地时y轴的值会变大
            self.fdown += 1  # 计数器加 1
        else:
            self.fdown = 0  # 计数器清零
        if self.fup > acc_step:  # 计数器超出上限，即倒地时间超过acc_step个仿真步长
            self.motionManager.playPage(10)  # 执行面朝下倒地起身动作
            self.motionManager.playPage(9)  # 恢复准备行走姿势
            self.fup = 0  # 计数器清零
        elif self.fdown > acc_step:
            self.motionManager.playPage(11)  # 执行背朝下倒地起身动作
            self.motionManager.playPage(9)  # 恢复准备行走姿势
            self.fdown = 0  # 计数器清零

    # 执行torque类型的动作，更加平滑，但更难设计
    def executeTorque(self, torque):  # action为扭矩
        for i in range(0, self.motorNum):
            desTorque = np.clip(torque[i], -self.motorMaxTorque[i], self.motorMaxTorque[i])
            self.Motors[i].setTorque(desTorque)

    # 执行目标位置类型的动作，但动作太大可能会使机器人崩溃
    def executePosition(self, position):  # action为位置编码，不建议使用这种action
        for i in range(0, self.motorNum):
            desPosition = np.clip(position[i], self.motorMinPositions[i], self.motorMaxPositions[i])
            self.Motors[i].setPosition(desPosition)

    # 执行目标位置增量类型的动作，比较推荐
    def executePositionInc(self, positionInc):  # action为位置编码的增量
        for i in range(0, self.motorNum):
            position = self.PositionSensors[i].getValue()
            delta = np.clip(positionInc[i], -self.deltaLimit, self.deltaLimit)
            desPosition = np.clip(position + delta, self.motorMinPositions[i], self.motorMaxPositions[i])
            self.Motors[i].setPosition(desPosition)

    # 从加速度计读取加速度
    def getAcceleration(self):
        a = self.Accelerometer.getValues()
        return a

    # 根据读取的加速度计算机器人的倾斜角、滚转角
    def getAngle(self):
        a = self.getAcceleration()
        # Yaw通常使用其它传感器测量，但Darwin没有
        pitch = math.atan2(-a[0], math.sqrt(a[1] * a[1] + a[2] * a[2]))
        roll = math.atan2(a[1], a[2])
        return [pitch, roll]

    # 从陀螺仪读取角速度
    def getAngularVelocity(self):
        w = self.Gyro.getValues()
        return w

    # 获取20个motor的位置
    def getPosition(self):
        position = []
        for i in range(0, self.motorNum):  # 关节类型不同，测量得到以弧度为单位的角位置或以米为单位的线性位置
            position.append(self.PositionSensors[i].getValue())
        return position

    # 获取状态，目前还缺关节速度、偏航角
    def getState(self):  # 一部分状态，速度还需另外获取
        position = self.getPosition()  # dim=20
        angle = self.getAngle()  # dim=2
        w = self.getAngularVelocity()  # dim=3
        state = position
        state.extend(angle)
        state.extend(w)
        return state

    # 使机器人按照官方标准库运动，采集数据
    def generateData(self):
        self.step()  # 仿真一个步长，刷新传感器读数
        self.motionManager.playPage(9)
        self.wait(0.2)  # 等待0.2s
        self.gaitManager.setXAmplitude(0)  # 设置机器人停止
        self.gaitManager.setAAmplitude(0)  # 不转向
        self.gaitManager.start()
        self.wait(0.2)  # 等待0.2s
        positionList = []
        t = 0  # 计数单位为s
        while t < 5:
            if t < 4:
                self.gaitManager.setXAmplitude(1.0)  # 设置机器人前进
            else:
                self.gaitManager.setXAmplitude(0)  # 停止前进
                self.gaitManager.stop()
            self.gaitManager.step(self.timeStep)  # 生成一个步长的动作
            self.step()  # 运行一个step
            t += self.timeStep / 1000
            positionList.append(self.getPosition())
        # 关节位置数据保存到文件中
        row, col = np.shape(positionList)
        print("data: row =", row, ", col =", col)
        workBook = xlwt.Workbook(encoding="UTF-8")
        sheet = workBook.add_sheet("Sheet1", cell_overwrite_ok=True)
        for r in range(row):
            for c in range(col):
                sheet.write(r, c, positionList[r][c])
        workBook.save("./data/positionData.xls")  # 保存为xlsx可能会报错

    # 根据goal执行动作
    def executePositionData(self, goal):
        self.executePosition(goal)  # 采取目标位置作为动作
        self.step()  # 仿真一个step
        # # 下面是采取位置增量作为动作的代码，但是可能会把官方动作库的一个step的动作拆分成多个step的动作，导致运动卡顿
        # position = self.getPosition()  # dim=20
        # # 若goal比position更大，则err为正，可直接施加于position
        # err = list(map(lambda x: x[0] - x[1], zip(goal, position)))  # err = goal - position
        # normErr = norm2(err)
        # while normErr > 0.1:  # 实际上当采样频率比较高的时候，动作可以一步执行到位，此时while用处不大
        #     self.executePositionInc(err)  # 使用比位置类型动作更温和的位置增量类型动作
        #     self.step()  # 仿真一个step
        #     position = self.getPosition()  # dim=20
        #     err = list(map(lambda x: x[0] - x[1], zip(goal, position)))  # err = goal - position
        #     normErr = norm2(err)
        #     # print("position:", position)
        #     # print("goal:", goal)
        #     # print("error:", err)

    # 根据generateData采集的数据，依次执行动作
    def executeData(self):
        # 读数据
        workBook = xlrd.open_workbook("./data/positionData.xls")
        sheet = workBook.sheet_by_name("Sheet1")
        datas = []
        for r in range(sheet.nrows):
            data = []
            for c in range(sheet.ncols):
                data.append(sheet.cell_value(r, c))
            datas.append(list(data))
        # 执行数据
        for data in datas:
            self.executePositionData(data)


if __name__ == '__main__':
    darwin = Darwin()  # 初始化Darwin类
    darwin.step()  # 仿真一个步长，刷新传感器读数
    darwin.generateData()
    darwin.executeData()
