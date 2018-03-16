#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Родительский модуль для управления кукой
"""

import math
import random
import datetime
from wx import wx
import rospy
import brics_actuator.msg
import geometry_msgs.msg
import sensor_msgs.msg
from kinematic import getDHMatrix, getG
import time
import numpy as np

from jacobian.main import get_inversed_jacobian


class KukaWrapper:
    """
    Родительский класс для управления кукой
    """

    # Максимальная скорость звеньев робота
    maxJointVelocity = math.radians(90)

    # диапазон допустимых значений джоинтов
    jointsRange = [
        [0.011, 5.840],
        [0.011, 2.617],
        [-5.0, -0.02],
        [0.03, 3.42],
        [0.15, 5.641],
    ]
    L = [33, 147, 155, 135, 217.5]

    task = [0, 0, 0, 0, 0, 0, 0]

    TARGET_TYPE_MANY_JOINTS = 0
    TARGET_TYPE_ONE_JOINT = 1
    TARGET_TYPE_FINGERS = 2
    TARGET_TYPE_NO_TARGET = -1

    candlePos = [2.9496, 1.1345, -2.5482, 1.7890, 2.9234]

    targetJPoses = [0, 0, 0, 0, 0]
    targetGPoses = [0, 0]
    targetJPos = 0
    targetJposNum = -1
    targetType = TARGET_TYPE_NO_TARGET
    startTime = 0

    jointOffsets = [
        -170.0 / 180 * math.pi,
        -65.0 / 180 * math.pi,
        (90 + 60.0) / 180 * math.pi,
        0.0 / 180 * math.pi,
        -90.0 / 180 * math.pi
    ]
    flgReached = True

    # диапазон допустимых значений движения гриппера
    gripperRange = [0.0, 0.011499]
    # константы типов сообщений
    TYPE_JOINT_POSITIONS = 0  # положение джоинта
    TYPE_JOINT_VELOCITIES = 1  # скорость джоинта
    TYPE_JOINT_TORQUES = 2  # момент джоинта
    TYPE_GRIPPER_POSITION = 3  # положение гриппера
    TYPE_GRIPPER_VELOCITY = 4  # скорость гриппера
    TYPE_GRIPPER_TORQUE = 5  # сила гриппера
    # ответные данные, полученные от куки
    jointState = sensor_msgs.msg.JointState()

    overG = [0] * 5
    G = [0] * 5
    F = [0] * 3
    wrenches = np.zeros((3, 6))

    G_ERROR_RANGE = [10, 0, 0, 0, 10]
    G_K = [0.2, 0.1, 0.07, 0.3, 0.5]
    MAX_V = [0.4, 0.4, 0.4, 0.4, 0.4]

    reachedAction = False

    def warn(self, message, caption='Warning'):
        """
            Показать сообщение
        :param message: Текст сообщения
        :param caption: Заголовок сообщения
        """
        dlg = wx.MessageDialog(None, message, caption, wx.OK | wx.ICON_WARNING)
        dlg.ShowModal()
        dlg.Destroy()

    def inCandleWithWaiting(self):
        """
            В свечу с режимом ожидания
        """
        self.setJointPositions(self.candlePos)
        self.targetType == self.TARGET_TYPE_MANY_JOINTS
        self.targetJPoses = self.candlePos
        while self.targetType == self.TARGET_TYPE_MANY_JOINTS:
            rospy.sleep(0.1)

    def checkPositionXYZEnable(self, pos):
        """
            проверка, что положение допустимо
        :param pos: координаты в декартовом пространстве
        :return: True/False
        """
        x = pos[0]
        y = pos[1]
        z = pos[2]
        if z < 200:
            return False
        elif z < 300:
            return x ** 2 + y ** 2 > 350 ** 2

        return True

    #
    def checkPositionJEnabled(self, joints):
        """
             проверка, что положение допустимо
        :param joints: обобщённые координаты
        :return: True/False
        """
        xyz = self.getEndEffectorPosByJ(joints)
        # print(xyz)
        return self.checkPositionXYZEnable(xyz)

    #
    def warmUpLink(self, n, t):
        """
            разогрев
        :param n: номер звена
        :param t: время разогрева
        """
        startTime = time.time()
        while (time.time() - startTime < t * 60):
            pos = random.uniform(self.jointsRange[n - 1][0], self.jointsRange[n - 1][1])
            self.setJointPosition(n, pos)
            rospy.sleep(0.5)

    def setPosAndWait(self, joints):
        """
             отправить в позицию и ждать, пока робот в неё не придёт
        :param joints: обобщённые координаты
        """
        if self.checkPositionJEnabled(joints):
            self.setJointPositions(joints)
            self.targetType == self.TARGET_TYPE_MANY_JOINTS
            self.targetJPoses = joints
            for k in range(5):
                self.task[k] = joints[k]
            cnt = 0
            while self.targetType == self.TARGET_TYPE_MANY_JOINTS and cnt < 5:
                rospy.sleep(0.1)
                cnt += 1
            return True
        return False

    def checkFingerPosition(self, fingerPos):
        return True

    def setGripperPosAndWait(self, leftP, rightP):
        """
        установить желаемое положение пальцев и ждать,
        пока оно не будет достигнуто
        """
        if self.checkFingerPosition(leftP) and self.checkFingerPosition(rightP):
            self.task[5] = leftP
            self.task[6] = rightP
            self.targetGPoses = [leftP, rightP]
            self.targetType = self.TARGET_TYPE_FINGERS
            self.setGripperPositions(leftP, rightP)

            cnt = 0
            while self.targetType == self.TARGET_TYPE_FINGERS and cnt < 5:
                rospy.sleep(0.5)
                cnt += 1

            return True
        return False

    def checkCurPositionEnabled(self):
        """
             проверить, что текущее положение допустимо(это нужно для проверки рассчётов допустимой зоны)
        :return: True/False
        """
        xyz = self.getEndEffectorPosByJ(self.jointState.position)
        # print(xyz)
        return self.checkPositionXYZEnable(xyz)

    def randomPoints(self, n, tp):
        """
            отправить робота в рандомные положения
        :param n: кол-во положений
        :param tp: пауза в секундах между командами
        :return:
        """
        for i in range(n):
            print(i)
            rospy.sleep(tp)
            flgCreated = False
            while not flgCreated:
                joints = []

                for j in range(5):
                    joints.append(random.uniform(self.jointsRange[j][0], self.jointsRange[j][1]))

                if self.checkPositionJEnabled(joints):
                    flgCreated = True
                    self.setJointPositions(joints)
                    self.targetType == self.TARGET_TYPE_MANY_JOINTS
                    self.targetJPoses = joints
                    for k in range(5):
                        self.task[k] = joints[k]

            while self.targetType == self.TARGET_TYPE_MANY_JOINTS:
                rospy.sleep(1)

            self.task = [0] * 7

        self.warn("Робот отработал все точки")

    def getDH(self, position):
        """
            по положению получаем матрицу перехода из энд-эффектора в асолютную систему координат
        :param position: массив из пяти элементов с обобщёнными координатами робота
        :return: Матрица преобразования 4х4
        """
        DH1 = getDHMatrix(math.pi / 2, self.L[0], self.L[1], position[0] + self.jointOffsets[0])
        DH2 = getDHMatrix(0, self.L[2], 0, position[1] + self.jointOffsets[1] + math.pi / 2)
        DH3 = getDHMatrix(0, self.L[3], 0, position[2] + self.jointOffsets[2])
        DH4 = getDHMatrix(math.pi / 2, 0, 0, position[3] + self.jointOffsets[3])
        DH5 = getDHMatrix(0, 0, self.L[4], position[4] + self.jointOffsets[4])
        return DH1 * DH2 * DH3 * DH4 * DH5

    def getTF(self):
        """
            Текущая матрица преобразования робота
        :return: Матрица преобразования 4х4
        """
        return self.getDH(self.jointState.position)

    def getEndEffectorPos(self):
        """
            получить положение энд-эффектора
        :return: Массив декартовых координат [x,y,z]
        """
        tf = self.getTF()
        return [tf.item(0, 3), tf.item(1, 3), tf.item(2, 3)]

    def getEndEffectorPosByJ(self, joints):
        """
            Возвращает положение энд-эффектора по обобщённым координтатам
        :param joints:  массив из пяти элементов с обобщёнными координатами робота
        :return: Массив декартовых координат [x,y,z]
        """
        tf = self.getDH(joints)
        return [tf.item(0, 3), tf.item(1, 3), tf.item(2, 3)]

    def checkIfListIsZero(self, lst):
        """
            проверить, что список состоит из нулей
        :param lst: массив(список)
        :return: True/False
        """
        flg = True
        for i in range(len(lst)):
            if lst[i] != 0:
                flg = False
        return flg

    def sign(self, x):
        return 1 if x >= 0 else -1

    def getGravityTorques(self, currentTorques, q):
        offset = [2.9496, 1.1345, -2.5482, 1.7890, 2.9234]  # by passport
        angle = [q[j] - offset[j] for j in range(5)]

        theta1 = np.array([-0.9762, 0.2294, -2.1608, 0.0582, -3.2580, -0.2012, 0.3085])
        theta2 = np.array([-0.9441, 0.0091, -1.9762, -0.0198, 0.1136])
        theta3 = [-0.7205, 0.0084, 0.1464]

        phi1 = np.array([math.sin(angle[1] + angle[2] + angle[3]),
                math.cos(angle[1] + angle[2] + angle[3]),
                math.sin(angle[1] + angle[2]),
                math.cos(angle[1] + angle[2]),
                math.sin(angle[1]),
                math.cos(angle[1]),
                self.sign(currentTorques[1])])

        phi2 = np.array([math.sin(angle[1] + angle[2] + angle[3]),
                math.cos(angle[1] + angle[2] + angle[3]),
                math.sin(angle[1] + angle[2]),
                math.cos(angle[1] + angle[2]),
                self.sign(currentTorques[2])])

        phi3 = [math.sin(angle[1] + angle[2] + angle[3]),
                math.cos(angle[1] + angle[2] + angle[3]),
                self.sign(currentTorques[3])]

        torques = [
            0,
            sum(theta1*np.transpose(phi1)),
            sum(theta2*np.transpose(phi2)),
            sum([theta3[0]*phi3[0], theta3[1]*phi3[1], theta3[2]*phi3[2]]),
            0
        ]

        return torques

    def calculateWrenches(self, position, allTorques, gravityTorques):
        invJ = get_inversed_jacobian(position)

        torques = [allTorques[0], allTorques[1], allTorques[2], allTorques[3], allTorques[4]]

        whole = np.dot(invJ, torques)
        inner = np.dot(invJ, gravityTorques)

        outer = np.dot(invJ, [torques[i] - gravityTorques[i] for i in range(5)])

        return [
            whole,
            inner,
            outer # (whole - inner)/100
        ]

        # return [
        #     np.dot(invJ, np.array(torques)),
        #     np.dot(invJ, np.array(gravityTorques)),
        #     np.dot(invJ, np.array(torques - gravityTorques))
        # ]


    def calculateOverG(self):
        """
             посчитать избыточные моменты
        :return: массив(список) избыточных моментов на каждом звене
        """

        gravityTorques = self.getGravityTorques(self.jointState.effort, self.jointState.position)

        for i in range(5):
            self.overG[i] = self.jointState.effort[i] - gravityTorques[i]
            if abs(self.overG[i]) < self.G_ERROR_RANGE[i]:
                self.overG[i] = 0

        self.wrenches = self.calculateWrenches(self.jointState.position, self.jointState.effort, gravityTorques)
        # print('OverG {}'.format(self.overG))
        # print('r {}'.format(self.jointState.effort))
        # print('e {}'.format(gravityTorques))

    def getF(self):
        self.F[0] = self.overG[0]
        self.F[1] = self.overG[1]
        self.F[2] = self.overG[2]

    def jointStateCallback(self, data):
        """
            обработчик пришедших показаний датчика из роса
        :param data: Структура роса с показаниями датчика
        """
        # созраняем пришедшее значение
        self.jointState = data
        self.G = self.getGravityTorques(self.jointState.effort, self.jointState.position)
        self.calculateOverG()
        self.getF()
        sum = 0
        logStr = "%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s,%s\n" % (
            time.time() - self.startTime,
            data.position[0], data.position[1], data.position[2], data.position[3], data.position[4],
            data.velocity[0], data.velocity[1], data.velocity[2], data.velocity[3], data.velocity[4],
            data.effort[0], data.effort[1], data.effort[2], data.effort[3], data.effort[4],
            self.task[0], self.task[1], self.task[2], self.task[3], self.task[4], self.outLogMode, self.outLogSubMode
        )

        self.outLog.write(logStr)

        if self.targetType == self.TARGET_TYPE_MANY_JOINTS:
            for i in range(5):
                delta = (self.targetJPoses[i] - data.position[i])
                sum += delta * delta

            sum = math.sqrt(sum) / 5
        elif self.targetType == self.TARGET_TYPE_ONE_JOINT:
            sum = abs(self.targetJPos - data.position[self.targetJposNum])
        elif self.targetType == self.TARGET_TYPE_FINGERS:
            for i in range(2):
                delta = (self.targetGPoses[i] - data.position[5 + i])
                sum += delta * delta

        if sum < 0.1 and self.targetType != self.TARGET_TYPE_NO_TARGET:
            self.targetType = self.TARGET_TYPE_NO_TARGET

        # print('left {}, right{}, target {}'.format(self.task[5], self.task[6], self.targetType))

    def __init__(self):
        """
            конструктор
        """
        # переменные для публикации в топики
        self.positionArmPub = rospy.Publisher("/arm_1/arm_controller/position_command",
                                              brics_actuator.msg.JointPositions, queue_size=1)
        self.torqueArmPub = rospy.Publisher("/arm_1/arm_controller/torques_command", brics_actuator.msg.JointTorques,
                                            queue_size=1)
        self.velocityArmPub = rospy.Publisher("/arm_1/arm_controller/velocity_command",
                                              brics_actuator.msg.JointVelocities, queue_size=1)
        self.cartVelPub = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.positionGripperPub = rospy.Publisher("/arm_1/gripper_controller/position_command",
                                                  brics_actuator.msg.JointPositions, queue_size=1)
        self.forceGripperPub = rospy.Publisher("/gripper_controller/force_command", brics_actuator.msg.JointTorques,
                                               queue_size=1)
        self.velocityGripperPub = rospy.Publisher("/gripper_controller/velocity_command",
                                                  brics_actuator.msg.JointVelocities, queue_size=1)
        self.jointStateSubscriber = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState,
                                                     self.jointStateCallback)
        dt = datetime.datetime.now()
        date = dt.strftime("%Y%m%d_%I%M%p")
        self.outLogMode = 'default'
        self.outLogSubMode = 'default'

        self.outLog = open('logs/' + date + '.csv', 'wb')

        logHead = "t,j1,j2,j3,j4,j5,w1,w2,w3,w4,w5,t1,t2,t3,t4,t5,task1,task2,task3,task4,task5,mode,submode\n"
        self.outLog.write(logHead)

        self.startTime = time.time()
        # пауза необходима для правильной обработки пакетов
        rospy.sleep(1)
        rospy.loginfo("Kuka created")

    def getUnitValue(self, tp):
        """
            получаем по типу топика размерность
        :param tp: Топик
        :return: размерность измеряемой величины
        """
        if tp == self.TYPE_JOINT_POSITIONS:  # положение джоинтов
            return "rad"
        elif tp == self.TYPE_JOINT_VELOCITIES:  # скорость джоинтов
            return "s^-1 rad"
        elif tp == self.TYPE_JOINT_TORQUES:  # силы джоинтов
            return "m^2 kg s^-2 rad^-1"
        elif tp == self.TYPE_GRIPPER_POSITION:  # положение гриппера
            return "m"
        elif tp == self.TYPE_GRIPPER_VELOCITY:  # сокрость гриппера
            return "s^-1 m"
        elif tp == self.TYPE_GRIPPER_TORQUE:  # сила гриппера
            return "m kg s^-2"
        else:
            return None

    def generateJoinVal(self, joint_num, val, tp):
        """
            генерируем сообщение по одному звену для роса
        :param joint_num: номер звена
        :param val: значение
        :param tp: единица измерения
        :return: сообщение по одному звену
        """
        # создаём сообщение джоинта
        jv = brics_actuator.msg.JointValue()
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        # имя джоинта
        jv.joint_uri = "arm_joint_" + str(joint_num)
        # размерность
        jv.unit = self.getUnitValue(tp)
        # непосредственное значение
        jv.value = val
        return jv

    def setCarrigeVel(self, x, y, z):
        """
            задать скорость каретке
        :param x, y: линейные перемещения
        :param z:поворот
        """
        # создаём сообщение
        msg = geometry_msgs.msg.Twist()
        # заполняем его данными
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = z
        # публикуем сообщение в топик
        self.cartVelPub.publish(msg)
        # выполняем задержку (ебаный рос)
        rospy.sleep(1)

    def setGripperPositions(self, leftG, rightG):
        """
            управляет положением гриппера в миллиметрах
        :param leftG:  положение левого пальца
        :param rightG: положение правого пальцев
        """
        # формируем сообщение положения джоинтов
        msg = brics_actuator.msg.JointPositions()
        # положения джоинтов - это список
        msg.positions = []
        # создаём объект описания жвижения жоинта
        jv = brics_actuator.msg.JointValue()
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        # тип объекта - левый палец
        jv.joint_uri = "gripper_finger_joint_l"
        # размерность
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_POSITION)
        # значение делим на 1000, так как масимальное смещение гриппера 110 мм
        jv.value = 0.01
        # добавляем объект в сообщение
        msg.positions.append(jv)
        # делаем тоже самое для правого пальца
        jv = brics_actuator.msg.JointValue()
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        jv.joint_uri = "gripper_finger_joint_r"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_POSITION)
        jv.value = 0.005
        msg.positions.append(jv)
        # публикуем сообщение в топике
        self.positionGripperPub.publish(msg)
        # делаем задержку
        rospy.sleep(1)

    def setGripperVelocities(self, leftG, rightG):
        """
            управляет скоростью гриппера в мм/с
        :param leftG:  скорость левого пальца
        :param rightG: скорость правого пальцев
        """
        msg = brics_actuator.msg.JointVelocities()
        msg.velocities = []
        jv = brics_actuator.msg.JointValue()
        jv.joint_uri = "gripper_finger_joint_l"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_VELOCITY)
        jv.value = leftG / 1000
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        msg.velocities.append(jv)
        jv = brics_actuator.msg.JointValue()
        jv.joint_uri = "gripper_finger_joint_r"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_VELOCITY)
        jv.value = rightG / 1000
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        msg.velocities.append(jv)
        self.velocityGripperPub.publish(msg)
        rospy.sleep(1)

    def setGripperTorques(self, leftG, rightG):
        """
            управляет силой гриппера в Н
        :param leftG:  сила левого пальца
        :param rightG: сила правого пальцев
        """
        msg = brics_actuator.msg.JointTorques()
        msg.torques = []
        jv = brics_actuator.msg.JointValue()
        jv.joint_uri = "gripper_finger_joint_l"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_TORQUE)
        jv.value = leftG
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        msg.torques.append(jv)
        jv = brics_actuator.msg.JointValue()
        jv.joint_uri = "gripper_finger_joint_r"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_TORQUE)
        jv.value = rightG
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        msg.torques.append(jv)
        self.forceGripperPub.publish(msg)
        rospy.sleep(1)

    def setRobotToCandle(self):
        """
            Поставить робота в свечку
        """
        self.setJointPositions(self.candlePos)

    def setJointPositions(self, joints):
        """
            задаём положения джоинтов в радианах
        :param joints: массив из пяти элементов с желаемыми положениями
        """
        msg = brics_actuator.msg.JointPositions()
        msg.positions = []
        # в цикле создаём объекты для сообщения, подробнее смотри setGripperPositions
        for i in range(5):
            j = joints[i]
            if j > self.jointsRange[i][1]:
                j = self.jointsRange[i][1]
            if j < self.jointsRange[i][0]:
                j = self.jointsRange[i][0]
            jv = self.generateJoinVal(i + 1, j, self.TYPE_JOINT_POSITIONS)
            msg.positions.append(jv)
        self.positionArmPub.publish(msg)
        self.targetJPoses = joints
        self.targetType = self.TARGET_TYPE_MANY_JOINTS

        rospy.sleep(1)

    def setJointPositionsImm(self, joints):
        """
            задаём положения джоинтов в радианах
        :param joints: массив из пяти элементов с желаемыми положениями
        """
        msg = brics_actuator.msg.JointPositions()
        msg.positions = []
        # в цикле создаём объекты для сообщения, подробнее смотри setGripperPositions
        for i in range(5):
            j = joints[i]
            if j > self.jointsRange[i][1]:
                j = self.jointsRange[i][1]
            if j < self.jointsRange[i][0]:
                j = self.jointsRange[i][0]
            jv = self.generateJoinVal(i + 1, j, self.TYPE_JOINT_POSITIONS)
            msg.positions.append(jv)
        self.positionArmPub.publish(msg)
        self.targetJPoses = joints
        self.targetType = self.TARGET_TYPE_MANY_JOINTS

    def setJointPosition(self, joint_num, value):
        """
             задаёт положение джоинта в радианах
        :param joint_num: номер джоинта
        :param value: угол поворота
        """
        # проверяем, что джоинт подходит
        if not joint_num in range(1, 5):
            rospy.logerror("Звено с номером " + str(joint_num) + " не определено")
            return

        msg = brics_actuator.msg.JointPositions()
        msg.positions = [self.generateJoinVal(joint_num, value, self.TYPE_JOINT_POSITIONS)]
        self.positionArmPub.publish(msg)

        rospy.sleep(1)

    def setJointTorques(self, joints):
        """
            задаёт моменты джоинтов в Н*м
        :param joints: массив из пяти элементов с желаемыми моментами
        """
        msg = brics_actuator.msg.JointTorques()
        msg.torques = []
        for i in range(5):
            jv = self.generateJoinVal(i + 1, joints[i], self.TYPE_JOINT_TORQUES)
            msg.torques.append(jv)
        self.torqueArmPub.publish(msg)
        rospy.sleep(1)

    def setJointTorque(self, joint_num, value):
        """
             задаёт момент джоинта в Н*м
        :param joint_num: номер джоинта
        :param value: желаемый момент
        """
        msg = brics_actuator.msg.JointTorques()
        msg.torques = [self.generateJoinVal(joint_num, value, self.TYPE_JOINT_TORQUES)]
        self.torqueArmPub.publish(msg)
        rospy.sleep(1)

    def setJointVelocity(self, joint_num, value):
        """
            задаёт скорости джоинтов в рады/с
        :param joints: массив из пяти элементов с желаемыми скорости
        """
        self.task[joint_num - 1] = value
        msg = brics_actuator.msg.JointVelocities()
        msg.velocities = [self.generateJoinVal(joint_num, value, self.TYPE_JOINT_VELOCITIES)]
        self.velocityArmPub.publish(msg)
        rospy.sleep(0.1)

    def setJointVelocities(self, joints):
        """
        задаёт скорость джоинта в рад/с
        :param joint_num: номер джоинта
        :param value: желаемая скорость
        """
        msg = brics_actuator.msg.JointVelocities()
        msg.velocities = []
        for i in range(5):
            jv = self.generateJoinVal(i + 1, joints[i], self.TYPE_JOINT_VELOCITIES)
            msg.velocities.append(jv)
        self.velocityArmPub.publish(msg)
        rospy.sleep(1)
