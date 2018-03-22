#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import brics_actuator.msg
from threading import Thread
from scripts.kukaWrapper.kinematic import getDHMatrix


class KukaThreaded(Thread):
    # константы типов сообщений
    TYPE_JOINT_POSITIONS = 0  # положение джоинта
    TYPE_JOINT_VELOCITIES = 1  # скорость джоинта
    TYPE_JOINT_TORQUES = 2  # момент джоинта
    TYPE_GRIPPER_POSITION = 3  # положение гриппера
    TYPE_GRIPPER_VELOCITY = 4  # скорость гриппера
    TYPE_GRIPPER_TORQUE = 5  # сила гриппера

    jointsRange = [
        [0.011, 5.840],
        [0.011, 2.617],
        [-5.0, -0.02],
        [0.03, 3.42],
        [0.15, 5.641],
    ]

    jointOffsets = [
        -170.0 / 180 * math.pi,
        -65.0 / 180 * math.pi,
        (90 + 60.0) / 180 * math.pi,
        0.0 / 180 * math.pi,
        -90.0 / 180 * math.pi
    ]

    def __init__(self, position, duration):
        Thread.__init__(self)

        self.positionArmPub = rospy.Publisher("/arm_1/arm_controller/position_command",
                                              brics_actuator.msg.JointPositions, queue_size=1, tcp_nodelay=True)

        self.position = position
        self.duration = duration

    def run(self):
        if not self.checkPositionJEnabled(self.position):
            return False

        self.setJointPositions(self.position)
        rospy.sleep(1)
        self.injectHFS(self, self.position, self.position, rate=30)

        return True

    def checkPositionXYZEnable(self, pos):
        x = pos[0]
        y = pos[1]
        z = pos[2]
        if z < 200:
            return False
        elif z < 300:
            return x ** 2 + y ** 2 > 350 ** 2

        return True

    def getDH(self, position):
        DH1 = getDHMatrix(math.pi / 2, self.L[0], self.L[1], position[0] + self.jointOffsets[0])
        DH2 = getDHMatrix(0, self.L[2], 0, position[1] + self.jointOffsets[1] + math.pi / 2)
        DH3 = getDHMatrix(0, self.L[3], 0, position[2] + self.jointOffsets[2])
        DH4 = getDHMatrix(math.pi / 2, 0, 0, position[3] + self.jointOffsets[3])
        DH5 = getDHMatrix(0, 0, self.L[4], position[4] + self.jointOffsets[4])
        return DH1 * DH2 * DH3 * DH4 * DH5

    def getEndEffectorPosByJ(self, joints):
        """
            Возвращает положение энд-эффектора по обобщённым координтатам
        :param joints:  массив из пяти элементов с обобщёнными координатами робота
        :return: Массив декартовых координат [x,y,z]
        """
        tf = self.getDH(joints)
        return [tf.item(0, 3), tf.item(1, 3), tf.item(2, 3)]

    def checkPositionJEnabled(self, joints):
        """
             проверка, что положение допустимо
        :param joints: обобщённые координаты
        :return: True/False
        """
        xyz = self.getEndEffectorPosByJ(joints)
        # print(xyz)
        return self.checkPositionXYZEnable(xyz)

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

    def setJointPositions(self, joints):
        """
            задаём положения джоинтов в радианах
        :param joints: массив из пяти элементов с желаемыми положениями
        """
        msg = brics_actuator.msg.JointPositions()
        msg.positions = []

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

    def injectHFS(self, j, duration, rate):
        print('Injecting hf-signal at rate {:d}Hz: on'.format(rate))
        r = rospy.Rate(rate)

        for i in range(duration * rate):
            m = float(-1 if i % 2 else 1)
            n = [m / 4000, m / 8000, m / 4000, m / 2000, m / 2000]
            pos = [j[k] + n[k] for k in range(5)]
            if i % 100 == 1:
                print(n)

            self.setJointPositionsImm(pos)

            r.sleep()

        print('Injecting hf-signal: off')