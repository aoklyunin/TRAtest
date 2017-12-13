#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv
import math
import random

import numpy as np

import datetime
import tkMessageBox
from wx import wx

import rospy
import brics_actuator.msg
import geometry_msgs.msg
import sensor_msgs.msg

# ToDO
# проверка, что задание по положению джоинтов не выходят за границы
# подумать, как останавливать управление по силам, если моменты обнулить, то робот прото упадёт
#   -можно попробовать задавать скорости в 0
import time

from kinematic import getDHMatrix


class KukaController:
    # масимальная скорость руки
    maxArmVelocity = math.radians(90)
    # диапазон допустимых значений джоинтов
    jointsRange = [
        [0.011, 5.840],
        [0.011, 2.617],
        [-5.0, -0.02],
        [0.03, 3.42],
        [0.15, 5.641],
    ]
    L = [33, 147, 155, 135, 217.5]

    TARGET_TYPE_MANY_JOINTS = 0
    TARGET_TYPE_ONE_JOINT = 1
    TARGET_TYPE_NO_TARGET = -1

    targetJPoses = [0, 0, 0, 0, 0]
    targetJPos = 0
    targetJposNum = -1
    targetType = TARGET_TYPE_NO_TARGET

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

    reachedAction = False

    def warn(self, message, caption='Ае!'):
        dlg = wx.MessageDialog(None, message, caption, wx.OK | wx.ICON_WARNING)
        dlg.ShowModal()
        dlg.Destroy()

    # трапеция номер джоина, целевое положение, максимальная скорость, ускорение
    def makeTrapeze(self, jointNum, arange, maxW, betta):
        angleStart = self.jointState.position[jointNum - 1]
        accAngle = maxW ** 2 / (2 * (betta ** 2))
        print(accAngle)
        curPos = angleStart

        # влев
        while curPos - angleStart < arange:
            curPos = self.jointState.position[jointNum - 1]
            delta = curPos - angleStart
            if delta <= accAngle:
                w = maxW * delta / accAngle
            elif delta >= arange - accAngle:
                w = (arange - delta) / accAngle * maxW
            else:
                w = maxW
            if w < 0.01:
                w = 0.01
            print("_")
            print(w)
            print(curPos - angleStart)
            self.setJointVelocity(jointNum, w)

        self.setJointVelocity(jointNum, 0)
        while curPos - angleStart > -arange:
            curPos = self.jointState.position[jointNum - 1]
            if curPos > angleStart:
                delta = curPos - angleStart
                if delta <= accAngle:
                    w = maxW * delta / accAngle
                elif delta >= arange - accAngle:
                    w = (arange - delta) / accAngle * maxW
                else:
                    w = maxW
            else:
                delta = angleStart - curPos
                if delta <= accAngle:
                    w = maxW * delta / accAngle
                elif delta >= arange - accAngle:
                    w = (arange - delta) / accAngle * maxW
                else:
                    w = maxW

            if w < 0.01:
                w = 0.01

            print("_")
            print(w)
            print(curPos - angleStart)
            self.setJointVelocity(jointNum, -w)

        self.setJointVelocity(jointNum, 0)

        # в начало
        while curPos < angleStart:
            curPos = self.jointState.position[jointNum - 1]

            delta = angleStart - curPos
            if delta <= accAngle:
                w = maxW * delta / accAngle
            elif delta >= arange - accAngle:
                w = (arange - delta) / accAngle * maxW
            else:
                w = maxW

            if w < 0.01:
                w = 0.01

            print("_")
            print(w)
            print(angleStart-curPos)
            self.setJointVelocity(jointNum, w)

        self.setJointVelocity(jointNum, 0)


    def checkPositionXYZEnable(self, pos):
        x = pos[0]
        y = pos[1]
        z = pos[2]
        if abs(x) < 90 or abs(y) < 90:
            return False
        if z < 80:
            return False

        return True

    def checkPositionJEnabled(self, joints):
        xyz = self.getEndEffectorPosByJ(joints)
        return self.checkPositionXYZEnable(xyz)

    def randomPoints(self, n):
        for i in range(n):
            print(i)
            time.sleep(7)
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

            while self.targetType == self.TARGET_TYPE_MANY_JOINTS:
                time.sleep(1)

        self.warn("Робот отработал все точки")

    def getDH(self, position):
        DH1 = getDHMatrix(math.pi / 2, self.L[0], self.L[1], position[0] + self.jointOffsets[0])
        DH2 = getDHMatrix(0, self.L[2], 0, position[1] + self.jointOffsets[1] + math.pi / 2)
        DH3 = getDHMatrix(0, self.L[3], 0, position[2] + self.jointOffsets[2])
        DH4 = getDHMatrix(math.pi / 2, 0, 0, position[3] + self.jointOffsets[3])
        DH5 = getDHMatrix(0, 0, self.L[4], position[4] + self.jointOffsets[4])
        return DH1 * DH2 * DH3 * DH4 * DH5

    # матрица преобразования
    def getTF(self):
        return self.getDH(self.jointState.position)

    def getEndEffectorPos(self):
        tf = self.getTF()
        return [tf.item(0, 3), tf.item(1, 3), tf.item(2, 3)]

    def getEndEffectorPosByJ(self, joints):
        tf = self.getDH(joints)
        return [tf.item(0, 3), tf.item(1, 3), tf.item(2, 3)]

    def checkIfListIsZero(self, lst):
        flg = True
        for i in range(len(lst)):
            if lst[i] != 0:
                flg = False
        return flg

    # обработчик пришедших значенийitem
    def jointStateCallback(self, data):
        # созраняем пришедшее значение
        self.jointState = data
        sum = 0
        if self.targetType == self.TARGET_TYPE_MANY_JOINTS:
            for i in range(5):
                delta = (self.targetJPoses[i] - data.position[i])
                sum += delta * delta
            self.spamwriter.writerow((time.time(),) + data.position + data.velocity + data.effort)
            sum = math.sqrt(sum) / 5
        elif self.targetType == self.TARGET_TYPE_ONE_JOINT:
            sum = abs(self.targetJPos - data.position[self.targetJposNum])

        if sum < 0.1 and self.targetType != self.TARGET_TYPE_NO_TARGET:
            self.targetType = self.TARGET_TYPE_NO_TARGET

    # конструктор
    def __init__(self):
        # переменные для публикации в топики
        self.positionArmPub = rospy.Publisher("/arm_1/arm_controller/position_command",
                                              brics_actuator.msg.JointPositions, queue_size=1)
        self.torqueArmPub = rospy.Publisher("/arm_1/arm_controller/torques_command", brics_actuator.msg.JointTorques,
                                            queue_size=1)
        self.velocityArmPub = rospy.Publisher("/arm_1/arm_controller/velocity_command",
                                              brics_actuator.msg.JointVelocities, queue_size=1)
        self.cartVelPub = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.positionGripperPub = rospy.Publisher("/gripper_controller/position_command",
                                                  brics_actuator.msg.JointPositions, queue_size=1)
        self.forceGripperPub = rospy.Publisher("/gripper_controller/force_command", brics_actuator.msg.JointTorques,
                                               queue_size=1)
        self.velocityGripperPub = rospy.Publisher("/gripper_controller/velocity_command",
                                                  brics_actuator.msg.JointVelocities, queue_size=1)
        self.jointStateSubscriber = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState,
                                                     self.jointStateCallback)
        dt = datetime.datetime.now()
        date = dt.strftime("%d_%m_%Y_%I_%M%p")
        self.outLog = open('logs/' + date + '.csv', 'wb')
        self.spamwriter = csv.writer(self.outLog, delimiter=' ',
                                     escapechar=' ', quoting=csv.QUOTE_NONE)
        # пауза необходима для правильной обработки пакетов
        rospy.sleep(1)
        rospy.loginfo("Kuka created")

    # подогнать джоинт так, чтобы момент в нём был нулевым(работает коряво, впадлу пока допиливать)
    def zeroMomentInJoint(self, j):
        for i in range(10):
            curPos = self.jointState.position[j - 1]
            curMoment = self.jointState.effort[j - 1]
            posStar = curMoment / 2 + curPos
            print(curPos)
            print(posStar)
            self.setJointPosition(j, posStar)
            self.targetType = self.TARGET_TYPE_ONE_JOINT
            self.targetJPos = posStar
            self.targetJposNum = j
            rospy.sleep(2)

    # получаем по типу топика размерность
    def getUnitValue(self, tp):
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

    # по номеру джоинта, значению и типу генерируем сообщение
    def generateJoinVal(self, joint_num, val, tp):
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

    # задать скорость каретке x, y - линейные перемещений, z - поворот
    def setCarrigeVel(self, x, y, z):
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

    # управляем положением гриппера в миллиметрах, положение левого и правого пальцев
    def setGripperPositions(self, leftG, rightG):
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
        jv.value = leftG / 1000
        # добавляем объект в сообщение
        msg.positions.append(jv)
        # делаем тоже самое для правого пальца
        jv = brics_actuator.msg.JointValue()
        # получаем текущее время
        jv.timeStamp = rospy.Time.now()
        jv.joint_uri = "gripper_finger_joint_r"
        jv.unit = self.getUnitValue(self.TYPE_GRIPPER_POSITION)
        jv.value = rightG / 1000
        msg.positions.append(jv)
        # публикуем сообщение в топике
        self.positionGripperPub.publish(msg)
        # делаем задержку
        rospy.sleep(1)

    # управляем скоростью гриппера - тоже самое, что и setGripperPositions
    def setGripperVelocities(self, leftG, rightG):
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

    # управляем силой гриппера - тоже самое, что и setGripperPositions
    def setGripperTorques(self, leftG, rightG):
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

    # Поставить робота в свечку
    def setRobotToCandle(self):
        joints = [2.01, 1.09, -2.44, 1.74, 2.96]
        self.setJointPositions(joints)

    # задаём положения всех джоинтов
    def setJointPositions(self, joints):
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

    # тоже самое, что и setJointPositions, но управляем только одним джоинтом
    def setJointPosition(self, joint_num, value):
        # проверяем, что джоинт подходит
        if not joint_num in range(1, 5):
            rospy.logerror("Звено с номером " + str(joint_num) + " не определено")
            return

        msg = brics_actuator.msg.JointPositions()
        msg.positions = [self.generateJoinVal(joint_num, value, self.TYPE_JOINT_POSITIONS)]
        self.positionArmPub.publish(msg)

        rospy.sleep(1)

    # задаём моменты всех джоинтов подробнее смотри setJointPositions
    def setJointTorques(self, joints):
        msg = brics_actuator.msg.JointTorques()
        msg.torques = []
        for i in range(5):
            jv = self.generateJoinVal(i + 1, joints[i], self.TYPE_JOINT_TORQUES)
            msg.torques.append(jv)
        self.torqueArmPub.publish(msg)
        rospy.sleep(1)

    # задаём момент конкретному джоинту подробнее смотри setJointPosition
    def setJointTorque(self, joint_num, value):
        msg = brics_actuator.msg.JointTorques()
        msg.torques = [self.generateJoinVal(joint_num, value, self.TYPE_JOINT_TORQUES)]
        self.torqueArmPub.publish(msg)
        rospy.sleep(1)

    # задаём скорости всех джоинтов подробнее смотри setJointPositions
    def setJointVelocity(self, joint_num, value):
        msg = brics_actuator.msg.JointVelocities()
        msg.velocities = [self.generateJoinVal(joint_num, value, self.TYPE_JOINT_VELOCITIES)]
        self.velocityArmPub.publish(msg)
        rospy.sleep(0.1)

    # задаём скорость конкретному джоинту подробнее смотри setJointPosition
    def setJointVelocities(self, joints):
        msg = brics_actuator.msg.JointVelocities()
        msg.velocities = []
        for i in range(5):
            jv = self.generateJoinVal(i + 1, joints[i], self.TYPE_JOINT_VELOCITIES)
            msg.velocities.append(jv)
        self.velocityArmPub.publish(msg)
        rospy.sleep(1)

    def checkJPosSuccess(self):
        # self.targetJPoses
        pass
