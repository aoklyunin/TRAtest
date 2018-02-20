#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Модуль математических вычислений
"""

import numpy as np
from numpy import cos, sin


def sign(a):
    if a > 0:
        return 1
    elif a < 0:
        return -1
    return 0


def getDHMatrix(alpha, a, d, theta):
    """
        Возвращает матрицу преобразования
    :param alpha, a, d, theta: DH параметры
    :return: Матрица перехода  4х4
    """
    return np.matrix(
        [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
         [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
         [0, np.sin(alpha), np.cos(alpha), d],
         [0, 0, 0, 1]
         ])


def getG(qList, tList):
    """
        Значение моментов по положению
    :param qList: массив из пяти обощённых координат
    :return: массив из пяти моментов
    """
    q1 = qList[0]
    q2 = qList[1]
    q3 = qList[2]
    q4 = qList[3]
    q5 = qList[4]

    t1 = sign(tList[0])
    t2 = sign(tList[1])
    t3 = sign(tList[2])
    t4 = sign(tList[3])
    t5 = sign(tList[4])

    return [0,
            0.4998 * t2 + 0.6754 * cos(q2 + q3 + q4) - 1.647 * sin(q2 + q3 + q4) - 0.06341 * cos(
                q2 + q3 + q4 + q5) + 0.06085 * sin(q2 + q3 + q4 + q5) - 2.94 * cos(q2 + q3) + 0.06341 * cos(
                q2 + q3 + q4 - 1.0 * q5) - 0.5982 * sin(q2 + q3) + 0.06085 * sin(q2 + q3 + q4 - 1.0 * q5) + 3.753 * cos(
                q2) - 2.093 * sin(q2),
            0.6754 * cos(q2 + q3 + q4) - 0.01009 * t3 - 1.647 * sin(q2 + q3 + q4) - 0.06341 * cos(
                q2 + q3 + q4 + q5) + 0.06085 * sin(q2 + q3 + q4 + q5) - 2.94 * cos(q2 + q3) + 0.06341 * cos(
                q2 + q3 + q4 - 1.0 * q5) - 0.5982 * sin(q2 + q3) + 0.06085 * sin(q2 + q3 + q4 - 1.0 * q5),
            0.094 * t4 + 0.6754 * cos(q2 + q3 + q4) - 1.647 * sin(q2 + q3 + q4) - 0.06341 * cos(
                q2 + q3 + q4 + q5) + 0.06085 * sin(q2 + q3 + q4 + q5) + 0.06341 * cos(
                q2 + q3 + q4 - 1.0 * q5) + 0.06085 * sin(
                q2 + q3 + q4 - 1.0 * q5),
            0.3248 * t5 - 0.06341 * cos(q2 + q3 + q4 + q5) + 0.06085 * sin(q2 + q3 + q4 + q5) - 0.06341 * cos(
                q2 + q3 + q4 - 1.0 * q5) - 0.06085 * sin(q2 + q3 + q4 - 1.0 * q5)
            ]


def getNewG(qList):
    """
        Значение моментов по положению
    :param qList: массив из пяти обощённых координат
    :return: массив из пяти моментов
    """
    q1 = qList[0]
    q2 = qList[1]
    q3 = qList[2]
    q4 = qList[3]
    q5 = qList[4]
    theta = [
        1.5905,
        -2.2143,
        0.4223,
        0.0216,
        0.5406,
        0.8684,
        0.0093,
        0.0313,
        0.1889,
        0.1109,
        0.0536,
        0.0585
    ]

    W = [[-sin(q2), cos(q2 + q3), -sin(q2 + q3), 155 * cos(q2), cos(q2 + q3 + q4), -sin(q2 + q3 + q4),
          - sin(q2 + q3 + q4 + q5) / 2 - sin(q2 + q3 + q4 - q5) / 2,
          cos(q2 + q3 + q4 - q5) / 2 - cos(q2 + q3 + q4 + q5) / 2],
         [0, cos(q2 + q3), -sin(q2 + q3), 0, cos(q2 + q3 + q4), -sin(q2 + q3 + q4),
          - sin(q2 + q3 + q4 + q5) / 2 - sin(q2 + q3 + q4 - q5) / 2,
          cos(q2 + q3 + q4 - q5) / 2 - cos(q2 + q3 + q4 + q5) / 2],
         [0, 0, 0, 0, cos(q2 + q3 + q4), -sin(q2 + q3 + q4),
          - sin(q2 + q3 + q4 + q5) / 2 - sin(q2 + q3 + q4 - q5) / 2,
          cos(q2 + q3 + q4 - q5) / 2 - cos(q2 + q3 + q4 + q5) / 2],
         [0, 0, 0, 0, 0, 0, sin(q2 + q3 + q4 - q5) / 2 - sin(q2 + q3 + q4 + q5) / 2,
          - cos(q2 + q3 + q4 + q5) / 2 - cos(q2 + q3 + q4 - q5) / 2]]
