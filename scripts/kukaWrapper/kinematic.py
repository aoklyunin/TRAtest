#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Модуль математических вычислений
"""

import numpy as np
from numpy import cos, sin


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


def getG(qList):
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

    return [0, 2.362139999011447302225668448683 * cos(q2 + q3 + 3.04) + 0.21958444029937820851650087661255 * sin(
        q2 + q3 + 3.04) + 3.8872332116227679269285753826946 * cos(q2 + 0.44) + 0.15687867946142067254733376557851 * sin(
        q2 + 0.44) - 1.0 * cos(q5 - 1.6) * (
                0.20351009246408627295643611887499 * cos(q2 + q3 + 3.04) * cos(
                    q4) - 0.20351009246408627295643611887499 * sin(
                    q2 + q3 + 3.04) * sin(q4)) + sin(q5 - 1.6) * (
                0.0071620057083474769621389555140922 * cos(q2 + q3 + 3.04) * cos(
                    q4) - 0.0071620057083474769621389555140922 * sin(
                    q2 + q3 + 3.04) * sin(q4)) + 0.94929910679663768348769507798846 * cos(q2 + q3 + 3.04) * sin(
        q4) + 0.94929910679663768348769507798846 * sin(q2 + q3 + 3.04) * cos(q4),
            2.3424057615965685719377868678759 * cos(q2 + q3 + 3.04) + 0.21958444029937820851650087661255 * sin(
                q2 + q3 + 3.04) - 0.32912006465472060101618012595281 * cos(
                q2 + q3 + q4 + 3.04) + 0.94929910679663768348769507798846 * sin(
                q2 + q3 + q4 + 3.04) - 0.20351009246408627295643611887499 * cos(q2 + q3 + q4 + 3.04) * cos(
                q5 - 1.6) + 0.0071620057083474769621389555140922 * cos(q2 + q3 + q4 + 3.04) * sin(q5 - 1.6),
            0.94929910679663768348769507798846 * sin(q2 + q3 + q4 + 3.04) - 0.32912006465472060101618012595281 * cos(
                q2 + q3 + q4 + 3.04) - 0.20351009246408627295643611887499 * cos(q2 + q3 + q4 + 3.04) * cos(
                q5 - 1.6) + 0.0071620057083474769621389555140922 * cos(q2 + q3 + q4 + 3.04) * sin(q5 - 1.6),
            0]


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