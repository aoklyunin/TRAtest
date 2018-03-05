# coding=utf8

from math import sin, cos, pi
from numpy.linalg import pinv
from numpy import transpose

CANDLE_CONF = [2.9496, 1.1345, -2.5482, 1.7890, 2.9234]


def get_jacobian(position):
    q1 = position[0] - CANDLE_CONF[0]
    q2 = position[1] - CANDLE_CONF[1]
    q3 = position[2] - CANDLE_CONF[2]
    q4 = position[3] - CANDLE_CONF[3]
    # q5 = position[4] - CANDLE_CONF[4] – don't used

    # d1 = 0.147 – don't used
    # d5 = 0.19426 – don't used
    a1 = 0.033
    a2 = 0.155
    a3 = 0.135
    th2 = -pi / 2
    th4 = pi / 2
    al1 = -pi / 2
    al4 = pi / 2

    jacobian = [
        [-a1 * sin(q1), - a2 * sin(q2 + th2) * cos(q1) - a2 * cos(q2 + th2) * cos(al1) * sin(q1),
         - a3 * cos(q3) * (sin(q2 + th2) * cos(q1) + cos(q2 + th2) * cos(al1) * sin(q1)) - a3 * sin(q3) * (
                 cos(q2 + th2) * cos(q1) - sin(q2 + th2) * cos(al1) * sin(q1)), 0, 0],
        [a1 * cos(q1), a2 * cos(q2 + th2) * cos(al1) * cos(q1) - a2 * sin(q2 + th2) * sin(q1),
         - a3 * cos(q3) * (sin(q2 + th2) * sin(q1) - cos(q2 + th2) * cos(al1) * cos(q1)) - a3 * sin(q3) * (
                 cos(q2 + th2) * sin(q1) + sin(q2 + th2) * cos(al1) * cos(q1)), 0, 0],
        [0, a2 * cos(q2 + th2) * sin(al1),
         a3 * cos(q2 + th2) * cos(q3) * sin(al1) - a3 * sin(q2 + th2) * sin(al1) * sin(q3), 0, 0],
        [0, sin(al1) * sin(q1), sin(al1) * sin(q1), sin(al1) * sin(q1), cos(q4 + th4) * sin(al4) * (
                cos(q3) * (sin(q2 + th2) * cos(q1) + cos(q2 + th2) * cos(al1) * sin(q1)) + sin(q3) * (
                cos(q2 + th2) * cos(q1) - sin(q2 + th2) * cos(al1) * sin(q1))) + sin(q4 + th4) * sin(al4) * (
                 cos(q3) * (cos(q2 + th2) * cos(q1) - sin(q2 + th2) * cos(al1) * sin(q1)) - sin(q3) * (
                 sin(q2 + th2) * cos(q1) + cos(q2 + th2) * cos(al1) * sin(q1))) + cos(al4) * sin(al1) * sin(q1)],
        [0, -cos(q1) * sin(al1), -cos(q1) * sin(al1), -cos(q1) * sin(al1), cos(q4 + th4) * sin(al4) * (
                cos(q3) * (sin(q2 + th2) * sin(q1) - cos(q2 + th2) * cos(al1) * cos(q1)) + sin(q3) * (
                cos(q2 + th2) * sin(q1) + sin(q2 + th2) * cos(al1) * cos(q1))) + sin(q4 + th4) * sin(al4) * (
                 cos(q3) * (cos(q2 + th2) * sin(q1) + sin(q2 + th2) * cos(al1) * cos(q1)) - sin(q3) * (
                 sin(q2 + th2) * sin(q1) - cos(q2 + th2) * cos(al1) * cos(q1))) - cos(al4) * cos(q1) * sin(al1)],
        [1, cos(al1), cos(al1), cos(al1), cos(al1) * cos(al4) - cos(q4 + th4) * sin(al4) * (
                cos(q2 + th2) * cos(q3) * sin(al1) - sin(q2 + th2) * sin(al1) * sin(q3)) + sin(q4 + th4) * sin(al4) * (
                 cos(q2 + th2) * sin(al1) * sin(q3) + sin(q2 + th2) * cos(q3) * sin(al1))]
    ]

    return jacobian


def get_inversed_jacobian(position):
    return pinv(transpose(get_jacobian(position)))