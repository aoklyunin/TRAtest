# coding=utf8

from math import sin, cos
from numpy.linalg import inv
from numpy import transpose, dot

from jacob1 import jacob1
from jacob2 import jacob2
from jacob3 import jacob3

from consts import th2, th4, al1, al4, CANDLE_CONF


def get_jacobian(position):
    q1 = position[0] - CANDLE_CONF[0]
    q2 = position[1] - CANDLE_CONF[1]
    q3 = position[2] - CANDLE_CONF[2]
    q4 = position[3] - CANDLE_CONF[3]
    q5 = position[4] - CANDLE_CONF[4]

    R = cos(al1) * cos(al4) - cos(q4 + th4) * sin(al4) * (
    cos(q2 + th2) * cos(q3) * sin(al1) - sin(q2 + th2) * sin(al1) * sin(q3)) + sin(q4 + th4) * sin(al4) * (
    cos(q2 + th2) * sin(al1) * sin(q3) + sin(q2 + th2) * cos(q3) * sin(al1))

    if R < 1:
        return jacob1(q1, q2, q3, q4, q5)
    elif R == 1:
        return jacob2(q1, q2, q3, q4, q5)
    elif R == -1:
        return jacob3(q1, q2, q3, q4, q5)
    else:
        raise ValueError('Jacobian problem')


def get_inversed_jacobian(position):
    J = get_jacobian(position)

    rJ = dot([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ], J)

    return inv(transpose(rJ))