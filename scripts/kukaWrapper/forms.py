#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Родительский модуль для описания окна приложения
"""

from abc import ABCMeta, abstractmethod, abstractproperty

import wx
import wx.grid as gridlib

from kinematic import getG
from scripts.kukaWrapper.kukaWrapper import KukaWrapper


class Frame(wx.Frame):
    """
        Класс для описания окна приложения
    """

    @abstractmethod
    def initExpItems(self):
        """Инициализация элементов управления экспериментами"""

    @abstractmethod
    def OnTest(self, event):
        """Тестовый метод"""

    @abstractmethod
    def ExpTimer(self):
        """Таймер для экспериментов. Срабатывает раз в пол-секунды"""

    def OnCandle(self, event):
        """
        отправить робота в свечку
        """
        print (self.kuka.setRobotToCandle())

    def onClose(self, event):
        """
            закрытие формы
        """
        # останавливаем таймер
        self.timer.Stop()
        self.Close()

    def setDataToGrid(self, data):
        """
        записать полученную от куки дату в таблицу
        """
        for i in range(len(data.position)):
            # if i < 5:
            #    s = self.kuka.jointOffsets[i]
            # else:
            s = 0
            self.myGrid.SetCellValue(i, 0, ("%.2f" % (data.position[i] + s)))
            self.myGrid.SetCellValue(i, 1, ("%.2f" % data.velocity[i]))
            self.myGrid.SetCellValue(i, 2, ("%.2f" % data.effort[i]))
            if i < 5:
                self.myGrid.SetCellValue(i, 3, ("%.2f" % self.kuka.G[i]))
                self.myGrid.SetCellValue(i, 4, ("%.2f" % self.kuka.overG[i]))

    def setJacobianToGrid(self, data):
        """
        Обновить данные по силам и моментам на энд-эффекторе
        """
        for i in range(3):
            for j in range(5):
                self.jacobianGrid.SetCellValue(i, j, ("%.2f" % (data[i][j])))


    def getJoinfFromText(self):
        """
            получить углы из полей ввобда джоинтов
        :return:  массив обощённых координат робота
        """
        return [
            float(self.j1PosTex.GetValue()),
            float(self.j2PosTex.GetValue()),
            float(self.j3PosTex.GetValue()),
            float(self.j4PosTex.GetValue()),
            float(self.j5PosTex.GetValue()),
        ]

    def OnSendJPos(self, event):
        """
            управление положением по джоинтам
        """
        self.kuka.setJointPositions(self.getJoinfFromText())

    def OnSendJPosV(self, event):
        """
            управление положением по джоинтам c инжекцией вч сигнала на 10с
        """
        self.kuka.moveToConfV(self.getJoinfFromText(), 10)

    def OnSendJVel(self, event):
        """
            управление скоростями по джоинтам
        """
        self.kuka.setJointVelocities(self.getJoinfFromText())

    def OnSendJTor(self, event):
        """
        управление моментами по джоинтам
        """
        self.kuka.setJointTorques(self.getJoinfFromText())

    def OnSendGPos(self, event):
        """
        управление гриппером по положению
        """
        self.kuka.setGripperPositions(float(self.glPosTex.GetValue()), float(self.grPosTex.GetValue()))

    def OnSendGVel(self, event):
        """
            управление гриппером по положению
                """
        self.kuka.setGripperVelocities(float(self.glPosTex.GetValue()), float(self.grPosTex.GetValue()))

    def OnSendGTor(self, event):
        """
            управление гриппером по скоростям

                """
        self.kuka.setGripperTorques(float(self.glPosTex.GetValue()), float(self.grPosTex.GetValue()))

    def setDHChords(self):
        """
            Задать координаты в декартовом пространстве
                """
        self.posXTex.Clear()
        self.posYTex.Clear()
        self.posZTex.Clear()
        ps = self.kuka.getEndEffectorPos()
        self.posXTex.AppendText(str(round(ps[0])))
        self.posYTex.AppendText(str(round(ps[1])))
        self.posZTex.AppendText(str(round(ps[2])))

    def OnTimer(self, event):
        """
            события по таймеру 2Гц
                """
        self.setDataToGrid(self.kuka.jointState)
        self.setJacobianToGrid(self.kuka.wrenches)

        self.setDHChords()
        if self.kuka.checkCurPositionEnabled():
            self.posEnabledTex.SetLabel("Enabled")
        else:
            self.posEnabledTex.SetLabel("Disabled")
        self.ExpTimer()

    def OnUpdateJPos(self, event):
        """
         Обновить положения робота в полях ввода
                """
        ps = self.kuka.jointState.position
        self.j1PosTex.Clear()
        self.j2PosTex.Clear()
        self.j3PosTex.Clear()
        self.j4PosTex.Clear()
        self.j5PosTex.Clear()
        self.j1PosTex.AppendText(str(round(ps[0], 2)))
        self.j2PosTex.AppendText(str(round(ps[1], 2)))
        self.j3PosTex.AppendText(str(round(ps[2], 2)))
        self.j4PosTex.AppendText(str(round(ps[3], 2)))
        self.j5PosTex.AppendText(str(round(ps[4], 2)))

    def OnZeroJPos(self, event):
        """
            обнулить положения робота
                """
        self.j1PosTex.Clear()
        self.j2PosTex.Clear()
        self.j3PosTex.Clear()
        self.j4PosTex.Clear()
        self.j5PosTex.Clear()
        self.j1PosTex.AppendText("0")
        self.j2PosTex.AppendText("0")
        self.j3PosTex.AppendText("0")
        self.j4PosTex.AppendText("0")
        self.j5PosTex.AppendText("0")

    def OnStopKuka(self, event):
        """
            остановить движение робота
                """
        self.kuka.setJointVelocities([0, 0, 0, 0, 0])

    def OnMeasure(self, event):
        """
            Начать измерения
                """
        self.kuka.measure(duration=10, rate=30)

    def __init__(self, parent=None, id=-1, title='', pos=(0, 0), size=(690, 900)):
        """
            конструктор
        """
        # создаём фрейм
        wx.Frame.__init__(self, parent, id, title, pos, size)

        # добавляем на фрейм панель
        self.panel = wx.Panel(self)
        # инициализируем панель
        self.initGrid()
        # добавляем прокрутку на таблицу (пока что почему-то не работает, мб потому что таблица помещается полностью)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.myGrid)
        self.panel.SetSizer(self.sizer)
        # инициализируем элементы управления
        self.initControlItems()
        self.jacobianGrid()

        # запускаем таймер
        self.timer = wx.Timer(self, -1)
        # раз в 0.5 секунды
        self.timer.Start(500)
        # указываем функцию, которая будет вызываться по таймеру
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)

    def initControlItems(self):
        """
            инициализирует элементы управления
        """
        # Управление джоинтами
        wx.StaticText(self.panel, -1, "Джоинты", (30, 140))
        # создаём поля ввода для джоинтов
        self.j1PosTex = wx.TextCtrl(self.panel, -1, '0', pos=(30, 220), size=(50, 30))
        self.j2PosTex = wx.TextCtrl(self.panel, -1, '0', pos=(90, 220), size=(50, 30))
        self.j3PosTex = wx.TextCtrl(self.panel, -1, '0', pos=(150, 220), size=(50, 30))
        self.j4PosTex = wx.TextCtrl(self.panel, -1, '0', pos=(210, 220), size=(50, 30))
        self.j5PosTex = wx.TextCtrl(self.panel, -1, '0', pos=(270, 220), size=(50, 30))

        # задаём фон заливки
        self.j1PosTex.SetBackgroundColour('#DDFFEE')
        self.j2PosTex.SetBackgroundColour('#DDFFEE')
        self.j3PosTex.SetBackgroundColour('#DDFFEE')
        self.j4PosTex.SetBackgroundColour('#DDFFEE')
        self.j5PosTex.SetBackgroundColour('#DDFFEE')

        # конпки управления джоинтами и привязка методов к ним
        self.updateJposBtn = wx.Button(self.panel, label="Обновить", pos=(330, 220), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnUpdateJPos, self.updateJposBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.zeroJposBtn = wx.Button(self.panel, label="Обнулить", pos=(330, 260), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnZeroJPos, self.zeroJposBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.stopJspeedBtn = wx.Button(self.panel, label="Стоп", pos=(140, 340), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnStopKuka, self.stopJspeedBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.sendJposBtn = wx.Button(self.panel, label="Положения", pos=(30, 260), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendJPos, self.sendJposBtn)
        # выставить в положение с инжекций высокочастотного сигнала
        self.sendJposVBtn = wx.Button(self.panel, label="Положения (В)", pos=(140, 260), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendJPosV, self.sendJposVBtn)

        self.measureBtn = wx.Button(self.panel, label="Измерения (В)", pos=(140, 300), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnMeasure, self.measureBtn)

        self.sendJvelBtn = wx.Button(self.panel, label="Скорости", pos=(30, 300), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendJVel, self.sendJvelBtn)
        self.sendJtorBtn = wx.Button(self.panel, label="Моменты", pos=(30, 340), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendJTor, self.sendJtorBtn)

        # Положение в Декартовом пространстве
        wx.StaticText(self.panel, -1, "X", (340, 310))
        wx.StaticText(self.panel, -1, "Y", (430, 310))
        wx.StaticText(self.panel, -1, "Z", (520, 310))
        self.posXTex = wx.TextCtrl(self.panel, -1, '0', pos=(330, 340), size=(70, 30))
        self.posYTex = wx.TextCtrl(self.panel, -1, '0', pos=(420, 340), size=(70, 30))
        self.posZTex = wx.TextCtrl(self.panel, -1, '0', pos=(510, 340), size=(70, 30))
        self.posEnabledTex = wx.StaticText(self.panel, -1, "Enabled", (595, 345))

        # удобные кнопочки
        self.testBtn = wx.Button(self.panel, label="Тест", pos=(480, 220), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnTest, self.testBtn)
        self.candleBtn = wx.Button(self.panel, label="В свечку", pos=(480, 260), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnCandle, self.candleBtn)

        # добавляем на фрейм панель
        self.expPanel = wx.Panel(self.panel, pos=(0, 400), size=(690, 500))
        # добавляем элементы из эксперимента
        self.initExpItems()

    def jacobianGrid(self):
        """
        """
        # создаём таблицу
        self.jacobianGrid = gridlib.Grid(self.expPanel, pos=(30, 350), size=(690, 120))
        # кол-во строк и столбцов
        # нумерация в таблице начинается с ячейки (0,0)
        self.jacobianGrid.CreateGrid(3, 5)
        # задаём имена столбцов
        self.jacobianGrid.SetColLabelValue(4, "M2")
        self.jacobianGrid.SetColLabelValue(3, "M1")
        self.jacobianGrid.SetColLabelValue(2, "Fz")
        self.jacobianGrid.SetColLabelValue(1, "Fy")
        self.jacobianGrid.SetColLabelValue(0, "Fx")

        self.jacobianGrid.SetRowLabelValue(0, "EE")
        self.jacobianGrid.SetRowLabelValue(1, "EE internal")
        self.jacobianGrid.SetRowLabelValue(2, "EE external")

    def initGrid(self):
        """
            инициализация таблицу
        """
        # создаём таблицу
        self.myGrid = gridlib.Grid(self.panel)
        # кол-во строк и столбцов
        # нумерация в таблице начинается с ячейки (0,0)
        self.myGrid.CreateGrid(7, 5)
        # задаём имена столбцов
        self.myGrid.SetColLabelValue(4, "Ошибка")
        self.myGrid.SetColLabelValue(3, "Расчёт")
        self.myGrid.SetColLabelValue(2, "Момент")
        self.myGrid.SetColLabelValue(1, "Скорость")
        self.myGrid.SetColLabelValue(0, "Положение")
        # задаём ширину второго столбца больше, т.к. положение не "влезает"
        self.myGrid.SetColSize(2, 100)
        # задаём имена строк
        r = KukaWrapper.jointsRange
        gr = KukaWrapper.gripperRange
        self.myGrid.SetRowLabelValue(0, "arm_joint_1       (" + str(round(r[0][0], 2)) + "," + str(
            round(r[0][1], 2)) + ")")
        self.myGrid.SetRowLabelValue(1, "arm_joint_2        (" + str(round(r[1][0], 2)) + "," + str(
            round(r[1][1], 2)) + ")")
        self.myGrid.SetRowLabelValue(2, "arm_joint_3          (" + str(round(r[2][0], 2)) + "," + str(
            round(r[2][1], 2)) + ")")
        self.myGrid.SetRowLabelValue(3, "arm_joint_4        (" + str(round(r[3][0], 2)) + "," + str(
            round(r[3][1], 2)) + ")")
        self.myGrid.SetRowLabelValue(4, "arm_joint_5        (" + str(round(r[4][0], 2)) + "," + str(
            round(r[4][1], 2)) + ")")
        self.myGrid.SetRowLabelValue(5, "gripper_joint_r (" + str(round(gr[0], 2)) + "," + str(round(gr[1], 2)) + ")")
        self.myGrid.SetRowLabelValue(6, "gripper_joint_l (" + str(round(gr[0], 2)) + "," + str(round(gr[1], 2)) + ")")
        # задаём ширину столбца с именами строк
        self.myGrid.SetRowLabelSize(230)
