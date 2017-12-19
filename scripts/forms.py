#!/usr/bin/env python
# -*- coding: utf-8 -*-

import wx
import wx.grid as gridlib

from kukaController import KukaController


class Frame(wx.Frame):
    # закрытие формы
    def onClose(self, event):
        # останавливаем таймер
        self.timer.Stop()
        self.Close()

    # записать полученную от куки дату в таблицу
    def setDataToGrid(self, data):
        for i in range(len(data.position)):
            # if i < 5:
            #    s = self.kuka.jointOffsets[i]
            # else:
            s = 0
            self.myGrid.SetCellValue(i, 0, ("%.2f" % (data.position[i] + s)))
            self.myGrid.SetCellValue(i, 1, ("%.2f" % data.velocity[i]))
            self.myGrid.SetCellValue(i, 2, ("%.2f" % data.effort[i]))
        pass

    # получить углы из полей ввобда джоинтов
    def getJoinfFromText(self):
        return [
            float(self.j1PosTex.GetValue()),
            float(self.j2PosTex.GetValue()),
            float(self.j3PosTex.GetValue()),
            float(self.j4PosTex.GetValue()),
            float(self.j5PosTex.GetValue()),
        ]

    # управление джоинтами по положению
    def OnSendJPos(self, event):
        self.kuka.setJointPositions(self.getJoinfFromText())
        pass

    # управление джоинтами по скоростям
    def OnSendJVel(self, event):
        self.kuka.setJointVelocities(self.getJoinfFromText())
        pass

    # управление джоинтами по моментам
    def OnSendJTor(self, event):
        self.kuka.setJointTorques(self.getJoinfFromText())
        pass

    # управление гриппером по положению
    def OnSendGPos(self, event):
        self.kuka.setGripperPositions(float(self.glPosTex.GetValue()), float(self.grPosTex.GetValue()))
        pass

    # управление гриппером по положению
    def OnSendGVel(self, event):
        self.kuka.setGripperVelocities(float(self.glPosTex.GetValue()), float(self.grPosTex.GetValue()))
        pass

    # управление гриппером по скоростям
    def OnSendGTor(self, event):
        self.kuka.setGripperTorques(float(self.glPosTex.GetValue()), float(self.grPosTex.GetValue()))
        pass

    # управление джоинтами по моментам
    def OnSendCVel(self, event):
        self.kuka.setCarrigeVel(float(self.cxPosTex.GetValue()), float(self.cyPosTex.GetValue()),
                                float(self.czPosTex.GetValue()))
        pass

    def setDHChords(self):
        self.posXTex.Clear()
        self.posYTex.Clear()
        self.posZTex.Clear()
        ps = self.kuka.getEndEffectorPos()
        self.posXTex.AppendText(str(round(ps[0])))
        self.posYTex.AppendText(str(round(ps[1])))
        self.posZTex.AppendText(str(round(ps[2])))

    # события по таймеру
    def OnTimer(self, event):
        self.setDataToGrid(self.kuka.jointState)
        self.setDHChords()

        pass

        # события по таймеру

    def OnUpdateJPos(self, event):
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
        pass

    def OnZeroJPos(self, event):
        ps = self.kuka.jointState.position
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
        pass

    def OnRandomJPos(self, event):
        self.kuka.randomPoints(int(self.randomTex.GetValue()), 0.5)
        pass

    def OnStopJSpeed(self, event):
        self.kuka.setJointVelocities([0, 0, 0, 0, 0])
        pass

    def OnStopJTorque(self, event):
        self.kuka.setJointVelocities([0, 0, 0, 0, 0])
        pass

    def OnTest(self, event):
        print (self.kuka.getEndEffectorPos())

    def OnCandle(self, event):
        print (self.kuka.setRobotToCandle())

    def OnFriction(self, event):
        print (self.kuka.makeTrapezeSimpleCiclic(
            int(self.frictionJNumTex.GetValue()),
            float(self.frictionAngleEndTex.GetValue()),
            float(self.frictionMaxWTex.GetValue())
        ))

    def OnFullFriction(self, event):
        self.kuka.fullFriction()

    def OnZeroMoment(self, event):
        self.kuka.zeroMomentInJoint(int(self.zeroMomentTex.GetValue()))
        pass

    def OnWarmUp(self, event):
        self.kuka.warmUpLink(int(self.warmUpNumTex.GetValue()), float(self.warmUpTimeTex.GetValue()))
        pass

    def OnGravityFind(self,event):
        self.kuka.gravitationFind()

    # конструктор
    def __init__(self, parent=None, id=-1, title='', pos=(0, 0), size=(690, 900)):
        # создаём фрейм
        wx.Frame.__init__(self, parent, id, title, pos, size)
        # создаём объект для взаимодействия с роботом
        self.kuka = KukaController()
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
        # запускаем таймер
        self.timer = wx.Timer(self, -1)
        # раз в 0.5 секунды
        self.timer.Start(500)
        # указываем функцию, которая будет вызываться по таймеру
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)

    # инициализируем элементы управления
    def initControlItems(self):
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
        self.updateJposBtn = wx.Button(self.panel, label="Обновить", pos=(320, 220), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnUpdateJPos, self.updateJposBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.zeroJposBtn = wx.Button(self.panel, label="Обнулить", pos=(320, 260), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnZeroJPos, self.zeroJposBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.stopJspeedBtn = wx.Button(self.panel, label="Стоп", pos=(140, 300), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnStopJSpeed, self.stopJspeedBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.stopJtorqueBtn = wx.Button(self.panel, label="Стоп", pos=(140, 340), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnStopJTorque, self.stopJtorqueBtn)
        # конпки управления джоинтами и привязка методов к ним
        self.sendJposBtn = wx.Button(self.panel, label="Положения", pos=(30, 260), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendJPos, self.sendJposBtn)

        self.sendJvelBtn = wx.Button(self.panel, label="Скорости", pos=(30, 300), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendJVel, self.sendJvelBtn)
        self.sendJtorBtn = wx.Button(self.panel, label="Моменты", pos=(30, 340), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendJTor, self.sendJtorBtn)
        # управление гриппером
        wx.StaticText(self.panel, -1, "Гриппер: левый, правый", (30, 520))
        # создаём поля ввода для пальцев гриппера
        self.glPosTex = wx.TextCtrl(self.panel, -1, '0', pos=(30, 540), size=(40, 30))
        self.grPosTex = wx.TextCtrl(self.panel, -1, '0', pos=(80, 540), size=(40, 30))
        # задаём фон заливки
        self.glPosTex.SetBackgroundColour('#DDFFEE')
        self.grPosTex.SetBackgroundColour('#DDFFEE')
        # кнопки управления гриппером и привязка методов к ним
        self.sendGposBtn = wx.Button(self.panel, label="Положения", pos=(30, 570), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendGPos, self.sendGposBtn)
        self.sendGvelBtn = wx.Button(self.panel, label="Скорости", pos=(140, 570), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendGVel, self.sendGvelBtn)
        self.sendGtorBtn = wx.Button(self.panel, label="Моменты", pos=(250, 570), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendGTor, self.sendGtorBtn)
        # управление тележкой
        wx.StaticText(self.panel, -1, "Тележка: x,y,z", (30, 600))
        # создаём поля ввода для координат тележки
        self.cxPosTex = wx.TextCtrl(self.panel, -1, '0', pos=(30, 620), size=(40, 30))
        self.cyPosTex = wx.TextCtrl(self.panel, -1, '0', pos=(80, 620), size=(40, 30))
        self.czPosTex = wx.TextCtrl(self.panel, -1, '0', pos=(130, 620), size=(40, 30))
        # задаём фон заливки
        self.cxPosTex.SetBackgroundColour('#DDFFEE')
        self.cyPosTex.SetBackgroundColour('#DDFFEE')
        self.czPosTex.SetBackgroundColour('#DDFFEE')
        # кнопка управления тележкой и привязка метода к ней
        self.sendCvelBtn = wx.Button(self.panel, label="Положения", pos=(30, 650), size=(100, 30))
        self.Bind(wx.EVT_BUTTON, self.OnSendCVel, self.sendCvelBtn)

        self.randomJposBtn = wx.Button(self.panel, label="Рандом", pos=(70, 410), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnRandomJPos, self.randomJposBtn)
        self.randomTex = wx.TextCtrl(self.panel, -1, '0', pos=(30, 410), size=(40, 30))

        self.testBtn = wx.Button(self.panel, label="Тест", pos=(320, 300), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnTest, self.testBtn)

        self.posXTex = wx.TextCtrl(self.panel, -1, '0', pos=(30, 450), size=(80, 30))
        self.posYTex = wx.TextCtrl(self.panel, -1, '0', pos=(120, 450), size=(80, 30))
        self.posZTex = wx.TextCtrl(self.panel, -1, '0', pos=(210, 450), size=(80, 30))

        self.candleBtn = wx.Button(self.panel, label="В свечку", pos=(320, 330), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnCandle, self.candleBtn)

        self.frictionBtn = wx.Button(self.panel, label="Friction", pos=(320, 380), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnFriction, self.frictionBtn)

        self.frictionFullBtn = wx.Button(self.panel, label="FullFriction", pos=(440, 380), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnFullFriction, self.frictionFullBtn)

        self.frictionJNumTex = wx.TextCtrl(self.panel, -1, '1', pos=(320, 410), size=(80, 30))
        self.frictionAngleEndTex = wx.TextCtrl(self.panel, -1, '1', pos=(410, 410), size=(80, 30))
        self.frictionMaxWTex = wx.TextCtrl(self.panel, -1, '0.5', pos=(500, 410), size=(80, 30))

        self.zeroMomentBtn = wx.Button(self.panel, label="Нулевой момент", pos=(370, 450), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnZeroMoment, self.zeroMomentBtn)
        self.zeroMomentTex = wx.TextCtrl(self.panel, -1, '4', pos=(320, 450), size=(40, 30))

        self.warmUpBtn = wx.Button(self.panel, label="Разогрев", pos=(420, 490), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnWarmUp, self.warmUpBtn)
        self.warmUpNumTex = wx.TextCtrl(self.panel, -1, '4', pos=(320, 490), size=(40, 30))
        self.warmUpTimeTex = wx.TextCtrl(self.panel, -1, '0.5', pos=(370, 490), size=(40, 30))

        self.gravityFindBtn = wx.Button(self.panel, label="Гравитация", pos=(470, 650), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnGravityFind, self.gravityFindBtn)
        # управление гриппером
        wx.StaticText(self.panel, -1, "X:0 ", (440, 570))
        wx.StaticText(self.panel, -1, "Y:0 ", (490, 570))
        wx.StaticText(self.panel, -1, "Z:0 ", (540, 570))

    # инициализация таблицы
    def initGrid(self):
        # создаём таблицу
        self.myGrid = gridlib.Grid(self.panel)
        # кол-во строк и столбцов
        # нумерация в таблице начинается с ячейки (0,0)
        self.myGrid.CreateGrid(7, 3)
        # задаём имена столбцов
        self.myGrid.SetColLabelValue(2, "Момент")
        self.myGrid.SetColLabelValue(1, "Скорость")
        self.myGrid.SetColLabelValue(0, "Положение")
        # задаём ширину второго столбца больше, т.к. положение не "влезает"
        self.myGrid.SetColSize(2, 100)
        # задаём имена строк
        r = KukaController.jointsRange
        gr = KukaController.gripperRange
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
