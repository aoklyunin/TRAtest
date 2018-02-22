#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
    Модуль для создания окна
"""

from scripts.kukaController import KukaController
from scripts.kukaWrapper.forms import Frame
import wx


class KukaFrame(Frame):
    """
        Класс графического окна
    """

    def __init__(self, parent=None, id=-1, title=''):
        """
                Инициализация элементов панели self.expPanel
                Ширина 690, Высота 500

        """
        Frame.__init__(self, parent, id, title)
        # создаём объект для взаимодействия с роботом
        self.kuka = KukaController()

    def initExpItems(self):
        """
            Инициализация элементов окна
        """
        # трапеции
        self.frictionBtn = wx.Button(self.expPanel, label="Friction", pos=(30, 10), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnFriction, self.frictionBtn)

        self.frictionFullBtn = wx.Button(self.expPanel, label="FullFriction", pos=(30, 50), size=(120, 30))
        self.Bind(wx.EVT_BUTTON, self.OnFullFriction, self.frictionFullBtn)

        self.frictionJNumTex = wx.TextCtrl(self.expPanel, -1, '1', pos=(170, 50), size=(80, 30))
        self.frictionAngleEndTex = wx.TextCtrl(self.expPanel, -1, '1', pos=(270, 50), size=(80, 30))
        self.frictionMaxWTex = wx.TextCtrl(self.expPanel, -1, '0.5', pos=(370, 50), size=(80, 30))

        wx.StaticText(self.expPanel, -1, "Joint", (180, 20))
        wx.StaticText(self.expPanel, -1, "Angle", (280, 20))
        wx.StaticText(self.expPanel, -1, "MaxW", (380, 20))

        # поиск нулевого момента
        self.zeroMomentBtn = wx.Button(self.expPanel, label="Find Zero Moment", pos=(500, 10), size=(170, 30))
        self.Bind(wx.EVT_BUTTON, self.OnFindZeroMoment, self.zeroMomentBtn)
        self.zeroMomentTex = wx.TextCtrl(self.expPanel, -1, '4', pos=(570, 50), size=(40, 30))
        wx.StaticText(self.expPanel, -1, "Joint", (530, 55))

        # разогрев
        self.warmUpBtn = wx.Button(self.expPanel, label="Разогрев", pos=(30, 100), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnWarmUp, self.warmUpBtn)
        self.warmUpNumTex = wx.TextCtrl(self.expPanel, -1, '4', pos=(42, 150), size=(40, 30))
        self.warmUpTimeTex = wx.TextCtrl(self.expPanel, -1, '0.5', pos=(112, 150), size=(40, 30))
        wx.StaticText(self.expPanel, -1, "Joint", (45, 130))
        wx.StaticText(self.expPanel, -1, "Time", (115, 130))

        # поиск гравитации
        self.gravityFindBtn = wx.Button(self.expPanel, label="Gravity Exp", pos=(310, 120), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnGravityFind, self.gravityFindBtn)

        # поиск гравитации
        self.randomConfBtn = wx.Button(self.expPanel, label="Random Conf", pos=(310, 170), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnRandomConf, self.randomConfBtn)

        # Режим Force-контроля
        self.ForceControlBtn = wx.Button(self.expPanel, label="ForceControl", pos=(470, 120), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnForceControl, self.ForceControlBtn)

        # Режим Force-контроля
        self.WarmUpBtn = wx.Button(self.expPanel, label="Warm up", pos=(470, 170), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnWarmUPRobot, self.WarmUpBtn)



    def ExpTimer(self):
        """
            Таймер, срабатывает два раза в секунду. Всё, что сюда напишешь будет выполняться каждые 500 мсек
        """


        pass

    def OnTest(self, event):
        """
            тестовая функция(можешь написать, что угодно, оно будет срабатывать каждый раз, когда ты жмёшь кнопку "Тест")
        """
        print ("Test Button clicked")

    def OnFriction(self, event):
        """
            Эксперимент по трению с одним звеном
        """
        print (self.kuka.makeTrapezeSimpleCiclic(
            int(self.frictionJNumTex.GetValue()),
            float(self.frictionAngleEndTex.GetValue()),
            float(self.frictionMaxWTex.GetValue())
        ))

    def OnFullFriction(self, event):
        """
            эксперимент по трению со всеми звеньями
        """
        self.kuka.fullFriction()

    def OnFindZeroMoment(self, event):
        """
            поиск нулевого момента
        """
        self.kuka.zeroMomentInJoint(int(self.zeroMomentTex.GetValue()))
        pass

    def OnWarmUp(self, event):
        """
            разогрев звена
        """
        self.kuka.warmUpLink(int(self.warmUpNumTex.GetValue()), float(self.warmUpTimeTex.GetValue()))
        pass

    def OnGravityFind(self, event):
        """
            эксперимент с гравитацией
        """
        self.kuka.gravitationFindR()

    def OnWarmUPRobot(self,event):
        self.kuka.warmUpRobot()

    def OnForceControl(self, event):
        """
            режим Force-Control
        """
        self.kuka.forceControl()

    def OnRandomConf(self,event):
        """
            Перевести робота в случайную конфигурацию
        """
        self.kuka.moveToRandomConf(0.1)
