#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    Здесь создаётся графическое окно
"""
from scripts.kukaController import KukaController
from scripts.kukaWrapper.forms import Frame
import wx


class KukaFrame(Frame):
    """
        Инициализация элементов панели self.expPanel
        Ширина 690, Высота 500

    """

    def __init__(self, parent=None, id=-1, title=''):
        Frame.__init__(self, parent, id, title)
        # создаём объект для взаимодействия с роботом
        self.kuka = KukaController()

    def initExpItems(self):
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

        # Режим Force-контроля
        self.ForceControlBtn = wx.Button(self.expPanel, label="ForceControl", pos=(470, 120), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnForceControl, self.ForceControlBtn)

    # таймер, срабатывает два раза в секунду
    def ExpTimer(self):
        # всё, что сюда напишешь будет выполняться каждые 500 мсек
        pass

    # тестовая функция(можешь написать, что угодно, оно будет срабатывать каждый раз, когда ты жмёшь кнопку "Тест")
    def OnTest(self, event):
        print ("Test Button clicked")

    # Эксперимент по трению с одним звеном
    def OnFriction(self, event):
        print (self.kuka.makeTrapezeSimpleCiclic(
            int(self.frictionJNumTex.GetValue()),
            float(self.frictionAngleEndTex.GetValue()),
            float(self.frictionMaxWTex.GetValue())
        ))

    # эксперимент по трению со всеми звеньями
    def OnFullFriction(self, event):
        self.kuka.fullFriction()

    # поиск нулевого момента
    def OnFindZeroMoment(self, event):
        self.kuka.zeroMomentInJoint(int(self.zeroMomentTex.GetValue()))
        pass

    # разогрев звена
    def OnWarmUp(self, event):
        self.kuka.warmUpLink(int(self.warmUpNumTex.GetValue()), float(self.warmUpTimeTex.GetValue()))
        pass

    # эксперимент с гравитацией
    def OnGravityFind(self, event):
        self.kuka.gravitationFind()

    # режим Force-Control
    def OnForceControl(self, event):
        self.kuka.forceControl()
