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

        # поиск гравитации для всех звеньев
        self.gravityFindBtn = wx.Button(self.expPanel, label="Gravity Exp (all)", pos=(310, 120), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnGravityFind, self.gravityFindBtn)

        # поиск гравитации для трех звеньев
        self.gravityFindBtn3 = wx.Button(self.expPanel, label="Gravity Exp 3", pos=(470, 120), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnGravityFind3, self.gravityFindBtn3)

        # Установка в случайную позицию
        self.randomConfBtn = wx.Button(self.expPanel, label="Random Conf", pos=(310, 170), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnRandomConf, self.randomConfBtn)

        # Режим Force-контроля
        self.ForceControlBtn = wx.Button(self.expPanel, label="ForceControl", pos=(470, 220), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnForceControl, self.ForceControlBtn)

        # Режим Force-контроля
        self.WarmUpBtn = wx.Button(self.expPanel, label="Warm up", pos=(470, 170), size=(140, 30))
        self.Bind(wx.EVT_BUTTON, self.OnWarmUPRobot, self.WarmUpBtn)

        '''
        Исследование трения
        '''
        # Повторяемость в случайных позициях
        self.frictionRandFindBtn = wx.Button(self.expPanel, label="Friction (rand)", pos=(30, 220), size=(130, 30))
        self.Bind(wx.EVT_BUTTON, self.OnFrictionRandFind, self.frictionRandFindBtn)

        # Повторяемость в крайних положениях
        self.frictionMaxFindBtn = wx.Button(self.expPanel, label="Friction (max)", pos=(30, 270), size=(130, 30))
        self.Bind(wx.EVT_BUTTON, self.OnFrictionMaxFind, self.frictionMaxFindBtn)

        # Сжать пальцы
        self.fingersSqueezeBtn = wx.Button(self.expPanel, label="Squeeze fingers", pos=(170, 220), size=(130, 30))
        self.Bind(wx.EVT_BUTTON, self.OnFingersSqueeze, self.fingersSqueezeBtn)

        # Разжать пальцы
        self.fingersReleaseBtn = wx.Button(self.expPanel, label="Realease fingers", pos=(170, 270), size=(130, 30))
        self.Bind(wx.EVT_BUTTON, self.OnFingersRelease, self.fingersReleaseBtn)

        '''
        Оценка массы
        '''
        wx.StaticText(self.panel, -1, "Estimated mass:", (220, 270))

        '''
        Демо
        '''
        self.linearMovementBtn = wx.Button(self.expPanel, label="Linear", pos=(320, 270), size=(130, 30))
        self.Bind(wx.EVT_BUTTON, self.OnStartLinearMove, self.linearMovementBtn)

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
            эксперимент с гравитацией для всех звеньев
        """
        self.kuka.gravitationFindR()

    def OnGravityFind3(self, event):
        """
            эксперимент с гравитацией для трех звеньев
        """
        self.kuka.gravitationFindR3()

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

    def OnFrictionRandFind(self,event):
        self.kuka.frictionRandomVarianceExp()

    def OnFrictionMaxFind(self,event):
        self.kuka.frictionMaxVarianceExp()

    def OnFingersSqueeze(self,event):
        self.kuka.setGripperPositions(10,10)
        print "Button is push"

    def OnFingersRelease(self,event):
        self.kuka.releaseFingers()

    def OnStartLinearMove(self, event):
        pauseTime = 0.1
        maxOverG = 100
        self.kuka.linearMove( pauseTime, maxOverG)