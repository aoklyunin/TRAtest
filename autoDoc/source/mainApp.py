#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Главное приложение, здесь ничего менять не надо

"""

import wx
import rospy
from scripts.forms import KukaFrame


# класс приложения
class App(wx.App):
    """
        класс приложения
    """
    def OnInit(self):
        """
        инициализация приложения
        :return: True/False получилось ли создать приложение
        """
        # Задаём имя ноды
        self.node_name = "fucking_kuka_node"
        # инициализируем ноду роса
        rospy.init_node(self.node_name)
        # создание окна
        self.frame = KukaFrame(None,-1,"Kuka Controller")
        # отображение окна
        self.frame.Show(True)
        # указываем, что только что созданное окно - главное
        self.SetTopWindow(self.frame)
        # функция, вызываемая при закрытии ноды
        rospy.on_shutdown(self.shutdown)
        return True # ну не False же возвращать, правда?:)

    def shutdown(self):
        """
            Завершение приложения
        :return:
        """
        # закрываем приложение
        self.frame.Close()
        # Завершаем работу робота
        rospy.loginfo("Stopping the robot...")


# главная функция
if __name__=='__main__':
    # объект класса приложение
    app = App()
    # запускаем главный цикл приложения
    app.MainLoop()


