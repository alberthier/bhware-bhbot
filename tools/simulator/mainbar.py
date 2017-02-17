# encoding: utf-8

import os

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic

from definitions import *


(MainBar_Ui, MainBar_Widget) =  uic.loadUiType(os.path.join(os.path.dirname(__file__), "mainbar.ui"))




class MainBar(QWidget, MainBar_Ui):

    def __init__(self, parent = None):
        QWidget.__init__(self, parent)
        MainBar_Ui.__init__(self)
        self.setupUi(self)
        self.set_icon(self.reload, "refresh")
        self.set_icon(self.start_pause, "start")
        self.set_icon(self.stop, "stop")
        self.set_color_icon(self.main_right_robot, TEAM_RIGHT_COLOR)
        self.set_color_icon(self.secondary_right_robot, TEAM_RIGHT_COLOR)
        self.set_color_icon(self.main_left_robot, TEAM_LEFT_COLOR)
        self.set_color_icon(self.secondary_left_robot, TEAM_LEFT_COLOR)

        self.main_right_robot.setChecked(True)
        self.main_left_robot.setChecked(True)
        self.oldest_pressed = self.main_right_robot

        self.main_right_robot.toggled.connect(self.main_right_toggled)
        self.secondary_right_robot.toggled.connect(self.secondary_right_toggled)
        self.main_left_robot.toggled.connect(self.main_left_toggled)
        self.secondary_left_robot.toggled.connect(self.secondary_left_toggled)


    def set_icon(self, button, icon_name):
        icons_dir = os.path.join(os.path.dirname(__file__), "icons")
        button.setIcon(QIcon(os.path.join(icons_dir, "{}.svg".format(icon_name))))


    def set_color_icon(self, button, color):
        pixmap = QPixmap(QSize(16, 16))
        pixmap.fill(QColor(color))
        button.setIcon(QIcon(pixmap))


    def main_right_toggled(self):
        self.button_toggled(self.main_right_robot)


    def secondary_right_toggled(self):
        self.button_toggled(self.secondary_right_robot)


    def main_left_toggled(self):
        self.button_toggled(self.main_left_robot)


    def secondary_left_toggled(self):
        self.button_toggled(self.secondary_left_robot)


    def button_toggled(self, toggled):
        buttons = [ self.main_right_robot, self.secondary_right_robot, self.main_left_robot, self.secondary_left_robot ]
        cpt = 0
        for button in buttons:
            if button.isChecked():
                cpt += 1
        if cpt > 2:
            self.oldest_pressed.setChecked(False)
        for button in buttons:
            if button != toggled and button.isChecked():
                self.oldest_pressed = button
                break
        if self.oldest_pressed is None:
            self.oldest_pressed = toggled


    def get_expected_robots(self):
        expected = []
        buttons = [ self.main_left_robot, self.secondary_left_robot, self.main_right_robot, self.secondary_right_robot ]
        teams = [ TEAM_LEFT, TEAM_LEFT, TEAM_RIGHT, TEAM_RIGHT ]
        robot_type = [ True, False, True, False ]

        for button, team, is_main in zip(buttons, teams, robot_type):
            if button.isChecked():
                expected.append((team, is_main))

        return expected
