#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys

from PyQt5 import QtWidgets
from PyQt5 import QtGui
from PyQt5 import QtCore


def UI_Thread():
    global main_Window, map

    window_width =  1320 #1295
    window_height = 1320

    app = QtWidgets.QApplication(sys.argv) # 시작
    main_Window = QtWidgets.QLabel()
    main_Window.setWindowTitle("A1_Gazebo")
    main_Window.resize(window_width,window_height)
    main_Window.setStyleSheet("color: white;""background-color: #000000")
    main_Window.show()

    map = QtWidgets.QLabel(main_Window)
    map.setFont(QtGui.QFont('Arial Black',9))
    map.setGeometry(5, 5, 1310, 1310)
    map.show()

    sys.exit(app.exec_())