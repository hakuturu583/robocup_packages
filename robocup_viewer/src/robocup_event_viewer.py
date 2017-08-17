#!/usr/bin/env python
from robocup_msgs.msg import Event
from python_qt_binding import QtGui

class event_icon(QtGui.QWidget):
    def __init__(self):
        super(event_icon, self).__init__(parent=None)
