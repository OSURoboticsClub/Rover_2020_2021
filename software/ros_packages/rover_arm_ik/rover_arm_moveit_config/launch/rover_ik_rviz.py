#!/usr/bin/env python

import roslib
import rospkg
import sys
import rospy

##"python_qt_binding" package which hides differences between PyQt and PySide
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from PyQt5.QtWidgets import QApplication, QWidget, \
    QVBoxLayout, QSlider, QHBoxLayout, QPushButton
import rviz

class MoveItUI(QWidget):
    def __init__(self):
        QWidget.__init__(self)

        self.frame = rviz.VisualizationFrame()


        self.frame.setSplashPath( "" )


        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "moveit.rviz" )
        self.frame.load( config )

        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )

        self.manager = self.frame.getManager()

        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )

        layout = QVBoxLayout()
        layout.addWidget( self.frame )

        h_layout = QHBoxLayout()
        layout.addLayout( h_layout )
        self.setLayout( layout )

if __name__ == '__main__':
    app = QApplication( sys.argv )

    myviz = MoveItUI(  )
    myviz.resize( 960, 960 )
    myviz.show(  )

    app.exec_(  )
