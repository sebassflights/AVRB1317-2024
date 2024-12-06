import base64
import json
import math
from enum import Enum, auto
from typing import List, Optional, Tuple
from time import sleep

import colour
import numpy as np
import scipy.interpolate
from bell.avr.mqtt.payloads import (
    AvrPcmFireLaserPayload,
    AvrPcmSetLaserOffPayload,
    AvrPcmSetLaserOnPayload,
    AvrPcmSetServoAbsPayload,
    AvrPcmSetServoPctPayload,
)
from bell.avr.mqtt.payloads import (
    AvrPcmSetBaseColorPayload,
    AvrPcmSetServoOpenClosePayload,
)
import pygame
pygame.init()
from bell.avr.utils.timing import rate_limit
from PySide6 import QtCore, QtGui, QtWidgets

from ..lib.calc import constrain, map_value
from ..lib.color import wrap_text
from ..lib.config import config
from ..lib.widgets import DoubleLineEdit
from .base import BaseTabWidget


class Direction(Enum):
    Left = auto()
    Right = auto()
    Up = auto()
    Down = auto()


class ThermalView(QtWidgets.QWidget):
    def __init__(self, parent: QtWidgets.QWidget) -> None:
        super().__init__(parent)

        # canvas size
        self.width_ = 300
        self.height_ = self.width_

        # pixels within canvas
        self.pixels_x = 30
        self.pixels_y = self.pixels_x
        self.pixel_width = self.width_ / self.pixels_x
        self.pixel_height = self.height_ / self.pixels_y

        # low range of the sensor (this will be blue on the screen)
        self.MINTEMP = 20.0

        # high range of the sensor (this will be red on the screen)
        self.MAXTEMP = 32.0

        # last lowest temp from camera
        self.last_lowest_temp = 999.0

        # how many color values we can have
        self.COLORDEPTH = 1024

        # how many pixels the camera is
        self.camera_x = 8
        self.camera_y = self.camera_x
        self.camera_total = self.camera_x * self.camera_y

        # create list of x/y points
        self.points = [
            (math.floor(ix / self.camera_x), (ix % self.camera_y))
            for ix in range(self.camera_total)
        ]
        # i'm not fully sure what this does
        self.grid_x, self.grid_y = np.mgrid[
            0 : self.camera_x - 1 : self.camera_total / 2j,
            0 : self.camera_y - 1 : self.camera_total / 2j,
        ]

        # create avaiable colors
        self.colors = [
            (int(c.red * 255), int(c.green * 255), int(c.blue * 255))
            for c in list(
                colour.Color("indigo").range_to(colour.Color("red"), self.COLORDEPTH)
            )
        ]

        # create canvas
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        self.canvas = QtWidgets.QGraphicsScene()
        self.view = QtWidgets.QGraphicsView(self.canvas)
        self.view.setGeometry(0, 0, self.width_, self.height_)

        layout.addWidget(self.view)

        # need a bit of padding for the edges of the canvas
        self.setFixedSize(self.width_ + 50, self.height_ + 50)

    def set_temp_range(self, mintemp: float, maxtemp: float) -> None:
        self.MINTEMP = mintemp
        self.MAXTEMP = maxtemp

    def set_calibrted_temp_range(self) -> None:
        self.MINTEMP = self.last_lowest_temp + 0.0
        self.MAXTEMP = self.last_lowest_temp + 15.0

    def update_canvas(self, pixels: List[int]) -> None:
        float_pixels = [
            map_value(p, self.MINTEMP, self.MAXTEMP, 0, self.COLORDEPTH - 1)
            for p in pixels
        ]

        # Rotate 90° to orient for mounting correctly
        float_pixels_matrix = np.reshape(float_pixels, (self.camera_x, self.camera_y))
        float_pixels_matrix = np.rot90(float_pixels_matrix, 1)
        rotated_float_pixels = float_pixels_matrix.flatten()

        bicubic = scipy.interpolate.griddata(
            self.points,
            rotated_float_pixels,
            (self.grid_x, self.grid_y),
            method="cubic",
        )

        pen = QtGui.QPen(QtCore.Qt.NoPen)
        self.canvas.clear()

        for ix, row in enumerate(bicubic):
            for jx, pixel in enumerate(row):
                brush = QtGui.QBrush(
                    QtGui.QColor(
                        *self.colors[int(constrain(pixel, 0, self.COLORDEPTH - 1))]
                    )
                )
                self.canvas.addRect(
                    self.pixel_width * jx,
                    self.pixel_height * ix,
                    self.pixel_width,
                    self.pixel_height,
                    pen,
                    brush,
                )


class JoystickWidget(BaseTabWidget):
    def __init__(self, parent: QtWidgets.QWidget) -> None:
        super().__init__(parent)

        self.setFixedSize(300, 300)

        self.moving_offset = QtCore.QPointF(0, 0)

        self.grab_center = False
        self.__max_distance = 100

        self.current_y = 0
        self.current_x = 0

        self.servoxmin = 10
        self.servoymin = 10
        self.servoxmax = 99
        self.servoymax = 99

        # servo declarations
        self.SERVO_ABS_MAX = 2200
        self.SERVO_ABS_MIN = 700
        # joystick declaration
        joystick = pygame.joystick.Joystick(0)
        joystick.init()



    def move_gimbal(self, x_servo_percent: int, y_servo_percent: int) -> None:
        self.send_message(
            "avr/pcm/set_servo_pct",
            AvrPcmSetServoPctPayload(servo=2, percent=x_servo_percent),
        )
        self.send_message(
            "avr/pcm/set_servo_pct",
            AvrPcmSetServoPctPayload(servo=3, percent=y_servo_percent),
        )

    def move_gimbal_absolute(self, x_servo_abs: int, y_servo_abs: int) -> None:
        self.send_message(
            "avr/pcm/set_servo_abs",
            AvrPcmSetServoAbsPayload(servo=2, absolute=x_servo_abs),
        )
        self.send_message(
            "avr/pcm/set_servo_abs",
            AvrPcmSetServoAbsPayload(servo=3, absolute=y_servo_abs),
        )

    # def servo_update(self, event: pygame.event.Event) -> None:
    #     for event in pygame.event.get(): # User did something
    #     # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
    #         if event.type == pygame.JOYBUTTONDOWN:
    #             print("Joystick button pressed.")
    #         if event.type == pygame.JOYBUTTONUP:
    #             print("Joystick button released.")

    #     joystick = pygame.joystick.Joystick(0)
    #     joystick.init()
    #     axis_x = round(joystick.get_axis(0), 2)
    #     axis_y = round(joystick.get_axis(1), 2)

    #     rate_limit(self.move_gimbal_absolute(axis_x, axis_y), 50)

        # self.send_message(
        #     "avr/pcm/set_servo_pct",
        #     AvrPcmSetServoPctPayload(servo=2, percent=axis_x),
        # )
        # self.send_message(
        #     "avr/pcm/set_servo_pct",
        #     AvrPcmSetServoPctPayload(servo=3, percent=axis_y),
        # )

    # rate_limit(servo_update, 50)

    # for event in pygame.event.get(self): # User did something







    # def mouseMoveEvent(self) -> None:
    #   rate_limit(self.update_servos, frequency=50)


class ThermalViewControlWidget(BaseTabWidget):
    def __init__(self, parent: QtWidgets.QWidget) -> None:
        super().__init__(parent)

        self.setWindowTitle("New Slide")

    def build(self) -> None:
        """
        Build the GUI layout
        """
        self.axis_x = 700
        self.axis_y = 700
        layout = QtWidgets.QHBoxLayout(self)
        layout_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setLayout(layout)

        # viewer
        viewer_groupbox = QtWidgets.QGroupBox("Viewer")
        viewer_layout = QtWidgets.QVBoxLayout()
        viewer_groupbox.setLayout(viewer_layout)

        self.viewer = ThermalView(self)
        viewer_layout.addWidget(self.viewer)

        # set temp range

        # lay out the host label and line edit
        temp_range_layout = QtWidgets.QFormLayout()

        self.temp_min_line_edit = DoubleLineEdit()
        temp_range_layout.addRow(QtWidgets.QLabel("Min Temp:"), self.temp_min_line_edit)
        self.temp_min_line_edit.setText(str(self.viewer.MINTEMP))

        self.temp_max_line_edit = DoubleLineEdit()
        temp_range_layout.addRow(QtWidgets.QLabel("Max Temp:"), self.temp_max_line_edit)
        self.temp_max_line_edit.setText(str(self.viewer.MAXTEMP))

        set_temp_range_button = QtWidgets.QPushButton("Set Temp Range")
        temp_range_layout.addWidget(set_temp_range_button)

        set_temp_range_calibrate_button = QtWidgets.QPushButton(
            "Auto Calibrate Temp Range"
        )
        temp_range_layout.addWidget(set_temp_range_calibrate_button)

        viewer_layout.addLayout(temp_range_layout)

        set_temp_range_button.clicked.connect(  # type: ignore
            lambda: self.viewer.set_temp_range(
                float(self.temp_min_line_edit.text()),
                float(self.temp_max_line_edit.text()),
            )
        )

        set_temp_range_calibrate_button.clicked.connect(  # type: ignore
            lambda: self.calibrate_temp()
        )

        layout_splitter.addWidget(viewer_groupbox)

        # joystick
        joystick_groupbox = QtWidgets.QGroupBox("Joystick")
        joystick_layout = QtWidgets.QVBoxLayout()
        joystick_groupbox.setLayout(joystick_layout)

        sub_joystick_layout = QtWidgets.QHBoxLayout()
        joystick_layout.addLayout(sub_joystick_layout)

        self.joystick = JoystickWidget(self)
        sub_joystick_layout.addWidget(self.joystick)

        fire_laser_button = QtWidgets.QPushButton("Fire Laser")
        joystick_layout.addWidget(fire_laser_button)

        laser_toggle_layout = QtWidgets.QHBoxLayout()

        laser_on_button = QtWidgets.QPushButton("Laser On")
        laser_toggle_layout.addWidget(laser_on_button)

        laser_off_button = QtWidgets.QPushButton("Laser Off")
        laser_toggle_layout.addWidget(laser_off_button)

        self.laser_toggle_label = QtWidgets.QLabel()
        self.laser_toggle_label.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter
        )
        laser_toggle_layout.addWidget(self.laser_toggle_label)

        joystick_layout.addLayout(laser_toggle_layout)

        # https://i.imgur.com/yvgNiFE.jpg
        self.joystick_inverted_checkbox = QtWidgets.QCheckBox("Invert Joystick")
        joystick_layout.addWidget(self.joystick_inverted_checkbox)
        self.joystick_inverted_checkbox.setChecked(config.joystick_inverted)

        layout_splitter.addWidget(joystick_groupbox)
        layout.addWidget(layout_splitter)

        # connect signals
        self.joystick.emit_message.connect(self.emit_message.emit)

        fire_laser_button.clicked.connect(  # type: ignore
            lambda: self.send_message("avr/pcm/fire_laser", AvrPcmFireLaserPayload())
        )

        laser_on_button.clicked.connect(lambda: self.set_laser(True))  # type: ignore
        laser_off_button.clicked.connect(lambda: self.set_laser(False))  # type: ignore

        self.joystick_inverted_checkbox.clicked.connect(self.inverted_checkbox_clicked)  # type: ignore

        # don't allow us to shrink below size hint
        self.setMinimumSize(self.sizeHint())

    def inverted_checkbox_clicked(self) -> None:
        """
        Callback when joystick inverted checkbox is clicked
        """
        config.joystick_inverted = self.joystick_inverted_checkbox.isChecked()



    def set_laser(self, state: bool) -> None:
        if state:
            self.laserOn = True
            topic = "avr/pcm/set_laser_on"
            payload = AvrPcmSetLaserOnPayload()
            text = "Laser On"
            color = "green"
            self.axis_x = 1450
            self.axis_y = 1450
            self.press_count = False
        else:
            self.laserOn = False
            topic = "avr/pcm/set_laser_off"
            payload = AvrPcmSetLaserOffPayload()
            text = "Laser Off"
            color = "red"
        self.send_message(topic, payload)
        self.laser_toggle_label.setText(wrap_text(text, color))

        if self.laserOn == True:
            while True:
                for event in pygame.event.get(): # User did something
                # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
                    if event.type == pygame.JOYBUTTONDOWN:
                        print("1") # might be able to replace with pass, but not sure (unable to test)
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                laser_zoom = joystick.get_axis(4)
                if laser_zoom >= 0.5:
                    zoom = 7.5
                else:
                    zoom = 16

                self.joy_x = round(joystick.get_axis(2), 2) ** 3 * zoom
                self.joy_y = round(joystick.get_axis(3), 2) ** 3 * zoom * 0.75

                if abs(self.joy_x) > 0.75:
                    self.axis_x = self.joy_x + self.axis_x
                if abs(self.joy_y) > 0.75:
                    self.axis_y = self.joy_y + self.axis_y
                button = joystick.get_button(0)
                button2 = joystick.get_button(1)
                button3 = joystick.get_hat(0)


                laser_left = joystick.get_button(4)
                laser_right = joystick.get_button(5)
                laser_right_axis = joystick.get_axis(5)
                if self.axis_y > 2200:
                    self.axis_y = 2200
                elif self.axis_y < 700:
                    self.axis_y = 700
                if self.axis_x > 2200:
                    self.axis_x = 2200
                elif self.axis_x < 700:
                    self.axis_x = 700
                if laser_right_axis >= 0.5 or laser_right == True:
                    topic = "avr/pcm/set_laser_on"
                    payload = AvrPcmSetLaserOffPayload()
                    self.send_message(topic, payload)
                if laser_left == True:
                    topic = "avr/pcm/set_laser_off"
                    payload = AvrPcmSetLaserOffPayload()
                    self.send_message(topic, payload)
                if button3 == (0,1):
                    self.axis_x = 1450
                    self.axis_y = 1050
                if button3 == (1,0):
                    self.axis_x = 2200
                    self.axis_y = 1150
                if button3 == (0,-1):
                    self.axis_x = 1450
                    self.axis_y = 2200
                if button3 == (-1,0):
                    self.axis_x = 700
                    self.axis_y = 1150
                self.send_message(
                    "avr/pcm/set_servo_abs",
                    AvrPcmSetServoAbsPayload(servo=2, absolute = int(self.axis_x)),
                )
                self.send_message(
                    "avr/pcm/set_servo_abs",
                    AvrPcmSetServoAbsPayload(servo=3, absolute = int(self.axis_y)),
                    )
                if button == True:
                    self.send_message("avr/pcm/set_servo_open_close", AvrPcmSetServoOpenClosePayload(servo=0, action="open"),)
                if button2 == True:
                    self.send_message("avr/pcm/set_servo_open_close", AvrPcmSetServoOpenClosePayload(servo=0, action="close"),)
                sleep(0.035)




    def calibrate_temp(self) -> None:
        self.viewer.set_calibrted_temp_range()
        self.temp_min_line_edit.setText(str(self.viewer.MINTEMP))
        self.temp_max_line_edit.setText(str(self.viewer.MAXTEMP))


    def process_message(self, topic: str, payload: str) -> None:
        """
        Process an incoming message and update the appropriate component
        """
        # discard topics we don't recognize
        if topic != "avr/thermal/reading":
            return

        data = json.loads(payload)["data"]

        # decode the payload
        base64Decoded = data.encode("utf-8")
        asbytes = base64.b64decode(base64Decoded)
        pixel_ints = list(bytearray(asbytes))

        # find lowest temp
        lowest = min(pixel_ints)
        self.viewer.last_lowest_temp = lowest

        # update the canvase
        # pixel_ints = data
        self.viewer.update_canvas(pixel_ints)

    def clear(self) -> None:
        self.viewer.canvas.clear()
