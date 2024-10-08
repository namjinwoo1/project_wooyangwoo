#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from PySide2.QtWidgets import QDialog, QLabel, QInputDialog
from PySide2.QtCore import Qt
import rospy

class LargeMapWindow(QDialog):
    def __init__(self, parent=None):
        super(LargeMapWindow, self).__init__(parent)
        self.setWindowTitle("클릭해서 좌표 저장")
        self.setGeometry(100, 100, 800, 900)

        # 지도 표시용 QLabel
        self.map_label = QLabel(self)
        self.map_label.setGeometry(10, 10, 768, 768)
        self.map_label.setAlignment(Qt.AlignCenter)

        # 부모 창에서 이미지를 받아와 표시
        pixmap = parent.map_label.pixmap()
        if pixmap:
            self.scaled_pixmap = pixmap.scaled(self.map_label.size(), Qt.KeepAspectRatio)
            self.map_label.setPixmap(self.scaled_pixmap)

        # 부모의 해상도, 원점 정보 가져옴
        self.map_resolution = parent.map_resolution / 4 
        self.map_origin_x = -0.6
        self.map_origin_y = -0.1
        self.scale_factor = parent.scale_factor * 2
        self.parent = parent

    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.map_resolution is not None:
            # 클릭한 위치의 좌표 가져오기
            click_position = event.pos()

            # QLabel 크기 가져오기 (확장된 맵 크기)
            label_width = self.map_label.width()
            label_height = self.map_label.height()

            # 중앙에서 클릭한 위치까지의 거리(픽셀 단위)를 해상도에 맞춰 변환하여 실제 좌표로 변환
            map_x = (click_position.x() - (label_width / 2)) * self.map_resolution + self.map_origin_x
            map_y = ((label_height / 2) - click_position.y()) * self.map_resolution + self.map_origin_y

            # 좌표를 저장하는 부분
            name, ok = QInputDialog.getText(self, "좌표 저장", "좌표 이름을 입력하세요:")
            if ok and name:
                self.parent.saved_coordinates[name] = (map_x, map_y)
                rospy.loginfo(f"좌표 저장됨: {name}, X: {map_x}, Y: {map_y}")
                self.parent.save_coordinates_to_file()





