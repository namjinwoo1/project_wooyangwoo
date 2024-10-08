#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import subprocess
import numpy as np
import tf2_ros
from PySide2.QtGui import QImage, QPixmap, QPainter, QColor, QPen, QBrush, QPolygon
from PySide2.QtWidgets import QMainWindow, QLabel, QScrollArea, QPushButton, QListWidget, QMenuBar, QAction
from PySide2.QtCore import Qt, QSize, QPoint, QTimer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from math import cos, sin, radians
import json
import os
from explore_lite_gui import Ui_MainWindow
from large_map_window import LargeMapWindow

class ExploreLiteController(QMainWindow):
    def __init__(self):
        super(ExploreLiteController, self).__init__()

        # ROS 노드 초기화
        rospy.init_node('explore_lite_gui_controller', anonymous=True)

        # tf2_ros 버퍼 및 리스너 초기화 (로봇의 위치를 추적하기 위해)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # UI 초기화
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # 지도 시각화를 위한 스크롤 영역 추가
        self.scroll_area = QScrollArea(self.ui.centralwidget)
        self.scroll_area.setGeometry(10, 10, 500, 500)
        self.scroll_area.setWidgetResizable(True)

        # 지도 라벨 추가
        self.map_label = QLabel(self.scroll_area)
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setText("매핑중입니다")
        self.scroll_area.setWidget(self.map_label)

        # 로봇 좌표를 표시할 라벨 추가
        self.robot_position_label = QLabel(self)
        self.robot_position_label.setGeometry(100, 550, 200, 40)
        self.robot_position_label.setText("로봇 위치: ")

        # 로봇 위치와 방향을 표시할 변수
        self.robot_position_x = None
        self.robot_position_y = None
        self.robot_yaw = None

        # 타이머 추가 (500ms마다 로봇 위치 업데이트)
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_robot_position)
        self.update_timer.start(500)

        # 좌표 저장을 위한 딕셔너리 {이름: (x, y)}
        self.saved_coordinates = {}
        self.load_saved_coordinates()

        # ROS 이동 목표 퍼블리셔
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # 좌표 목록을 보기 위한 버튼 추가
        self.show_saved_coordinates_button = QPushButton("저장된 좌표 보기", self)
        self.show_saved_coordinates_button.setGeometry(600, 40, 180, 40)
        self.show_saved_coordinates_button.clicked.connect(self.toggle_saved_coordinates)

        # 저장된 좌표 리스트 추가 (초기에는 숨김)
        self.list_window = QListWidget(self)
        self.list_window.setGeometry(600, 90, 180, 300)
        self.list_window.hide()

        # 좌표 삭제 버튼 추가
        self.delete_coordinate_button = QPushButton("좌표 삭제", self)
        self.delete_coordinate_button.setGeometry(700, 420, 80, 30)
        self.delete_coordinate_button.clicked.connect(self.delete_selected_coordinate)
        self.delete_coordinate_button.hide()

        # 좌표 이동 버튼 추가
        self.move_coordinate_button = QPushButton("좌표 이동", self)
        self.move_coordinate_button.setGeometry(600, 420, 80, 30)
        self.move_coordinate_button.clicked.connect(self.move_to_selected_coordinate)
        self.move_coordinate_button.hide()

        # 메뉴바 추가 (좌표 저장 기능을 위한)
        self.menu_bar = QMenuBar(self)
        self.setMenuBar(self.menu_bar)

        self.save_menu = self.menu_bar.addMenu("좌표저장하기")
        self.save_action = QAction("맵 크게 띄우기", self)
        self.save_menu.addAction(self.save_action)
        self.save_action.triggered.connect(self.open_large_map)

        # 지도 해상도 및 원점 값 저장 (OccupancyGrid 메시지에서 갱신)
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None

        # 버튼 클릭 시 호출될 함수 연결
        self.ui.startButton.clicked.connect(self.start_explore_lite)
        self.ui.stopButton.clicked.connect(self.stop_explore_lite)

        # 맵 확대 비율
        self.scale_factor = 2

        # Explore Lite 실행 프로세스
        self.explore_process = None

        # 지도 데이터 구독
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

    def update_robot_position(self):
        try:
            # tf2_ros 버퍼에서 변환된 좌표와 회전 정보 가져오기
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))

            self.robot_position_x = trans.transform.translation.x
            self.robot_position_y = trans.transform.translation.y
            self.robot_yaw = self.get_yaw_from_quaternion(trans.transform.rotation)

            # 좌표 라벨 업데이트
            self.robot_position_label.setText(f"로봇 위치: X: {self.robot_position_x:.2f}, Y: {self.robot_position_y:.2f}")

            # 맵 위에 로봇 위치 그리기
            self.draw_robot_on_map()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def get_yaw_from_quaternion(self, rotation):
        """쿼터니언을 이용해 Yaw(회전 각도)를 계산하는 함수"""
        q_x = rotation.x
        q_y = rotation.y
        q_z = rotation.z
        q_w = rotation.w

        yaw = np.arctan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z))
        return np.degrees(yaw)

    def map_callback(self, data):
        # OccupancyGrid 데이터를 이미지로 변환
        map_data = np.array(data.data).reshape((data.info.height, data.info.width))

        # 맵의 해상도와 원점 정보 가져오기
        self.map_resolution = data.info.resolution  # 각 픽셀당 미터
        self.map_origin_x = -0.2  # 보정 필요시 조정
        self.map_origin_y = -0.35  # 보정 필요시 조정

        # 맵 확대 비율 설정 (scale_factor 조정)
        new_height = data.info.height * self.scale_factor
        new_width = data.info.width * self.scale_factor

        # 원본 데이터를 확대
        map_data_expanded = np.kron(map_data, np.ones((self.scale_factor, self.scale_factor)))

        # 100이 장애물, 0이 비어 있는 공간, -1은 미확인 지역
        map_img = np.zeros((new_height, new_width, 3), dtype=np.uint8)
        map_img[map_data_expanded == 100] = [0, 0, 0]     # 검정색 (장애물)
        map_img[map_data_expanded == 0] = [255, 255, 255]  # 흰색 (비어 있는 공간)
        map_img[map_data_expanded == -1] = [128, 128, 128]  # 회색 (미확인)

        # Numpy 이미지를 QImage로 변환
        q_image = QImage(map_img.data, new_width, new_height, QImage.Format_RGB888)

          # 상하 좌우 반전 처리
        q_image = q_image.mirrored(False, True)  # 첫 번째 인자는 좌우 반전, 두 번째 인자는 상하 반전
        pixmap = QPixmap.fromImage(q_image)

        # 스크롤 영역 크기에 맞게 지도를 확장
        scroll_width = self.scroll_area.width()
        scroll_height = self.scroll_area.height()

        # 확장 비율 계산
        x_expand_ratio = scroll_width / new_width
        y_expand_ratio = scroll_height / new_height

        # 스크롤 영역에 꽉 차게 확장 (유지)
        scaled_pixmap = pixmap.scaled(scroll_width, scroll_height, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        # QLabel에 이미지를 표시하고, 스크롤 영역을 꽉 채움
        self.map_label.setPixmap(scaled_pixmap)
        self.map_label.resize(scaled_pixmap.width(), scaled_pixmap.height())

        # 디버깅 로그 추가
        rospy.loginfo(f"맵 원점 (보정 후): X: {self.map_origin_x}, Y: {self.map_origin_y}")

        # 저장용 확장 비율을 따로 저장
        self.x_expand_ratio = x_expand_ratio
        self.y_expand_ratio = y_expand_ratio

    def draw_robot_on_map(self):
        """맵 위에 로봇의 위치와 방향을 그리는 함수"""
        if self.robot_position_x is None or self.robot_position_y is None or self.robot_yaw is None:
            return

        pixmap = self.map_label.pixmap()

        if pixmap:
            # 새로운 QPixmap을 생성해서 로봇 위치와 방향을 그리기
            robot_pixmap = pixmap.copy()
            painter = QPainter(robot_pixmap)

            if not painter.isActive():
                painter.begin(robot_pixmap)  # QPainter 시작

            pen = QPen(QColor(255, 0, 0), 3)
            painter.setPen(pen)
            brush = QBrush(QColor(255, 0, 0))
            painter.setBrush(brush)

            # 로봇의 좌표를 맵 해상도와 원점을 고려하여 변환, 확장 비율도 반영
            gui_x = int((self.robot_position_x - self.map_origin_x) / self.map_resolution * self.scale_factor * self.x_expand_ratio)
            gui_y = int((self.robot_position_y - self.map_origin_y) / self.map_resolution * self.scale_factor * self.y_expand_ratio)

            # GUI 중심 좌표 보정
            center_x = self.map_label.width() // 2
            center_y = self.map_label.height() // 2

            # 로봇의 위치를 GUI 중앙 기준으로 변환
            gui_x = center_x + gui_x
            gui_y = center_y - gui_y  # Y 좌표는 위로 갈수록 감소하므로 반전

            # 로봇의 방향에 따라 삼각형 모양으로 그리기
            triangle_size = 10  # 삼각형 크기
            angle = radians(self.robot_yaw)

            # 삼각형의 세 꼭짓점 계산
            point1 = QPoint(gui_x + triangle_size * cos(angle), gui_y - triangle_size * sin(angle))
            point2 = QPoint(gui_x - triangle_size * cos(angle + radians(120)), gui_y + triangle_size * sin(angle + radians(120)))
            point3 = QPoint(gui_x - triangle_size * cos(angle - radians(120)), gui_y + triangle_size * sin(angle - radians(120)))

            triangle = QPolygon([point1, point2, point3])

            # 삼각형 그리기
            painter.drawPolygon(triangle)

            painter.end()  # QPainter 종료

            # 그려진 픽셀 맵을 다시 QLabel에 설정
            self.map_label.setPixmap(robot_pixmap)

    def open_large_map(self):
        """좌표 저장을 위한 큰 맵 창 열기"""
        self.large_map_window = LargeMapWindow(self)
        self.large_map_window.show()

    def start_explore_lite(self):
        rospy.loginfo("Explore Lite 시작")
        if self.explore_process is None or self.explore_process.poll() is not None:
            try:
                self.explore_process = subprocess.Popen(
                    ["roslaunch", "explore_lite", "explore.launch"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                rospy.loginfo("Explore Lite가 실행되었습니다.")
            except Exception as e:
                rospy.logerr(f"Explore Lite 실행 중 오류 발생: {e}")
        else:
            rospy.loginfo("Explore Lite가 이미 실행 중입니다.")

    def stop_explore_lite(self):
        rospy.loginfo("Explore Lite 종료")
        if self.explore_process and self.explore_process.poll() is None:
            try:
                self.explore_process.terminate()
                self.explore_process = None
                rospy.loginfo("Explore Lite가 종료되었습니다.")
            except Exception as e:
                rospy.logerr(f"Explore Lite 종료 중 오류 발생: {e}")
        else:
            rospy.loginfo("Explore Lite가 실행 중이 아닙니다.")

    def load_saved_coordinates(self):
        """저장된 좌표를 파일에서 불러오기"""
        if os.path.exists("saved_coordinates.json"):
            with open("saved_coordinates.json", "r") as f:
                self.saved_coordinates = json.load(f)
            rospy.loginfo("저장된 좌표 불러오기 완료")

    def save_coordinates_to_file(self):
        """좌표를 파일로 저장"""
        with open("saved_coordinates.json", "w") as f:
            json.dump(self.saved_coordinates, f)
        rospy.loginfo("좌표 저장 완료")

    def toggle_saved_coordinates(self):
        """좌표 리스트를 토글하는 함수"""
        if self.list_window.isVisible():
            self.list_window.hide()  # 좌표 리스트 숨기기
            self.delete_coordinate_button.hide()
            self.move_coordinate_button.hide()
        else:
            # 좌표 리스트 업데이트 및 보여주기
            self.list_window.clear()
            for name, coords in self.saved_coordinates.items():
                self.list_window.addItem(f"{name}: {coords}")
            self.list_window.show()
            self.delete_coordinate_button.show()
            self.move_coordinate_button.show()

    def move_to_selected_coordinate(self):
        """선택된 좌표로 로봇을 이동"""
        selected_item = self.list_window.currentItem()
        if selected_item:
            name = selected_item.text().split(":")[0]
            x, y = self.saved_coordinates[name]

            rospy.loginfo(f"좌표로 이동: {name}, X: {x}, Y: {y}")

            # ROS로 목표 좌표 퍼블리시
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.orientation.w = 1.0  # 기본 회전값

            self.goal_publisher.publish(goal_msg)

    def delete_selected_coordinate(self):
        """선택된 좌표 삭제"""
        selected_item = self.list_window.currentItem()
        if selected_item:
            name = selected_item.text().split(":")[0]
            del self.saved_coordinates[name]
            self.save_coordinates_to_file()
            self.toggle_saved_coordinates()  # 리스트를 다시 로드
