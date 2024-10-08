#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from explore_lite_controller import ExploreLiteController
from PySide2.QtWidgets import QApplication

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 메인 윈도우 생성 및 실행
    window = ExploreLiteController()
    window.show()
    
    sys.exit(app.exec_())
