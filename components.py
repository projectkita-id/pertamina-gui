from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLineEdit, QLabel, QPushButton

class Button(QPushButton):
    def __init__(self, text: str, parent):
        super().__init__(text, parent)

class Header(QLabel):
    def __init__(self, text: str, parent):
        super().__init__(text, parent)
        self.setObjectName("header")

class Text(QLabel):
    def __init__(self, text: str, parent):
        super().__init__(text, parent)
        self.setObjectName("text")

class Input(QLineEdit):
    def __init__(self, placeholder: str, parent):
        super().__init__(parent)
        self.setPlaceholderText(placeholder)
        self.setObjectName("input")