from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QFrame, QStyleOptionSlider, QStyle
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor

class CustomSliderWidget(QWidget):
    def __init__(self, min_val, max_val, scale_factor, label_text, update_method, thruster, parent=None):
        super().__init__(parent)

        self.scale_factor = scale_factor
        self.min_val = min_val
        self.max_val = max_val
        self.update_method = update_method
        self.thruster = thruster
        self.feedback_value = None  # Initialize feedback value

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        min_label = QLabel(f'{min_val}')
        max_label = QLabel(f'{max_val}')

        slider_layout = QHBoxLayout()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(min_val * scale_factor))
        self.slider.setMaximum(int(max_val * scale_factor))
        self.slider.setFixedWidth(300)  # Set a fixed width for the sliders
        self.slider.valueChanged.connect(lambda value, thr=thruster: update_method(value / scale_factor, thr))

        slider_layout.addWidget(min_label)
        slider_layout.addWidget(self.slider)
        slider_layout.addWidget(max_label)

        self.layout.addLayout(slider_layout)

        # Custom frame to draw the zero line, only if 0 is not an endpoint
        #if min_val != 0 and max_val != 0:
            #self.zero_line_frame = QFrame()
            #self.zero_line_frame.setFixedHeight(20)  # Adjust the height to 30 pixels
            #self.zero_line_frame.setFrameShape(QFrame.HLine)
            #self.zero_line_frame.setFrameShadow(QFrame.Sunken)
            #self.layout.addWidget(self.zero_line_frame)

    def set_feedback_value(self, value):
        self.feedback_value = value
        self.update()  # Trigger a repaint to show the feedback value

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)

        # Draw zero line if applicable
        if self.min_val != 0 and self.max_val != 0:
            painter.setPen(QColor(0, 0, 0))
            zero_position = int((0 - self.min_val) / (self.max_val - self.min_val) * (self.slider.width() - self.get_slider_handle_width()))
            zero_x = self.slider.geometry().left() + zero_position + self.get_slider_handle_width() // 2
            slider_geometry = self.slider.geometry()
            line_length = slider_geometry.height() * 1.5
            painter.drawLine(int(zero_x), int(slider_geometry.top() - (line_length - slider_geometry.height()) / 2), int(zero_x), int(slider_geometry.bottom() + (line_length - slider_geometry.height()) / 2))

        # Draw feedback indicator if available
        if self.feedback_value is not None:
            feedback_position = int((self.feedback_value - self.min_val) / (self.max_val - self.min_val) * (self.slider.width() - self.get_slider_handle_width()))
            feedback_x = self.slider.geometry().left() + feedback_position + self.get_slider_handle_width() // 2
            slider_top = self.slider.geometry().top()
            slider_bottom = self.slider.geometry().bottom()
            bar_height = slider_top - 10  # 10 px height above the slider

            # Draw feedback bar (above the slider)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(120, 120, 120))  # Black color
            painter.drawRect(int(feedback_x) - 4, slider_top - bar_height - 0, 8, 10)  # rectangle

        painter.end()

    def get_slider_handle_width(self):
        opt = QStyleOptionSlider()
        self.slider.initStyleOption(opt)
        return self.slider.style().subControlRect(QStyle.CC_Slider, opt, QStyle.SC_SliderHandle, self.slider).width()
