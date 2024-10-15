from PyQt5.QtWidgets import QHBoxLayout, QPushButton

def add_thruster_buttons(self):
    button_layout = QHBoxLayout()
    self.thruster_buttons = {}

    # Define buttons
    buttons = {
        'Standby': toggle_button_state,  # Using the local function now
        'Position': toggle_button_state,
        'Sail': toggle_button_state,
        'Track': toggle_button_state,
    }

    # Add buttons
    for label, func in buttons.items():
        button = QPushButton(label)
        button.setCheckable(True)
        button.clicked.connect(lambda checked, b=button: func(self, b))  # Pass self here
        style_button(button)
        button_layout.addWidget(button)
        self.thruster_buttons[label] = button

    # Add Exit button
    exit_button = QPushButton('Exit')
    exit_button.clicked.connect(self.close_application)
    button_layout.addWidget(exit_button)

    return button_layout


def style_button(button):
    """Gi knappene standard stil."""
    button.setStyleSheet("""
        QPushButton {
            background-color: #1E90FF;
            border-radius: 10px;
            border: 2px solid #ffffff;
            color: white;
            padding: 10px;
            font-size: 14px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #00BFFF;
        }
        QPushButton:pressed {
            background-color: #007ACC;
        }
    """)

def toggle_button_state(self, button):
    """Toggle the button state and deactivate the previously active button."""
    if button == self.active_button:  # If the same button is pressed again, deactivate it
        button.setChecked(False)
        style_button(button)
        self.active_button = None
    else:
        if self.active_button:  # Deactivate the previously active button
            style_button(self.active_button)
            self.active_button.setChecked(False)

        # Activate the new button
        button.setChecked(True)
        button.setStyleSheet("""
            QPushButton {
                background-color: #90EE90;  /* Lys grønn */
                border-radius: 10px;
                border: 2px solid #ffffff;
                color: black;
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.active_button = button  # Update the active button
