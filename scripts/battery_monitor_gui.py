#!/usr/bin/env python3

import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Float32
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
from PyQt5.QtCore import Qt, pyqtSignal

class BatteryMonitorNode(Node):
    def __init__(self, update_callback):
        super().__init__('battery_monitor_gui')
        
        # Callback to update GUI (thread-safe signal emit)
        self.update_callback = update_callback
        
        # Parameters
        self.declare_parameter('max_voltage', 24.0)
        self.declare_parameter('min_voltage', 20.0)
        self.max_voltage = self.get_parameter('max_voltage').value
        self.min_voltage = self.get_parameter('min_voltage').value
        
        # Subscriber
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10)
        
        # Publisher for simplified status
        self.status_pub = self.create_publisher(String, '/battery/status_str', 10)
        # Publisher for Override (Critical for Auto Docking Node)
        self.override_pub = self.create_publisher(Float32, '/battery/override', 10)
        
        self.override_voltage = -1.0 # -1 means disabled
        self.current_voltage = 0.0
        self.percentage = 0.0
        self.status_str = "UNKNOWN"
        
        self.get_logger().info('Battery Monitor GUI Started')

    def battery_callback(self, msg):
        if self.override_voltage >= 0:
            self.current_voltage = self.override_voltage
        else:
            self.current_voltage = msg.voltage
            
        # Calculate Percentage
        self.percentage = (self.current_voltage - self.min_voltage) / (self.max_voltage - self.min_voltage)
        self.percentage = max(0.0, min(1.0, self.percentage))
        
        percentage_display = self.percentage * 100.0
        
        # Determine Status (Simplified for GUI)
        if self.override_voltage >= 0:
            self.status_str = "MANUAL_OVERRIDE"
        elif msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            self.status_str = "CHARGING"
        elif msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
            self.status_str = "DISCHARGING"
        else:
            self.status_str = "IDLE/FULL"
            
        # Publish
        log_msg = f'Battery: {self.current_voltage:.2f}V ({percentage_display:.1f}%) | Status: {self.status_str}'
        self.status_pub.publish(String(data=log_msg))
        
        # Update GUI via signal callback
        if self.update_callback:
            self.update_callback(self.current_voltage, percentage_display, self.status_str)

class BatterySimPanel(QWidget):
    # Signal for thread-safe GUI updates: voltage, percentage, status
    update_signal = pyqtSignal(float, float, str)

    def __init__(self):
        super().__init__()
        self.ros_node = None
        self.update_signal.connect(self.update_display)
        self.initUI()
        
    def set_node(self, node):
        self.ros_node = node

    def initUI(self):
        self.setWindowTitle('AMR Battery Monitor & Sim')
        self.setGeometry(100, 100, 350, 250)

        layout = QVBoxLayout()

        # Monitor Section
        self.lbl_info = QLabel('Current Status', self)
        self.lbl_info.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self.lbl_info)
        
        self.lbl_display = QLabel('Waiting for data...', self)
        self.lbl_display.setStyleSheet("font-size: 12px; border: 1px solid gray; padding: 5px;")
        layout.addWidget(self.lbl_display)

        layout.addSpacing(20)

        # Simulator Section
        self.lbl_sim = QLabel('Simulation Override', self)
        self.lbl_sim.setStyleSheet("font-weight: bold; color: blue;")
        layout.addWidget(self.lbl_sim)

        self.btn_override = QPushButton('Enable Manual Override', self)
        self.btn_override.setCheckable(True)
        self.btn_override.clicked.connect(self.on_override_toggle)
        layout.addWidget(self.btn_override)

        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setMinimum(200) # 20.0 V
        self.slider.setMaximum(240) # 24.0 V
        self.slider.setValue(240)
        self.slider.setEnabled(False)
        self.slider.valueChanged.connect(self.on_slider_change)
        layout.addWidget(self.slider)
        
        self.lbl_slider_val = QLabel('Slider: 24.0 V', self)
        self.lbl_slider_val.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.lbl_slider_val)

        self.setLayout(layout)

    def on_override_toggle(self):
        if not self.ros_node: return
        
        is_active = self.btn_override.isChecked()
        self.slider.setEnabled(is_active)
        
        if is_active:
            self.btn_override.setText('Override ACTIVE (Using Slider)')
            self.btn_override.setStyleSheet("background-color: red; color: white;")
            # Apply current slider value immediately
            self.on_slider_change(self.slider.value())
        else:
            self.btn_override.setText('Enable Manual Override')
            self.btn_override.setStyleSheet("")
            self.ros_node.override_voltage = -1.0 # Disable

    def on_slider_change(self, value):
        if not self.ros_node: return
        
        if self.btn_override.isChecked():
            voltage = value / 10.0
            self.ros_node.override_voltage = voltage
            self.lbl_slider_val.setText(f'Slider: {voltage:.1f} V')
            
            # Publish Override for external nodes
            msg = Float32()
            msg.data = voltage
            self.ros_node.override_pub.publish(msg)

    def update_display(self, volt, pct, status):
        color = "green" if pct > 40 else "orange" if pct > 20 else "red"
        self.lbl_display.setText(f"Voltage: {volt:.2f} V\nLevel: {pct:.1f} %\nStatus: {status}")
        self.lbl_display.setStyleSheet(f"font-size: 16px; border: 2px solid {color}; padding: 10px; color: {color};")

def main():
    rclpy.init()
    
    app = QApplication(sys.argv)
    gui = BatterySimPanel()
    
    # Callback wrapper to emit signal
    def emit_update(v, p, s):
        gui.update_signal.emit(v, p, s)

    ros_node = BatteryMonitorNode(emit_update)
    gui.set_node(ros_node)
    
    # Run ROS spinning in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    ros_thread.start()
    
    gui.show()
    
    try:
        app.exec_()
    finally:
        # Proper shutdown
        ros_node.destroy_node()
        rclpy.shutdown()
        # ros_thread will die because it's daemon, or we could join it if rclpy.shutdown() breaks the spin loop
        
if __name__ == '__main__':
    main()
