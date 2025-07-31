from PyQt5.QtCore import QSize, Qt, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QWidget, QGridLayout, QLineEdit, QLabel
import pyqtgraph as pg
import pyqtgraph.exporters
from PyQt5 import QtWidgets
import time
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
import sys
import threading
import queue
import os
from pyqtgraph.exporters import CSVExporter

# serial port initialization
ser = serial.Serial()
ser.port = 'COM3'
ser.baudrate = 115200
ser.timeout = 0.1
data_queue = queue.Queue()

# flags
tracking = False
voltage_tracking = False
current_tracking = False
serial_busy = False

try:
    ser.open()
    print("Successfully connected to serial port")
except Exception as e:
    print(f"Failed to open serial port: {e}")


# read voltage and current readings
def serial_reader():
    global tracking
    ser.write(("TRACK" + "\n").encode('utf-8'))
    time.sleep(0.005)
    while tracking:
        try:
            line = ser.readline().decode().strip()
            if line == "END":
                tracking = False
                print("Tracking stopped")
            elif line:
                parts = line.split(",")
                if len(parts) == 2:
                    voltage = float(parts[0])
                    current = float(parts[1])
                    window.new_data_signal.emit(voltage,current) #sends a signal for the graphing function
        except Exception:
            continue

#read only voltage readings
def voltage_reader():
    global voltage_tracking
    ser.write(("TRACKV\n").encode('utf-8'))
    time.sleep(0.005)
    while voltage_tracking:
        try:
            line = ser.readline().decode().strip()
            if line == "END":
                voltage_tracking = False
                print("Voltage tracking stopped")
            elif line:
                voltage = float(line)
                window.voltage_signal.emit(voltage)
        except Exception:
            continue

#read only current readings
def current_reader():
    global current_tracking
    ser.write(("TRACKI\n").encode('utf-8'))
    time.sleep(0.005)
    while current_tracking:
        try:
            line = ser.readline().decode().strip()
            if line == "END":
                current_tracking = False
                print("Current tracking stopped")
            elif line:
                current = float(line)
                window.current_signal.emit(current)
        except Exception:
            continue


def start_voltage_tracking():
    global voltage_tracking
    voltage_tracking = True
    print("Voltage tracking started")
    thread = threading.Thread(target=voltage_reader, daemon=True)
    thread.start()

def start_current_tracking():
    global current_tracking
    current_tracking = True
    print("Current tracking started")
    thread = threading.Thread(target=current_reader, daemon=True)
    thread.start()

def stop_all_tracking():
    global voltage_tracking, current_tracking
    voltage_tracking = False
    current_tracking = False
    print("Tracking stopped")

def start_tracking():
    global tracking
    tracking = True
    print("Tracking started")
    thread = threading.Thread(target=serial_reader, daemon=True)
    thread.start()


def stop_tracking():
    global tracking
    tracking = False
    print("Tracking stopped")


# send command to rp2040
def send_command(cmd):
    global serial_busy
    if serial_busy:
        print("Serial Busy")
        return
    if ser and ser.is_open:
        try:
            serial_busy = True
            ser.write((cmd + "\n").encode('utf-8'))
            print(f"Sent: {cmd}")
            time.sleep(0.5)
            lines = ser.readlines()
            if lines:
                response = lines[-1].decode('utf-8').strip()

            else:
                response = "no response"
                return None
            print(f"Received: {response}")
            window.output.setText(response)
            return response
        except Exception as e:
            print(f"Error: {e}")
            window.output.setText("Error sending command")
            return None
        finally:
            serial_busy = False
    return None


def on_button_press(command):
    print(f"Button pressed: {command}")
    send_command(command)


def on_closing():
    print("closing window")
    send_command("VSET0")
    send_command("OFF")
    if ser.is_open:
        ser.close()


# main window container
class MainWindow(QMainWindow):
    voltage_signal = pyqtSignal(float)
    current_signal = pyqtSignal(float)
    new_data_signal = pyqtSignal(float,float)

    def __init__(self):
        super().__init__()

        # link to css file
        css_path = os.path.join(os.path.dirname(__file__), "style.css")
        with open(css_path, "r") as f:
            self.setStyleSheet(f.read())
        self.setWindowTitle("High Voltage Supply Control")
        self.setMinimumSize(QSize(1600, 500))

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        layout = QGridLayout()
        central_widget.setLayout(layout)
        layout.setContentsMargins(10, 10, 10, 10)  # left, top, right, bottom
        layout.setSpacing(10)  # space between widgets
        layout.setVerticalSpacing(5)
        title = QLabel("High Voltage Supply Control")
        layout.addWidget(title, 0, 0, 1, 4)
        title.setAlignment(Qt.AlignCenter)
        title.setObjectName("title")

        head1 = QLabel("Control Options")
        layout.addWidget(head1, 1, 0, 1, 4)
        head1.setAlignment(Qt.AlignCenter)
        head1.setObjectName("head1")

        idbutton = QPushButton("Identify Board")
        layout.addWidget(idbutton, 2, 0)
        idbutton.clicked.connect(lambda: send_command("*IDN?"))

        onbutton = QPushButton("ON")
        layout.addWidget(onbutton, 2, 1)
        onbutton.clicked.connect(lambda: send_command("ON"))

        offbutton = QPushButton("OFF")
        layout.addWidget(offbutton, 2, 2)
        offbutton.clicked.connect(lambda: send_command("OFF"))

        statusbutton = QPushButton("Check Status")
        layout.addWidget(statusbutton, 2, 3)
        statusbutton.clicked.connect(lambda: send_command("STATUS?"))

        vreadbutton = QPushButton("Read Voltage")
        layout.addWidget(vreadbutton, 3, 0)
        vreadbutton.clicked.connect(lambda: send_command("VREAD"))

        ireadbutton = QPushButton("Read Current")
        layout.addWidget(ireadbutton, 3, 1)
        ireadbutton.clicked.connect(lambda: send_command("IREAD"))

        voltage_label = QLabel("Set Voltage")
        layout.addWidget(voltage_label, 3, 2)
        voltage_label.setAlignment(Qt.AlignCenter)
        voltage_label.setStyleSheet("font-size: 22px; padding:4px;")

        self.enter_voltage = QLineEdit()
        layout.addWidget(self.enter_voltage, 3, 3)
        self.enter_voltage.returnPressed.connect(self.on_enter)

        self.output = QLabel("Output Message")
        layout.addWidget(self.output, 4, 0, 1, 4)
        self.output.setAlignment(Qt.AlignCenter)
        self.output.setStyleSheet("font-size:20px; padding:4px;")

        head2 = QLabel("Graph Options")
        layout.addWidget(head2, 5, 0, 1, 4)
        head2.setAlignment(Qt.AlignCenter)
        head2.setObjectName("head2")

        start_volt_button = QPushButton("Start Tracking")
        layout.addWidget(start_volt_button, 6, 0)
        start_volt_button.clicked.connect(start_tracking)

        # start_curr_button = QPushButton("Start Current Tracking")
        # layout.addWidget(start_curr_button, 6, 1)
        # start_curr_button.clicked.connect(start_current_tracking)

        stopbutton = QPushButton("Stop Tracking")
        layout.addWidget(stopbutton, 6, 1)
        stopbutton.clicked.connect(stop_all_tracking)

        clearbutton = QPushButton("Clear Graph")
        layout.addWidget(clearbutton, 6, 2)
        clearbutton.clicked.connect(self.clear_graph)

        csvbutton = QPushButton("Export to CSV")
        layout.addWidget(csvbutton, 6, 3)
        csvbutton.clicked.connect(self.export_csv)

        self.voltage_signal.connect(self.handle_voltage_data)
        self.current_signal.connect(self.handle_current_data)
        self.new_data_signal.connect(self.handle_new_data)
        # when received signal from serial_reader(), start handling data
        self.plot_data = []

        self.voltage_data = []
        self.current_data = []

        # voltage graph config
        self.plot_widget_voltage = pg.PlotWidget()
        layout.addWidget(self.plot_widget_voltage, 0, 4, 5, 1)
        self.plot_widget_voltage.setTitle("Voltage Monitoring")
        self.plot_widget_voltage.setLabel("left", "Voltage (V)")
        self.plot_widget_voltage.setLabel("bottom", "Time (0.01 s)")
        self.plot_widget_voltage.setBackground('w')
        self.plot_voltage = self.plot_widget_voltage.plot(pen='y')
        self.plot_voltage.setPen(pg.mkPen(color="#4B0082", width=2))

        #current graph config
        self.plot_widget_current = pg.PlotWidget()
        layout.addWidget(self.plot_widget_current, 0, 5, 5, 1)
        self.plot_widget_current.setTitle("Current Monitoring")
        self.plot_widget_current.setLabel("left", "Current (mA)")
        self.plot_widget_current.setLabel("bottom", "Time (0.01 s)")
        self.plot_widget_current.setBackground('w')
        self.plot_current = self.plot_widget_current.plot(pen='r')
        self.plot_current.setPen(pg.mkPen(color="red", width=2))

        # timer config
        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

        buttons = []
        buttons.extend(
            [idbutton, onbutton, offbutton, statusbutton, vreadbutton, ireadbutton, start_volt_button, stopbutton, csvbutton,
             clearbutton])
        for btn in buttons:
            btn.setFixedSize(200, 100)
            btn.setStyleSheet("font-size: 18px; padding:6px;")

    def clear_graph(self):
        self.voltage_data = []
        self.current_data = []
        self.plot_voltage.setData([])
        self.plot_current.setData([])

    def export_csv(self):
        if not self.plot_data:
            self.output.setText("No data to export")
            return
        file_path = os.path.join(os.path.dirname(__file__), "data.csv")
        exporter = CSVExporter(self.plot_widget.plotItem)
        try:
            exporter.export(file_path)
            self.output.setText("Successfully exported!")
        except Exception as e:
            self.output.setText("An error occurred: {}".format(e))

    def handle_voltage_data(self, voltage):
        self.voltage_data.append(voltage)
        self.voltage_data = self.voltage_data[-1000:]

    def handle_current_data(self, current):
        self.current_data.append(current*1000)
        self.current_data = self.current_data[-1000:]

    def handle_new_data(self, voltage, current):
        if voltage is not None:
            self.voltage_data.append(voltage)
            self.voltage_data = self.voltage_data[-1000:]
        if current is not None:
            self.current_data.append(current)
            self.current_data = self.current_data[-1000:]

    def update_plot(self):
        self.plot_voltage.setData(self.voltage_data)
        self.plot_current.setData(self.current_data)

    def on_enter(self):
        vol = self.enter_voltage.text()
        voltage = f"VSET{vol}"
        send_command(voltage)
        self.enter_voltage.clear()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())