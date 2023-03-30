import sys  # sys нужен для передачи argv в QApplication
from PyQt5 import QtWidgets
import design
from port import serial_ports,speeds
import serial



class ArduApp(QtWidgets.QMainWindow, design.Ui_Form):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.Port.addItems(serial_ports())
        self.Speed.addItems(speeds)
        self.realport = None
        self.ConnectButton.clicked.connect(self.connect)
        self.EnableBtn.clicked.connect(self.send)
        self.DisableBtn.clicked.connect(self.send_no)


    def connect(self):
        try:
            self.realport = serial.Serial(self.Port.currentText(),int(self.Speed.currentText()))
            self.ConnectButton.setStyleSheet("background-color: green")
            self.ConnectButton.setText('Подключено')
        except Exception as e:
            print(e)

    def send(self):
        if self.realport:
                self.realport.write(b'1')
    def send_no(self):
        if self.realport:
                self.realport.write(b'0')



def main():
    app = QtWidgets.QApplication(sys.argv)
    window = ArduApp()
    window.show()
    app.exec_()
    


if __name__ == '__main__':
    main()