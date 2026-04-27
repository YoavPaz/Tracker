from machine import UART, Pin
import time


class camera_controller:
    def __init__(self, tx=2, rx=3):
        self.uart = UART(
            0,
            baudrate=115200,
            bits=8,
            parity=None,
            stop=1,
            tx=Pin(tx),
            rx=Pin(rx)
        )
        
        self.buffer = b""
        self.current_value = 999
        self.wait = False
        
    def read_uart(self):
        if self.wait: return
        if self.uart.any():
            data = self.uart.read()
            if data:
                self.buffer += data

                while b"\n" in self.buffer:
                    line, self.buffer = self.buffer.split(b"\n", 1)
                    try:
                        value = int(line.decode().strip())
                        self.current_value = value
                        #print(f"[Camera Uart] last_value changed to {self.last_value}")
                        if value > 999:
                            print(f"[Camera Uart] Value: {value}")
                            self.wait = True
                            return True
                    except:
                        print("[Camera Uart] Invalid data:", line)
