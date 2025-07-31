from machine import UART, Pin
import time
import sys

# Initialize UART1 on GPIO4 (TX) and GPIO5 (RX)
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))


# print("UART Host Ready. Type a command:")

def loop():
    while True:
        try:
            line = sys.stdin.readline()
            if line:
                uart.write(line + "\n")
                time.sleep(0.3)
                response = b""
                while uart.any():
                    response += uart.read()
                    if response:
                        print(response.decode().strip())
                    else:
                        print("No response received")


        except KeyboardInterrupt:
            print("Exiting...")
            break


if __name__ == "__main__":
    loop()

