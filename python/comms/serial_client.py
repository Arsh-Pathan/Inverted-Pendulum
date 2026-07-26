import serial
import serial.tools.list_ports
from queue import Queue, Empty
from PyQt6.QtCore import QThread, pyqtSignal

class SerialClient(QThread):
    """
    Thread-safe serial client for interacting with the ESP32 Hardware Endpoint.
    Runs asynchronously in a background QThread, emitting received telemetry
    signals and draining command write queues without blocking the main event loop.
    """
    angle_received = pyqtSignal(float)
    status_changed = pyqtSignal(str, str) # text, color_state ("green", "gray", "red")

    def __init__(self, port: str, baud_rate: int = 115200, timeout: float = 0.01):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.running = False
        self.cmd_queue = Queue()
        self.ser = None

    def run(self):
        self.running = True
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.baud_rate
            self.ser.timeout = self.timeout
            self.ser.write_timeout = 0.05
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            
            print(f"[COM] Opened {self.port} at {self.baud_rate} baud (timeout={self.timeout}s)")
            self.status_changed.emit(f"Connected: {self.port}", "green")

            # Reset input buffer and discard initial partial line
            self.ser.reset_input_buffer()
            self.ser.readline()

            while self.running:
                # 1) Process all queued write commands
                while not self.cmd_queue.empty() and self.running:
                    try:
                        cmd = self.cmd_queue.get_nowait()
                        if self.ser and self.ser.is_open:
                            self.ser.write(cmd.encode("utf-8"))
                            self.ser.flush()
                    except Empty:
                        break
                    except Exception as e:
                        print(f"[COM WRITE ERROR] Failed sending command: {e}")

                # 2) Read incoming telemetry stream
                if not self.ser or not self.ser.is_open:
                    break

                raw = self.ser.readline()
                if not raw:
                    continue

                try:
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if not line or line.startswith("[") or line == "READY":
                        # Ignore system status / calibration logs from ESP32
                        if line:
                            print(f"[ESP32 LOG] {line}")
                        continue

                    angle_val = float(line)
                    self.angle_received.emit(angle_val)
                except ValueError:
                    # Line was not a pure float number
                    continue
                except Exception as e:
                    print(f"[COM DECODE ERROR] {e}")

            if self.ser and self.ser.is_open:
                self.ser.close()
            print(f"[COM] Closed port {self.port} normally.")
            self.status_changed.emit("Idle", "gray")

        except serial.SerialException as e:
            print(f"[COM ERROR] Serial exception on {self.port}: {e}")
            self.status_changed.emit("Disconnected", "red")
        except Exception as e:
            print(f"[COM ERROR] Unexpected error on {self.port}: {e}")
            self.status_changed.emit("Disconnected", "red")
        finally:
            self.running = False

    def send_command(self, cmd_str: str):
        """Thread-safe method to queue a command for transmission to the hardware endpoint."""
        self.cmd_queue.put(cmd_str)

    def stop_client(self):
        """Stops the reader loop and waits for thread termination."""
        self.running = False
        self.wait(2000)

    @staticmethod
    def list_available_ports():
        """Returns a list of detected COM port device names."""
        return [p.device for p in serial.tools.list_ports.comports()]
