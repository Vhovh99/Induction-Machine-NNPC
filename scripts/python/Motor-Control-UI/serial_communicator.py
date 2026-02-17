import serial
import threading
import queue
import time
from typing import Optional, Callable
from config import SERIAL_BAUDRATE, SERIAL_TIMEOUT


class SerialCommunicator:
    """Handles serial communication with the motor controller."""
    
    def __init__(self, port: Optional[str] = None, baudrate: int = SERIAL_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        self.receive_thread = None
        
        # Queue for receiving telemetry data
        self.data_queue = queue.Queue()
        
        # Callbacks
        self.on_data_received = None
        self.on_connection_change = None
        self.on_error = None
    
    def list_ports(self):
        """List available serial ports."""
        import serial.tools.list_ports
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports
    
    def connect(self, port: str) -> bool:
        """Connect to the specified serial port."""
        try:
            self.port = port
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT
            )
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            if self.on_connection_change:
                self.on_connection_change(True, f"Connected to {port}")
            
            return True
        except Exception as e:
            self.running = False
            if self.on_error:
                self.on_error(f"Connection error: {str(e)}")
            return False
    
    def disconnect(self):
        """Disconnect from the serial port."""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=2.0)
        
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                if self.on_error:
                    self.on_error(f"Disconnection error: {str(e)}")
        
        if self.on_connection_change:
            self.on_connection_change(False, "Disconnected")
    
    def is_connected(self) -> bool:
        """Check if connected to the motor controller."""
        return self.serial_port is not None and self.serial_port.is_open and self.running
    
    def send_command(self, command: str) -> bool:
        """Send a command to the motor controller."""
        if not self.is_connected():
            if self.on_error:
                self.on_error("Not connected to motor controller")
            return False
        
        try:
            # Ensure command ends with newline
            if not command.endswith('\n'):
                command += '\n'
            
            print(f"[DEBUG] TX: {repr(command)}")
            self.serial_port.write(command.encode('ascii'))
            return True
        except Exception as e:
            if self.on_error:
                self.on_error(f"Send error: {str(e)}")
            return False
    
    def set_frequency(self, frequency: float) -> bool:
        """Set the frequency (Hz)."""
        return self.send_command(f"F {frequency}")
    
    def set_amplitude(self, amplitude: float) -> bool:
        """Set the amplitude (0.0 - 1.0)."""
        return self.send_command(f"A {amplitude}")
    
    def set_rpm(self, rpm: float) -> bool:
        """Set the RPM."""
        return self.send_command(f"R {rpm}")
    
    def start_motor(self) -> bool:
        """Start the motor."""
        return self.send_command("S")
    
    def stop_motor(self) -> bool:
        """Stop the motor."""
        return self.send_command("X")
    
    def _receive_loop(self):
        """Background thread for receiving telemetry data."""
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('ascii').strip()
                    if line:
                        # Debug: Print raw data received
                        print(f"[DEBUG] Raw RX: {repr(line)}")
                        self._parse_telemetry(line)
            except Exception as e:
                if self.running:  # Only report if we're still trying to run
                    if self.on_error:
                        self.on_error(f"Receive error: {str(e)}")
                    self.running = False
            
            time.sleep(0.01)  # Small delay to prevent CPU spinning
    
    def _parse_telemetry(self, line: str):
        """Parse telemetry data from the motor controller."""
        try:
            # Format: SPD:1495.20 CUR:0.45,-0.42
            data = {}
            
            # Remove any extra whitespace
            line = ' '.join(line.split())
            
            # Find and extract SPD
            if 'SPD:' in line:
                spd_start = line.index('SPD:') + 4
                spd_str = line[spd_start:].split()[0]
                data['speed'] = float(spd_str)
                print(f"[DEBUG] Parsed speed: {data['speed']}")
            
            # Find and extract CUR
            if 'CUR:' in line:
                cur_start = line.index('CUR:') + 4
                cur_str = line[cur_start:]
                currents = cur_str.split(',')
                if len(currents) >= 2:
                    data['ia'] = float(currents[0].strip())
                    data['ib'] = float(currents[1].strip())
                    print(f"[DEBUG] Parsed currents: Ia={data['ia']}, Ib={data['ib']}")
            
            # Add timestamp
            data['timestamp'] = time.time()
            
            # Add to queue
            self.data_queue.put(data)
            
            # Call callback if set
            if self.on_data_received:
                self.on_data_received(data)
        
        except Exception as e:
            print(f"[DEBUG] Parse error: {str(e)} (line: {line})")
            if self.on_error:
                self.on_error(f"Parse error: {str(e)} (line: {line})")
    
    def get_data(self) -> Optional[dict]:
        """Get the latest received telemetry data (non-blocking)."""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None
