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

    def set_mode_vf(self) -> bool:
        """Switch to V/F control mode."""
        return self.send_command("M V")

    def set_mode_foc(self) -> bool:
        """Switch to FOC control mode."""
        return self.send_command("M F")

    def set_id_current(self, current: float) -> bool:
        """Set Id (flux) reference current (A)."""
        return self.send_command(f"I {current}")

    def set_iq_current(self, current: float) -> bool:
        """Set Iq (torque) reference current (A)."""
        return self.send_command(f"Q {current}")

    def set_id_pid(self, kp: float, ki: float, kd: float) -> bool:
        """Tune Id PID controller."""
        return self.send_command(f"P {kp} {ki} {kd}")

    def set_iq_pid(self, kp: float, ki: float, kd: float) -> bool:
        """Tune Iq PID controller."""
        return self.send_command(f"T {kp} {ki} {kd}")
    
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
            # Format: IDX:1 SPD:-1670.95 CUR:13.21,13.25 CLA:13.21,22.93 PARK:26.44,1.01 TH_M:3.65 TH_E:1.01
            data = {}
            
            # Remove any extra whitespace
            line = ' '.join(line.split())

            tokens = line.split()
            for token in tokens:
                if token.startswith('IDX:'):
                    data['index'] = int(token.split(':', 1)[1])
                elif token.startswith('SPD:'):
                    data['speed'] = float(token.split(':-', 1)[1])
                    print(f"[DEBUG] Parsed speed: {data['speed']}")
                elif token.startswith('CUR:'):
                    values = token.split(':', 1)[1].split(',')
                    if len(values) >= 2:
                        data['ia'] = float(values[0].strip())
                        data['ib'] = float(values[1].strip())
                        print(f"[DEBUG] Parsed currents: Ia={data['ia']}, Ib={data['ib']}")
                elif token.startswith('CLA:'):
                    values = token.split(':', 1)[1].split(',')
                    if len(values) >= 2:
                        data['i_alpha'] = float(values[0].strip())
                        data['i_beta'] = float(values[1].strip())
                        print(f"[DEBUG] Parsed clarke: Ialpha={data['i_alpha']}, Ibeta={data['i_beta']}")
                elif token.startswith('PARK:'):
                    values = token.split(':', 1)[1].split(',')
                    if len(values) >= 2:
                        data['id'] = float(values[0].strip())
                        data['iq'] = float(values[1].strip())
                        print(f"[DEBUG] Parsed park: Id={data['id']}, Iq={data['iq']}")
                elif token.startswith('TH_M:'):
                    data['theta_mechanical'] = float(token.split(':', 1)[1])
                    print(f"[DEBUG] Parsed theta_mechanical: {data['theta_mechanical']}")
                elif token.startswith('TH_E:'):
                    data['theta_electrical'] = float(token.split(':', 1)[1])
                    print(f"[DEBUG] Parsed theta_electrical: {data['theta_electrical']}")
                elif token.startswith('THETA:'):  # Keep backward compatibility
                    data['theta_electrical'] = float(token.split(':', 1)[1])
                    print(f"[DEBUG] Parsed theta (legacy): {data['theta_electrical']}")
            
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
