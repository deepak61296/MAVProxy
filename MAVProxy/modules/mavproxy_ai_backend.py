#!/usr/bin/env python3
'''
AI Backend Integration Module for MAVProxy
Enables natural language drone control via ArduPilot AI Backend

This module intercepts unknown commands and sends them to the AI backend.
Original MAVProxy commands work normally.

Usage:
    module load ai_backend
    ai_backend enable
    arm drone
    takeoff to 10 meters
    land now

Author: ArduPilot AI Backend Team
Version: 2.1.0
'''

import time
from typing import Optional, Dict, Any

try:
    import requests
except ImportError:
    print("ERROR: requests library not found. Install with: pip install requests")
    requests = None

from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings


class AIBackendModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialize AI Backend module"""
        super(AIBackendModule, self).__init__(mpstate, "ai_backend", "AI Backend Integration")

        # Module settings
        self.ai_settings = mp_settings.MPSettings([
            ('enabled', bool, False),
            ('backend_url', str, 'http://localhost:5000'),
            ('mode', str, 'agent'),  # agent, ask, or script
            ('safe_mode', bool, False),  # Require y/n confirmation
            ('verbose', bool, False),
        ])

        # Add commands
        self.add_command('ai_backend', self.cmd_ai_backend,
                        "AI Backend control",
                        ['enable', 'disable', 'status', 'url <URL>',
                         'mode <agent|ask|script>', 'safe', 'unsafe'])

        # State tracking
        self.backend_available = False
        self.last_health_check = 0
        self.health_check_interval = 30

        # Pending command for safe_mode confirmation
        self.pending_command = None

        if requests is None:
            print("AI Backend: requests library not available")
            return

    def usage(self):
        return """Usage: ai_backend <command>
Commands:
  enable    - Enable AI backend (unknown commands go to AI)
  disable   - Disable AI backend
  status    - Show current status
  url <URL> - Set backend URL
  mode <m>  - Set mode: agent, ask, script
  safe      - Enable safe mode (y/n confirmation)
  unsafe    - Disable safe mode (direct execution)

Examples:
  ai_backend enable
  ai_backend url http://192.168.1.100:5000
"""

    def cmd_ai_backend(self, args):
        """Handle ai_backend commands"""
        if len(args) == 0:
            print(self.usage())
            return

        cmd = args[0].lower()

        if cmd == "enable":
            self.ai_settings.enabled = True
            self.check_backend_health()
            print("AI Backend: Enabled - unknown commands will be processed by AI")

        elif cmd == "disable":
            self.ai_settings.enabled = False
            self.pending_command = None
            print("AI Backend: Disabled")

        elif cmd == "status":
            self.show_status()

        elif cmd == "url":
            if len(args) < 2:
                print("Usage: ai_backend url <URL>")
                return
            self.ai_settings.backend_url = args[1]
            print(f"AI Backend: URL set to {args[1]}")
            self.check_backend_health()

        elif cmd == "mode":
            if len(args) < 2:
                print("Usage: ai_backend mode <agent|ask|script>")
                return
            mode = args[1].lower()
            if mode in ['agent', 'ask', 'script']:
                self.ai_settings.mode = mode
                print(f"AI Backend: Mode set to {mode}")
            else:
                print("Invalid mode. Use: agent, ask, or script")

        elif cmd == "safe":
            self.ai_settings.safe_mode = True
            print("AI Backend: Safe mode ON - commands require y/n confirmation")

        elif cmd == "unsafe":
            self.ai_settings.safe_mode = False
            print("AI Backend: Safe mode OFF - commands execute directly")

        elif cmd == "y" or cmd == "yes":
            if self.pending_command:
                self.execute_pending_command()
            else:
                print("AI Backend: No pending command")

        elif cmd == "n" or cmd == "no":
            if self.pending_command:
                self.pending_command = None
                print("AI Backend: Command cancelled")
            else:
                print("AI Backend: No pending command")

        else:
            print(self.usage())

    def show_status(self):
        """Display current status"""
        pending = self.pending_command['cmd_str'] if self.pending_command else "None"
        print(f"""AI Backend Status:
  Enabled:      {self.ai_settings.enabled}
  Backend URL:  {self.ai_settings.backend_url}
  Mode:         {self.ai_settings.mode}
  Safe Mode:    {self.ai_settings.safe_mode}
  Backend:      {'Connected' if self.backend_available else 'Disconnected'}
  Pending:      {pending}""")

    def check_backend_health(self):
        """Check if backend is available"""
        if requests is None:
            return False
        try:
            url = f"{self.ai_settings.backend_url}/health"
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                self.backend_available = True
                print("AI Backend: Connected")
                return True
        except Exception as e:
            if self.ai_settings.verbose:
                print(f"AI Backend: Connection failed - {e}")
        self.backend_available = False
        return False

    def unknown_command(self, args):
        """
        Called by MAVProxy when a command is not recognized.
        This is the hook to send unknown commands to AI backend.
        """
        if not self.ai_settings.enabled:
            return False  # Let MAVProxy show "Unknown command"

        if not args:
            return False

        # Handle y/n for safe mode
        if len(args) == 1 and args[0].lower() in ['y', 'yes']:
            if self.pending_command:
                self.execute_pending_command()
                return True
            return False

        if len(args) == 1 and args[0].lower() in ['n', 'no']:
            if self.pending_command:
                self.pending_command = None
                print("AI Backend: Command cancelled")
                return True
            return False

        # Reconstruct the command
        command_text = ' '.join(args)

        if self.ai_settings.verbose:
            print(f"[AI] Unknown command: '{command_text}'")

        # Check backend health
        if not self.backend_available:
            if not self.check_backend_health():
                print("AI Backend: Not connected")
                return False

        # Send to AI backend
        print(f"AI Backend: Processing '{command_text}'...")
        self.process_ai_command(command_text)
        return True  # We handled it

    def get_telemetry(self) -> Dict[str, Any]:
        """Gather current telemetry from MAVProxy"""
        telemetry = {
            "battery": {},
            "gps": {},
            "status": {},
            "position": {},
            "attitude": {}
        }

        try:
            if 'SYS_STATUS' in self.master.messages:
                msg = self.master.messages['SYS_STATUS']
                telemetry["battery"] = {
                    "voltage": msg.voltage_battery / 1000.0,
                    "current": msg.current_battery / 100.0,
                    "remaining": msg.battery_remaining
                }

            if 'GPS_RAW_INT' in self.master.messages:
                msg = self.master.messages['GPS_RAW_INT']
                telemetry["gps"] = {
                    "latitude": msg.lat / 1e7,
                    "longitude": msg.lon / 1e7,
                    "altitude": msg.alt / 1000.0,
                    "satellites": msg.satellites_visible,
                    "fix_type": msg.fix_type
                }

            if 'HEARTBEAT' in self.master.messages:
                mode = self.master.flightmode
                armed = self.master.motors_armed()
                telemetry["status"] = {
                    "mode": mode,
                    "armed": armed
                }

            if 'GLOBAL_POSITION_INT' in self.master.messages:
                msg = self.master.messages['GLOBAL_POSITION_INT']
                telemetry["position"] = {
                    "latitude": msg.lat / 1e7,
                    "longitude": msg.lon / 1e7,
                    "altitude": msg.alt / 1000.0,
                    "relative_altitude": msg.relative_alt / 1000.0
                }

        except Exception as e:
            if self.ai_settings.verbose:
                print(f"AI Backend: Telemetry error - {e}")

        return telemetry

    def send_to_backend(self, message: str) -> Optional[Dict[str, Any]]:
        """Send message to AI backend"""
        if requests is None:
            return None

        try:
            url = f"{self.ai_settings.backend_url}/chat"
            payload = {
                "message": message,
                "mode": self.ai_settings.mode,
                "telemetry": self.get_telemetry()
            }
            response = requests.post(url, json=payload, timeout=15)
            if response.status_code == 200:
                return response.json()
            else:
                print(f"AI Backend: Error {response.status_code}")
                return None
        except Exception as e:
            print(f"AI Backend: Request failed - {e}")
            return None

    def process_ai_command(self, user_input: str):
        """Process command through AI backend"""
        response = self.send_to_backend(user_input)

        if not response:
            return

        # Display AI response
        ai_response = response.get('response', 'No response')
        print(f"AI: {ai_response}")

        # Check for command
        command = response.get('command')
        if command and command.get('type'):
            cmd_type = command['type']
            params = command.get('params', {})
            cmd_str = self.format_command(cmd_type, params)

            if self.ai_settings.safe_mode:
                # Safe mode: ask for confirmation
                self.pending_command = {
                    'type': cmd_type,
                    'params': params,
                    'cmd_str': cmd_str
                }
                print(f">>> Execute {cmd_str}? Type 'y' or 'n' <<<")
            else:
                # Direct execution
                print(f"Executing: {cmd_str}")
                self.execute_command(cmd_type, params)

    def execute_pending_command(self):
        """Execute pending command (safe mode)"""
        if not self.pending_command:
            return
        cmd = self.pending_command
        self.pending_command = None
        print(f"Executing: {cmd['cmd_str']}")
        self.execute_command(cmd['type'], cmd['params'])

    def format_command(self, cmd_type: str, params: Dict[str, Any]) -> str:
        """Format command for display"""
        if cmd_type == "ARM":
            return "ARM"
        elif cmd_type == "DISARM":
            return "DISARM"
        elif cmd_type == "TAKEOFF":
            alt = params.get('altitude', 10)
            return f"TAKEOFF {alt}m"
        elif cmd_type == "LAND":
            return "LAND"
        elif cmd_type == "RTL":
            return "RTL"
        elif cmd_type == "GOTO":
            lat = params.get('latitude')
            lon = params.get('longitude')
            alt = params.get('altitude', 0)
            return f"GOTO {lat},{lon} at {alt}m"
        elif cmd_type == "GOTO_HOME":
            return "GOTO HOME"
        elif cmd_type == "CHANGE_MODE":
            mode = params.get('mode')
            return f"MODE {mode}"
        elif cmd_type == "MOVE_DIRECTION":
            direction = params.get('direction', 'unknown')
            distance = params.get('distance', 0)
            return f"MOVE {direction.upper()} {distance}m"
        elif cmd_type == "ALTITUDE_CHANGE":
            change = params.get('change', 0)
            direction = "UP" if change > 0 else "DOWN"
            return f"ALTITUDE {direction} {abs(change)}m"
        elif cmd_type == "GET_PARAM":
            param = params.get('parameter', 'unknown')
            return f"GET {param}"
        elif cmd_type == "SET_PARAM":
            param = params.get('parameter', 'unknown')
            value = params.get('value', 0)
            return f"SET {param}={value}"
        elif cmd_type == "REBOOT":
            return "REBOOT"
        else:
            return f"{cmd_type}"

    def execute_command(self, cmd_type: str, params: Dict[str, Any]):
        """Execute command via MAVProxy"""
        try:
            if cmd_type == "ARM":
                self.mpstate.functions.process_stdin("arm throttle")

            elif cmd_type == "DISARM":
                self.mpstate.functions.process_stdin("disarm")

            elif cmd_type == "TAKEOFF":
                altitude = params.get('altitude', 10)
                self.mpstate.functions.process_stdin("mode GUIDED")
                time.sleep(0.3)
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, altitude
                )

            elif cmd_type == "LAND":
                self.mpstate.functions.process_stdin("mode LAND")

            elif cmd_type == "RTL":
                self.mpstate.functions.process_stdin("mode RTL")

            elif cmd_type == "CHANGE_MODE":
                mode = params.get('mode', '')
                self.mpstate.functions.process_stdin(f"mode {mode}")

            elif cmd_type == "GOTO":
                lat = params.get('latitude')
                lon = params.get('longitude')
                alt = params.get('altitude', 0)
                # Use guided mode position target
                self.mpstate.functions.process_stdin("mode GUIDED")
                time.sleep(0.2)
                self.master.mav.mission_item_int_send(
                    self.target_system,
                    self.target_component,
                    0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    2, 0, 0, 0, 0, 0,
                    int(lat * 1e7), int(lon * 1e7), alt
                )

            elif cmd_type == "GOTO_HOME":
                # Return to home position
                self.mpstate.functions.process_stdin("mode RTL")

            elif cmd_type == "MOVE_DIRECTION":
                # Move in a direction (requires GUIDED mode and position offset)
                direction = params.get('direction', '').lower()
                distance = params.get('distance', 10)
                self._move_direction(direction, distance)

            elif cmd_type == "ALTITUDE_CHANGE":
                # Change altitude
                change = params.get('change', 0)
                self._change_altitude(change)

            elif cmd_type == "GET_PARAM":
                param_name = params.get('parameter', '')
                if param_name:
                    self.mpstate.functions.process_stdin(f"param show {param_name}")

            elif cmd_type == "SET_PARAM":
                param_name = params.get('parameter', '')
                value = params.get('value', 0)
                if param_name:
                    self.mpstate.functions.process_stdin(f"param set {param_name} {value}")

            elif cmd_type == "REBOOT":
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                    0, 1, 0, 0, 0, 0, 0, 0
                )

            else:
                print(f"AI Backend: Command {cmd_type} not implemented")

        except Exception as e:
            print(f"AI Backend: Execution error - {e}")

    def _move_direction(self, direction: str, distance: float):
        """Move in a cardinal direction"""
        import math

        try:
            # Get current position
            if 'GLOBAL_POSITION_INT' not in self.master.messages:
                print("AI Backend: No position data available")
                return

            msg = self.master.messages['GLOBAL_POSITION_INT']
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0

            # Calculate offset (approximate meters to degrees)
            # 1 degree lat ~ 111km, 1 degree lon varies with latitude
            lat_offset = distance / 111000.0
            lon_offset = distance / (111000.0 * math.cos(math.radians(lat)))

            if direction == 'north':
                lat += lat_offset
            elif direction == 'south':
                lat -= lat_offset
            elif direction == 'east':
                lon += lon_offset
            elif direction == 'west':
                lon -= lon_offset
            else:
                print(f"AI Backend: Unknown direction '{direction}'")
                return

            # Send to new position
            self.mpstate.functions.process_stdin("mode GUIDED")
            time.sleep(0.2)
            self.master.mav.mission_item_int_send(
                self.target_system,
                self.target_component,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2, 0, 0, 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7), alt
            )

        except Exception as e:
            print(f"AI Backend: Move direction error - {e}")

    def _change_altitude(self, change: float):
        """Change altitude by specified amount"""
        try:
            # Get current position
            if 'GLOBAL_POSITION_INT' not in self.master.messages:
                print("AI Backend: No position data available")
                return

            msg = self.master.messages['GLOBAL_POSITION_INT']
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0
            new_alt = max(0, current_alt + change)

            # Send position command with new altitude
            self.mpstate.functions.process_stdin("mode GUIDED")
            time.sleep(0.2)
            self.master.mav.mission_item_int_send(
                self.target_system,
                self.target_component,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2, 0, 0, 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7), new_alt
            )

        except Exception as e:
            print(f"AI Backend: Altitude change error - {e}")

    def idle_task(self):
        """Periodic tasks"""
        if self.ai_settings.enabled:
            now = time.time()
            if now - self.last_health_check > self.health_check_interval:
                self.last_health_check = now
                # Silent health check
                if requests:
                    try:
                        url = f"{self.ai_settings.backend_url}/health"
                        response = requests.get(url, timeout=2)
                        self.backend_available = response.status_code == 200
                    except:
                        self.backend_available = False

    def unload(self):
        """Cleanup on unload"""
        pass


def init(mpstate):
    return AIBackendModule(mpstate)
