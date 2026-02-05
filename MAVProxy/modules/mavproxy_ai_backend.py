#!/usr/bin/env python3
'''
AI Backend Integration Module for MAVProxy
Enables natural language drone control via ArduPilot AI Backend

This module uses an input_handler to intercept ALL commands before MAVProxy
processes them, allowing natural language like "arm the drone" to work.

Usage:
    module load ai_backend
    ai_backend enable
    arm the drone
    takeoff to 10 meters
    land now

Author: ArduPilot AI Backend Team
Version: 2.2.0
'''

import time
import shlex
from typing import Optional, Dict, Any, Set

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

        # Store original input handler
        self.original_input_handler = None

        # Natural language indicators - words that suggest AI processing
        self.nl_indicators = {
            'the', 'to', 'please', 'now', 'meters', 'meter', 'm',
            'feet', 'foot', 'ft', 'seconds', 'second', 'drone',
            'copter', 'vehicle', 'aircraft', 'it', 'my', 'current',
            'what', 'how', 'where', 'when', 'why', 'is', 'are',
            'can', 'could', 'would', 'should', 'will', 'do', 'does',
            'and', 'then', 'after', 'before', 'north', 'south',
            'east', 'west', 'up', 'down', 'forward', 'backward',
            'left', 'right', 'higher', 'lower', 'faster', 'slower'
        }

        # Known MAVProxy command first words (partial list)
        self.mavproxy_commands = {
            'arm', 'disarm', 'mode', 'param', 'wp', 'rally', 'fence',
            'module', 'link', 'output', 'set', 'status', 'rc', 'servo',
            'relay', 'repeat', 'alias', 'watch', 'graph', 'log', 'terrain',
            'script', 'time', 'shell', 'position', 'velocity', 'guided',
            'takeoff', 'land', 'rtl', 'auto', 'loiter', 'stabilize'
        }

        # Valid subcommands for common commands (to detect valid vs natural language)
        self.valid_subcommands = {
            'arm': {'check', 'uncheck', 'skip', 'unskip', 'list', 'throttle',
                   'safetyon', 'safetyoff', 'safetystatus', 'bits', 'prearms'},
            'mode': {'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER',
                    'RTL', 'CIRCLE', 'LAND', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE',
                    'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB', 'GUIDED_NOGPS',
                    'SMART_RTL', 'FLOWHOLD', 'FOLLOW', 'ZIGZAG', 'SYSTEMID',
                    'AUTOROTATE', 'AUTO_RTL', 'TURTLE'},
            'param': {'show', 'set', 'load', 'save', 'diff', 'download', 'help',
                     'apropos', 'check', 'revert', 'undo', 'fetch', 'ftp'},
        }

        if requests is None:
            print("AI Backend: requests library not available")
            return

    def usage(self):
        return """Usage: ai_backend <command>
Commands:
  enable    - Enable AI backend (natural language commands)
  disable   - Disable AI backend
  status    - Show current status
  url <URL> - Set backend URL
  mode <m>  - Set mode: agent, ask, script
  safe      - Enable safe mode (y/n confirmation)
  unsafe    - Disable safe mode (direct execution)

Examples:
  ai_backend enable
  ai_backend url http://192.168.1.100:5000

When enabled, you can use natural language:
  arm the drone
  takeoff to 10 meters
  move north 20 meters
  what is my altitude?
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
            # Install our input handler
            self._install_input_handler()
            print("AI Backend: Enabled - natural language commands active")

        elif cmd == "disable":
            self.ai_settings.enabled = False
            self.pending_command = None
            # Remove our input handler
            self._remove_input_handler()
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

    def _install_input_handler(self):
        """Install our input handler to intercept all commands"""
        if self.mpstate.functions.input_handler != self._input_handler:
            self.original_input_handler = self.mpstate.functions.input_handler
            self.mpstate.functions.input_handler = self._input_handler
            if self.ai_settings.verbose:
                print("AI Backend: Input handler installed")

    def _remove_input_handler(self):
        """Remove our input handler"""
        if self.mpstate.functions.input_handler == self._input_handler:
            self.mpstate.functions.input_handler = self.original_input_handler
            if self.ai_settings.verbose:
                print("AI Backend: Input handler removed")

    def _input_handler(self, line):
        """
        Custom input handler that intercepts ALL input.
        Decides whether to send to AI backend or let MAVProxy handle it.
        """
        line = line.strip()
        if not line:
            return

        # Handle y/n confirmation for safe mode
        if line.lower() in ['y', 'yes'] and self.pending_command:
            self.execute_pending_command()
            return
        if line.lower() in ['n', 'no'] and self.pending_command:
            self.pending_command = None
            print("AI Backend: Command cancelled")
            return

        # Check if this looks like natural language or a valid MAVProxy command
        if self._is_natural_language(line):
            # Send to AI backend
            if self.ai_settings.verbose:
                print(f"[AI] Processing: '{line}'")

            if not self.backend_available:
                if not self.check_backend_health():
                    print("AI Backend: Not connected, trying MAVProxy...")
                    self._process_mavproxy(line)
                    return

            print(f"AI Backend: Processing '{line}'...")
            self.process_ai_command(line)
        else:
            # Let MAVProxy handle it
            if self.ai_settings.verbose:
                print(f"[MAVProxy] Processing: '{line}'")
            self._process_mavproxy(line)

    def _process_mavproxy(self, line):
        """Send command to MAVProxy's normal processing"""
        # Temporarily remove our handler to avoid recursion
        self.mpstate.functions.input_handler = self.original_input_handler
        try:
            # Import and call process_stdin
            from MAVProxy.mavproxy import process_stdin
            process_stdin(line)
        finally:
            # Reinstall our handler
            self.mpstate.functions.input_handler = self._input_handler

    def _is_natural_language(self, line: str) -> bool:
        """
        Determine if the input is natural language vs a valid MAVProxy command.
        Returns True if it should go to AI backend.
        """
        try:
            args = shlex.split(line.lower())
        except:
            args = line.lower().split()

        if not args:
            return False

        first_word = args[0]

        # ai_backend commands always go to MAVProxy
        if first_word == 'ai_backend':
            return False

        # Built-in commands that should always go to MAVProxy
        builtin_always = {'help', 'exit', 'module', 'link', 'output', 'set',
                         'alias', 'watch', 'script', 'shell'}
        if first_word in builtin_always:
            return False

        # If command has natural language indicators, likely AI
        words_set = set(args)
        nl_matches = words_set & self.nl_indicators
        if len(nl_matches) >= 1 and len(args) > 1:
            # Has natural language words - probably for AI
            # But check if it's still a valid MAVProxy command
            if first_word in self.valid_subcommands:
                valid_subs = self.valid_subcommands[first_word]
                if len(args) > 1 and args[1].upper() in valid_subs:
                    # Valid subcommand like "mode GUIDED"
                    return False
            return True

        # Check if it's a known MAVProxy command with valid syntax
        if first_word in self.valid_subcommands:
            valid_subs = self.valid_subcommands[first_word]
            if len(args) == 1:
                # Just the command alone - let MAVProxy show usage
                return False
            if len(args) > 1:
                # Check if second word is a valid subcommand
                if args[1].upper() in valid_subs or args[1] in valid_subs:
                    return False
                # Invalid subcommand like "arm the" - send to AI
                return True

        # Commands not in our tracking - check for natural language patterns
        if len(args) >= 3:
            # Multiple words often indicate natural language
            return True

        # Single word or two words without NL indicators - let MAVProxy try
        return False

    def show_status(self):
        """Display current status"""
        pending = self.pending_command['cmd_str'] if self.pending_command else "None"
        handler = "Active" if self.mpstate.functions.input_handler == self._input_handler else "Inactive"
        print(f"""AI Backend Status:
  Enabled:        {self.ai_settings.enabled}
  Backend URL:    {self.ai_settings.backend_url}
  Mode:           {self.ai_settings.mode}
  Safe Mode:      {self.ai_settings.safe_mode}
  Backend:        {'Connected' if self.backend_available else 'Disconnected'}
  Input Handler:  {handler}
  Pending:        {pending}""")

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
            response = requests.post(url, json=payload, timeout=30)
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
                self._process_mavproxy("arm throttle")

            elif cmd_type == "DISARM":
                self._process_mavproxy("disarm")

            elif cmd_type == "TAKEOFF":
                altitude = params.get('altitude', 10)
                self._process_mavproxy("mode GUIDED")
                time.sleep(0.3)
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, altitude
                )

            elif cmd_type == "LAND":
                self._process_mavproxy("mode LAND")

            elif cmd_type == "RTL":
                self._process_mavproxy("mode RTL")

            elif cmd_type == "CHANGE_MODE":
                mode = params.get('mode', '')
                self._process_mavproxy(f"mode {mode}")

            elif cmd_type == "GOTO":
                lat = params.get('latitude')
                lon = params.get('longitude')
                alt = params.get('altitude', 0)
                self._process_mavproxy("mode GUIDED")
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
                self._process_mavproxy("mode RTL")

            elif cmd_type == "MOVE_DIRECTION":
                direction = params.get('direction', '').lower()
                distance = params.get('distance', 10)
                self._move_direction(direction, distance)

            elif cmd_type == "ALTITUDE_CHANGE":
                change = params.get('change', 0)
                self._change_altitude(change)

            elif cmd_type == "GET_PARAM":
                param_name = params.get('parameter', '')
                if param_name:
                    self._process_mavproxy(f"param show {param_name}")

            elif cmd_type == "SET_PARAM":
                param_name = params.get('parameter', '')
                value = params.get('value', 0)
                if param_name:
                    self._process_mavproxy(f"param set {param_name} {value}")

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
            if 'GLOBAL_POSITION_INT' not in self.master.messages:
                print("AI Backend: No position data available")
                return

            msg = self.master.messages['GLOBAL_POSITION_INT']
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0

            lat_offset = distance / 111000.0
            lon_offset = distance / (111000.0 * max(0.1, abs(math.cos(math.radians(lat)))))

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

            self._process_mavproxy("mode GUIDED")
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
            if 'GLOBAL_POSITION_INT' not in self.master.messages:
                print("AI Backend: No position data available")
                return

            msg = self.master.messages['GLOBAL_POSITION_INT']
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0
            new_alt = max(0, current_alt + change)

            self._process_mavproxy("mode GUIDED")
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
                if requests:
                    try:
                        url = f"{self.ai_settings.backend_url}/health"
                        response = requests.get(url, timeout=2)
                        self.backend_available = response.status_code == 200
                    except:
                        self.backend_available = False

    def unload(self):
        """Cleanup on unload"""
        self._remove_input_handler()


def init(mpstate):
    return AIBackendModule(mpstate)
