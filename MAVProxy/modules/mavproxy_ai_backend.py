#!/usr/bin/env python3
'''
AI Backend Integration Module for MAVProxy
Enables natural language drone control via ArduPilot AI Backend

This module detects plain English commands, sends them to the AI backend
for processing, and executes the resulting commands after user confirmation.

Author: ArduPilot AI Backend Team
Date: 2026-02-05
Version: 1.2.0

Usage:
    module load ai_backend
    ai_backend enable
    arm the drone
    y
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
            ('auto_confirm', bool, False),  # Auto-confirm low-risk commands
            ('verbose', bool, True),  # Debug output enabled by default
        ])

        # Add commands
        self.add_command('ai_backend', self.cmd_ai_backend,
                        "AI Backend control",
                        ['enable', 'disable', 'status', 'url <URL>',
                         'mode <agent|ask|script>', 'set (SETTING)'])

        # State tracking
        self.backend_available = False
        self.last_health_check = 0
        self.health_check_interval = 30  # seconds

        # Command history
        self.command_history = []

        # Pending command for confirmation (state-based confirmation)
        self.pending_command = None  # {type, params, cmd_str}

        # Flag to prevent re-entry during input processing
        self.processing_input = False

        # Check if requests is available
        if requests is None:
            self.say("AI Backend: requests library not available. Module disabled.")
            return

        # Initial health check if enabled
        if self.ai_settings.enabled:
            self.check_backend_health()

    def usage(self):
        """Show help on command line options"""
        return """Usage: ai_backend <command>
Commands:
  enable              - Enable AI backend integration
  disable             - Disable AI backend integration
  status              - Show current status
  url <URL>           - Set backend URL (default: http://localhost:5000)
  mode <mode>         - Set mode: agent (execute), ask (query), script (generate)
  set <setting>       - Configure settings
  confirm             - Confirm pending command (same as 'y')
  cancel              - Cancel pending command (same as 'n')

Examples:
  ai_backend enable
  ai_backend url http://192.168.1.100:5000
  ai_backend mode agent
"""

    def cmd_ai_backend(self, args):
        """Handle ai_backend commands"""
        if len(args) == 0:
            print(self.usage())
            return

        cmd = args[0].lower()

        if cmd == "enable":
            self.ai_settings.enabled = True
            # Register input handler to intercept ALL commands
            self.mpstate.functions.input_handler = self.handle_input
            self.check_backend_health()
            self.say("AI Backend: Enabled - Natural language commands active")

        elif cmd == "disable":
            self.ai_settings.enabled = False
            # Remove input handler to restore normal MAVProxy behavior
            self.mpstate.functions.input_handler = None
            self.pending_command = None
            self.say("AI Backend: Disabled")

        elif cmd == "status":
            self.show_status()

        elif cmd == "url":
            if len(args) < 2:
                print("Usage: ai_backend url <URL>")
                return
            self.ai_settings.backend_url = args[1]
            self.say(f"AI Backend: URL set to {args[1]}")
            self.check_backend_health()

        elif cmd == "mode":
            if len(args) < 2:
                print("Usage: ai_backend mode <agent|ask|script>")
                return
            mode = args[1].lower()
            if mode in ['agent', 'ask', 'script']:
                self.ai_settings.mode = mode
                self.say(f"AI Backend: Mode set to {mode}")
            else:
                print("Invalid mode. Use: agent, ask, or script")

        elif cmd == "confirm":
            # Manual confirm command
            if self.pending_command:
                self.execute_pending_command()
            else:
                self.say("AI Backend: No pending command to confirm")

        elif cmd == "cancel":
            # Manual cancel command
            if self.pending_command:
                self.pending_command = None
                self.say("AI Backend: Command cancelled")
            else:
                self.say("AI Backend: No pending command to cancel")

        elif cmd == "set":
            self.ai_settings.command(args[1:])

        else:
            print(self.usage())

    def show_status(self):
        """Display current status"""
        pending = "None"
        if self.pending_command:
            pending = self.pending_command['cmd_str']

        status = f"""AI Backend Status:
  Enabled:        {self.ai_settings.enabled}
  Backend URL:    {self.ai_settings.backend_url}
  Mode:           {self.ai_settings.mode}
  Backend Health: {'Available' if self.backend_available else 'Unavailable'}
  Auto-confirm:   {self.ai_settings.auto_confirm}
  Pending Cmd:    {pending}
"""
        print(status)

    def check_backend_health(self):
        """Check if backend is available"""
        if requests is None:
            return False

        try:
            url = f"{self.ai_settings.backend_url}/health"
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                self.backend_available = True
                if self.ai_settings.verbose:
                    self.say("AI Backend: Connected")
                return True
        except Exception as e:
            if self.ai_settings.verbose:
                self.say(f"AI Backend: Connection failed - {str(e)}")

        self.backend_available = False
        return False

    def execute_pending_command(self):
        """Execute the pending command"""
        if not self.pending_command:
            return

        cmd = self.pending_command
        self.pending_command = None
        print(f"AI Backend: Executing {cmd['cmd_str']}...")
        self.execute_command(cmd['type'], cmd['params'])

    def handle_input(self, line):
        """
        Input handler that intercepts ALL commands when AI backend is enabled.
        """
        # Prevent re-entry
        if self.processing_input:
            return

        line = line.strip()
        if not line:
            return

        # Debug: show what we received
        if self.ai_settings.verbose:
            print(f"[AI Debug] Input received: '{line}' (pending: {self.pending_command is not None})")

        # Check if we're waiting for confirmation
        if self.pending_command is not None:
            self.handle_confirmation(line)
            return

        # Always allow ai_backend commands to pass through normally
        if line.lower().startswith('ai_backend'):
            self.pass_to_mavproxy(line)
            return

        # Check if this looks like natural language
        is_english = self.is_plain_english(line)
        if self.ai_settings.verbose:
            print(f"[AI Debug] is_plain_english('{line}') = {is_english}")

        if is_english:
            # Process through AI backend
            self.say(f"AI Backend: Processing '{line}'...")
            self.process_ai_command(line)
        else:
            # Pass through to normal MAVProxy processing
            if self.ai_settings.verbose:
                print(f"[AI Debug] Passing to MAVProxy: '{line}'")
            self.pass_to_mavproxy(line)

    def pass_to_mavproxy(self, line):
        """Pass a command to normal MAVProxy processing"""
        self.processing_input = True
        try:
            # Temporarily disable handler
            self.mpstate.functions.input_handler = None
            # Process through MAVProxy
            self.mpstate.input_queue.put(line)
        finally:
            # Re-enable handler after a small delay
            # Use idle_task to re-enable to avoid race condition
            self._reenable_handler = True
            self.processing_input = False

    def handle_confirmation(self, response):
        """Handle y/n confirmation for pending command"""
        original = response
        response = response.strip().lower()

        # Debug output
        if self.ai_settings.verbose:
            print(f"[AI Debug] Confirmation input: '{original}' -> '{response}'")

        # Be more lenient - check if starts with y or n
        if response.startswith('y'):
            self.execute_pending_command()
        elif response.startswith('n'):
            self.pending_command = None
            print("AI Backend: Command cancelled")
        else:
            # Not y or n - check if it's a new command
            # If user types a new plain English command, process it instead
            if self.is_plain_english(original):
                # Cancel current pending and process new command
                print("AI Backend: Previous command cancelled, processing new command...")
                self.pending_command = None
                self.process_ai_command(original)
            else:
                # Show prompt again
                cmd_str = self.pending_command['cmd_str']
                print(f">>> Enter 'y' to execute {cmd_str}, 'n' to cancel <<<")

    def is_plain_english(self, text: str) -> bool:
        """
        Detect if a command is plain English (not a MAVProxy command)
        """
        # Clean and normalize
        text = text.strip().lower()
        words = text.split()

        # Too short - probably a MAVProxy command
        if len(words) < 2:
            return False

        # Single word commands are always MAVProxy
        if len(words) == 1:
            return False

        # Question words - definitely natural language
        question_words = ['what', 'how', 'when', 'where', 'why', 'which', 'who', 'is', 'are', 'can']
        if words[0] in question_words:
            return True
        if any(word in words for word in ['what', 'how', 'when', 'where', 'why']):
            return True

        # Conversational starters - definitely natural language
        conversational = ['please', 'can', 'could', 'would', 'i', 'help', 'tell', 'show']
        if words[0] in conversational:
            return True

        # Action phrases with context - catches "arm the drone", "takeoff to 10m", etc.
        action_phrases = [
            'arm the', 'arm my', 'disarm the', 'disarm my',
            'takeoff to', 'take off to', 'takeoff',
            'land the', 'land my', 'land now', 'land at',
            'fly to', 'fly the', 'fly my',
            'go to', 'move to', 'return to', 'return home',
            'change mode', 'switch mode', 'set mode',
            'get altitude', 'get battery', 'get status',
            'show me', 'tell me', 'check the', 'check my',
            'the drone', 'the copter', 'the vehicle', 'my drone'
        ]
        if any(phrase in text for phrase in action_phrases):
            return True

        # Has articles/prepositions - indicates sentence structure
        structure_words = ['the', 'my', 'a', 'an']
        if any(word in words for word in structure_words):
            return True

        # Two word commands with 'to' - like "takeoff 10" vs "takeoff to"
        if 'to' in words and len(words) >= 2:
            return True

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
            # Battery
            if 'SYS_STATUS' in self.master.messages:
                msg = self.master.messages['SYS_STATUS']
                telemetry["battery"] = {
                    "voltage": msg.voltage_battery / 1000.0,
                    "current": msg.current_battery / 100.0,
                    "remaining": msg.battery_remaining
                }

            # GPS
            if 'GPS_RAW_INT' in self.master.messages:
                msg = self.master.messages['GPS_RAW_INT']
                telemetry["gps"] = {
                    "latitude": msg.lat / 1e7,
                    "longitude": msg.lon / 1e7,
                    "altitude": msg.alt / 1000.0,
                    "satellites": msg.satellites_visible,
                    "fix_type": msg.fix_type
                }

            # Status
            if 'HEARTBEAT' in self.master.messages:
                msg = self.master.messages['HEARTBEAT']
                mode = self.master.flightmode
                armed = self.master.motors_armed()
                telemetry["status"] = {
                    "mode": mode,
                    "armed": armed,
                    "system_status": msg.system_status
                }

            # Position
            if 'GLOBAL_POSITION_INT' in self.master.messages:
                msg = self.master.messages['GLOBAL_POSITION_INT']
                telemetry["position"] = {
                    "latitude": msg.lat / 1e7,
                    "longitude": msg.lon / 1e7,
                    "altitude": msg.alt / 1000.0,
                    "relative_altitude": msg.relative_alt / 1000.0
                }

            # Attitude
            if 'ATTITUDE' in self.master.messages:
                msg = self.master.messages['ATTITUDE']
                telemetry["attitude"] = {
                    "roll": msg.roll,
                    "pitch": msg.pitch,
                    "yaw": msg.yaw
                }

        except Exception as e:
            if self.ai_settings.verbose:
                self.say(f"AI Backend: Telemetry error - {str(e)}")

        return telemetry

    def send_to_backend(self, message: str) -> Optional[Dict[str, Any]]:
        """Send message to AI backend and get response"""
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
                self.say(f"AI Backend: Error {response.status_code}")
                return None

        except Exception as e:
            self.say(f"AI Backend: Request failed - {str(e)}")
            return None

    def process_ai_command(self, user_input: str):
        """Process a plain English command through AI backend"""
        if self.ai_settings.verbose:
            print(f"[AI Debug] process_ai_command called with: '{user_input}'")

        if not self.ai_settings.enabled:
            if self.ai_settings.verbose:
                print("[AI Debug] AI backend not enabled, returning")
            return

        if not self.backend_available:
            if self.ai_settings.verbose:
                print("[AI Debug] Backend not available, checking health...")
            # Try to reconnect
            if not self.check_backend_health():
                self.say("AI Backend: Backend not available. Check connection.")
                return

        if self.ai_settings.verbose:
            print("[AI Debug] Sending to backend...")

        # Send to backend
        response = self.send_to_backend(user_input)

        if self.ai_settings.verbose:
            print(f"[AI Debug] Backend response: {response}")

        if not response:
            print("AI Backend: No response from backend")
            return

        # Display AI response - use print() to show in main console
        ai_response = response.get('response', 'No response')
        print(f"AI: {ai_response}")

        # Check for command
        command = response.get('command')
        if command and command.get('type'):
            cmd_type = command['type']
            params = command.get('params', {})

            # Format command for display
            cmd_str = self.format_command(cmd_type, params)

            # Ask for confirmation (unless auto-confirm is enabled for low-risk)
            if self.ai_settings.auto_confirm and self.is_low_risk_command(cmd_type):
                print(f"AI Backend: Auto-executing {cmd_str}")
                self.execute_command(cmd_type, params)
            else:
                # Set pending command and prompt for confirmation
                self.pending_command = {
                    'type': cmd_type,
                    'params': params,
                    'cmd_str': cmd_str
                }
                print(f">>> Execute command: {cmd_str}? Type 'y' to confirm, 'n' to cancel <<<")

    def format_command(self, cmd_type: str, params: Dict[str, Any]) -> str:
        """Format command for display"""
        if cmd_type == "ARM":
            return "ARM"
        elif cmd_type == "DISARM":
            return "DISARM"
        elif cmd_type == "TAKEOFF":
            alt = params.get('altitude', 10)
            return f"TAKEOFF to {alt}m"
        elif cmd_type == "LAND":
            return "LAND"
        elif cmd_type == "RTL":
            return "RTL (Return to Launch)"
        elif cmd_type == "GOTO":
            lat = params.get('latitude')
            lon = params.get('longitude')
            alt = params.get('altitude', 'current')
            return f"GOTO ({lat}, {lon}, {alt}m)"
        elif cmd_type == "CHANGE_MODE":
            mode = params.get('mode')
            return f"CHANGE_MODE to {mode}"
        elif cmd_type == "MOVE_DIRECTION":
            direction = params.get('direction', 'unknown')
            distance = params.get('distance', 0)
            return f"MOVE {direction} {distance}m"
        else:
            return f"{cmd_type} {params}"

    def is_low_risk_command(self, cmd_type: str) -> bool:
        """Check if command is low risk (safe to auto-execute)"""
        low_risk = ['GET_PARAM']
        return cmd_type in low_risk

    def execute_command(self, cmd_type: str, params: Dict[str, Any]):
        """Execute a command via MAVProxy"""
        try:
            if cmd_type == "ARM":
                self.mpstate.functions.process_stdin("arm throttle")

            elif cmd_type == "DISARM":
                self.mpstate.functions.process_stdin("disarm")

            elif cmd_type == "TAKEOFF":
                altitude = params.get('altitude', 10)
                # Note: MAVProxy doesn't have a direct takeoff command
                # We need to use mode guided and then send takeoff via MAVLink
                self.mpstate.functions.process_stdin("mode GUIDED")
                time.sleep(0.5)
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, altitude
                )
                print(f"AI Backend: Sent TAKEOFF to {altitude}m")

            elif cmd_type == "LAND":
                self.mpstate.functions.process_stdin("mode LAND")

            elif cmd_type == "RTL":
                self.mpstate.functions.process_stdin("mode RTL")

            elif cmd_type == "CHANGE_MODE":
                mode = params.get('mode')
                self.mpstate.functions.process_stdin(f"mode {mode}")

            elif cmd_type == "GOTO":
                lat = params.get('latitude')
                lon = params.get('longitude')
                alt = params.get('altitude', 0)
                # Send MAVLink GOTO command
                self.master.mav.mission_item_send(
                    self.target_system,
                    self.target_component,
                    0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    2, 0, 0, 0, 0, 0,
                    lat, lon, alt
                )
                print(f"AI Backend: Sent GOTO ({lat}, {lon}, {alt}m)")

            elif cmd_type == "MOVE_DIRECTION":
                # Movement commands require GUIDED mode and velocity commands
                direction = params.get('direction', '').lower()
                distance = params.get('distance', 5)
                print(f"AI Backend: MOVE_DIRECTION not fully implemented yet")
                print(f"AI Backend: Would move {direction} by {distance}m")

            else:
                print(f"AI Backend: Command {cmd_type} not yet implemented")

        except Exception as e:
            print(f"AI Backend: Execution error - {str(e)}")

    def idle_task(self):
        """Called periodically by MAVProxy"""
        # Re-enable input handler if needed
        if hasattr(self, '_reenable_handler') and self._reenable_handler:
            self._reenable_handler = False
            if self.ai_settings.enabled:
                self.mpstate.functions.input_handler = self.handle_input

        # Periodic health check
        if self.ai_settings.enabled:
            now = time.time()
            if now - self.last_health_check > self.health_check_interval:
                self.last_health_check = now
                self.check_backend_health()

    def unload(self):
        """Called when module is unloaded"""
        # Clean up input handler
        if self.mpstate.functions.input_handler == self.handle_input:
            self.mpstate.functions.input_handler = None


def init(mpstate):
    """Initialize module"""
    return AIBackendModule(mpstate)
