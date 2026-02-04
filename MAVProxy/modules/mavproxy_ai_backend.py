#!/usr/bin/env python3
'''
AI Backend Integration Module for MAVProxy
Enables natural language drone control via ArduPilot AI Backend

This module detects plain English commands, sends them to the AI backend
for processing, and executes the resulting commands after user confirmation.

Author: ArduPilot AI Backend Team
Date: 2026-02-05
Version: 1.0.0
'''

import os
import time
import json
import re
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
            ('verbose', bool, False),
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
            self.check_backend_health()
            self.say("AI Backend: Enabled")
            
        elif cmd == "disable":
            self.ai_settings.enabled = False
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
                
        elif cmd == "set":
            self.ai_settings.command(args[1:])
            
        else:
            print(self.usage())
    
    def show_status(self):
        """Display current status"""
        status = f"""AI Backend Status:
  Enabled:        {self.ai_settings.enabled}
  Backend URL:    {self.ai_settings.backend_url}
  Mode:           {self.ai_settings.mode}
  Backend Health: {'Available' if self.backend_available else 'Unavailable'}
  Auto-confirm:   {self.ai_settings.auto_confirm}
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
    
    def is_plain_english(self, text: str) -> bool:
        """
        Detect if a command is plain English (not a MAVProxy command)
        
        Simple heuristics:
        - Contains spaces and is longer than 3 words
        - Contains question words (what, how, when, where, why)
        - Contains common verbs (arm, takeoff, land, fly, move, go, etc.)
        - Has natural language structure (articles, prepositions)
        """
        # Clean and normalize
        text = text.strip().lower()
        words = text.split()
        
        # Too short
        if len(words) < 3:
            return False
        
        # Contains question words - definitely plain English
        question_words = ['what', 'how', 'when', 'where', 'why', 'which', 'who']
        if any(word in words for word in question_words):
            return True
        
        # Contains action phrases - check this BEFORE checking known commands
        # This catches "arm the drone" even though "arm" is a MAVProxy command
        action_phrases = [
            'arm the', 'takeoff to', 'land at', 'land the', 'fly to', 'move to',
            'go to', 'return to', 'change mode', 'set altitude',
            'get battery', 'show me', 'tell me', 'check the', 'please '
        ]
        if any(phrase in text for phrase in action_phrases):
            return True
        
        # Has sentence structure (contains articles, prepositions)
        structure_words = ['the', 'to', 'at', 'in', 'on', 'for', 'with', 'please']
        if any(word in words for word in structure_words) and len(words) >= 3:
            return True
        
        # Known MAVProxy commands to exclude (single word commands only)
        # Only reject if it's EXACTLY a known command with no natural language
        mavproxy_commands = [
            'param', 'wp', 'rally', 'fence', 'module', 'set', 'status', 
            'link', 'watch', 'graph', 'map', 'rc', 'servo', 'relay', 
            'camera', 'gimbal', 'battery'
        ]
        
        # If it starts with a known command but has no natural language markers, reject it
        if words[0] in mavproxy_commands and len(words) < 4:
            return False
        
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
            
            response = requests.post(url, json=payload, timeout=10)
            
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
        if not self.ai_settings.enabled:
            return
        
        if not self.backend_available:
            self.say("AI Backend: Backend not available. Check connection.")
            return
        
        # Send to backend
        self.say(f"AI Backend: Processing '{user_input}'...")
        response = self.send_to_backend(user_input)
        
        if not response:
            return
        
        # Display AI response
        ai_response = response.get('response', 'No response')
        self.say(f"AI: {ai_response}")
        
        # Check for command
        command = response.get('command')
        if command and command.get('type'):
            cmd_type = command['type']
            params = command.get('params', {})
            
            # Format command for display
            cmd_str = self.format_command(cmd_type, params)
            
            # Ask for confirmation (unless auto-confirm is enabled for low-risk)
            if self.ai_settings.auto_confirm and self.is_low_risk_command(cmd_type):
                self.say(f"AI Backend: Auto-executing {cmd_str}")
                self.execute_command(cmd_type, params)
            else:
                # Prompt for confirmation
                confirm = input(f"Execute command: {cmd_str}? [y/n]: ").strip().lower()
                if confirm == 'y' or confirm == 'yes':
                    self.execute_command(cmd_type, params)
                else:
                    self.say("AI Backend: Command cancelled")
    
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
                self.say(f"AI Backend: Sent TAKEOFF to {altitude}m")
                
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
                self.say(f"AI Backend: Sent GOTO ({lat}, {lon}, {alt}m)")
                
            else:
                self.say(f"AI Backend: Command {cmd_type} not yet implemented")
                
        except Exception as e:
            self.say(f"AI Backend: Execution error - {str(e)}")
    
    
    def unknown_command(self, args):
        """
        Called by MAVProxy when a command is not recognized.
        This is our hook to intercept plain English commands.
        """
        if not self.ai_settings.enabled:
            return False  # Let MAVProxy handle it
        
        # Reconstruct the full command from args
        if not args:
            return False
        
        command_text = ' '.join(args)
        
        # Check if it looks like plain English
        if self.is_plain_english(command_text):
            # Process through AI backend
            self.process_ai_command(command_text)
            return True  # We handled it
        
        return False  # Not plain English, let MAVProxy show error
    
    def idle_task(self):
        """Called periodically by MAVProxy"""
        # Periodic health check
        if self.ai_settings.enabled:
            now = time.time()
            if now - self.last_health_check > self.health_check_interval:
                self.last_health_check = now
                self.check_backend_health()
    
    def unload(self):
        """Called when module is unloaded"""
        pass


def init(mpstate):
    """Initialize module"""
    return AIBackendModule(mpstate)
