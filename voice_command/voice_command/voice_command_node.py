#!/usr/bin/env python3
"""
voice_command_node.py
=====================
Listens for speech, and publishes a tool request only when the utterance
contains BOTH a valid command phrase AND a valid tool name — in that order.

Logic flow:
  1. Transcribe audio to text (Google STT, swappable via _transcribe).
  2. Normalize: lowercase + strip punctuation.
  3. Find the first matching command phrase (substring search).
  4. In the text AFTER the phrase, look for a whole-word tool match.
  5. If both found → publish tool_request. Otherwise → reject silently.

Published Topics
----------------
/voice_command/raw_text    (std_msgs/String)  every recognised utterance
/voice_command/tool_request (std_msgs/String)  "phrase=grab|tool=hammer|fiducial=1"

Parameters
----------
config_file      (string)  path to triggers.yaml  [default: package share]
language         (string)  STT language tag        [default: en-US]
energy_threshold (int)     mic sensitivity         [default: 300]
"""

import os
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import yaml
import speech_recognition as sr
from ament_index_python.packages import get_package_share_directory


class VoiceCommandNode(Node):

    def __init__(self):
        super().__init__('voice_command_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('config_file', '')
        self.declare_parameter('language', 'en-US')
        self.declare_parameter('energy_threshold', 300)
        # mic_device_index: PyAudio device index. -1 = look up by name.
        # mic_device_name: substring matched against PyAudio device names
        #   (case-insensitive, first hit wins). Default targets the USB
        #   webcam mic on the gripper rig — change if you swap webcams.
        self.declare_parameter('mic_device_index', -1)
        self.declare_parameter('mic_device_name', 'USB 2.0 Camera')

        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.language = self.get_parameter('language').get_parameter_value().string_value
        energy = self.get_parameter('energy_threshold').get_parameter_value().integer_value
        mic_index = self.get_parameter('mic_device_index').get_parameter_value().integer_value
        mic_name = self.get_parameter('mic_device_name').get_parameter_value().string_value

        if not config_path:
            share_dir = os.path.join(
                get_package_share_directory('voice_command'), 'config'
            )
            config_path = os.path.join(share_dir, 'triggers.yaml')

        # ── Load config ──────────────────────────────────────────────────────
        self.command_phrases: list[str] = []   # e.g. ["hand me", "grab"]
        self.tools: dict[str, int] = {}        # e.g. {"hammer": 1}
        self.home_phrases: list[str] = []      # e.g. ["go home"]
        self._load_config(config_path)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_raw     = self.create_publisher(String, 'voice_command/raw_text',     10)
        self.pub_request = self.create_publisher(String, 'voice_command/tool_request', 10)
        self.pub_home    = self.create_publisher(String, 'voice_command/home_request', 10)

        # ── Microphone setup ─────────────────────────────────────────────────
        self.recogniser = sr.Recognizer()
        self.recogniser.energy_threshold = energy

        resolved_index = self._resolve_mic_index(mic_index, mic_name)
        self.microphone = sr.Microphone(device_index=resolved_index)

        self.get_logger().info('Calibrating microphone — please wait...')
        with self.microphone as source:
            self.recogniser.adjust_for_ambient_noise(source, duration=1.5)
        self.get_logger().info(
            f'Listening in "{self.language}" '
            f'(energy_threshold={self.recogniser.energy_threshold:.0f})'
        )

        self._stop_listening = self.recogniser.listen_in_background(
            self.microphone,
            self._audio_callback,
            phrase_time_limit=5,
        )

    # ── Mic resolution ───────────────────────────────────────────────────────

    def _resolve_mic_index(self, explicit_index: int, name_substr: str):
        names = sr.Microphone.list_microphone_names()
        if explicit_index >= 0:
            if explicit_index >= len(names):
                self.get_logger().warn(
                    f'mic_device_index={explicit_index} out of range '
                    f'({len(names)} devices); falling back to default.'
                )
                return None
            self.get_logger().info(
                f'Mic pinned by index: [{explicit_index}] {names[explicit_index]}'
            )
            return explicit_index
        if name_substr:
            needle = name_substr.lower()
            for i, name in enumerate(names):
                if needle in name.lower():
                    self.get_logger().info(f'Mic pinned by name "{name_substr}": [{i}] {name}')
                    return i
            self.get_logger().warn(
                f'No mic name contains "{name_substr}"; falling back to default. '
                f'Available: {names}'
            )
        return None

    # ── Config ───────────────────────────────────────────────────────────────

    def _load_config(self, path: str) -> None:
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            self.command_phrases = [p.lower() for p in (data.get('command_phrases') or [])]
            self.tools = {
                str(k).lower(): int(v)
                for k, v in (data.get('tools') or {}).items()
            }
            self.home_phrases = [p.lower() for p in (data.get('home_phrases') or [])]

            self.get_logger().info(
                f'Loaded {len(self.command_phrases)} command phrase(s), '
                f'{len(self.tools)} tool(s), '
                f'{len(self.home_phrases)} home phrase(s) from: {path}'
            )
        except FileNotFoundError:
            self.get_logger().error(f'Config file not found: {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')

    # ── Audio callback (background thread) ───────────────────────────────────

    def _audio_callback(self, recogniser: sr.Recognizer, audio: sr.AudioData) -> None:
        text = self._transcribe(recogniser, audio)
        if text is None:
            return

        self.get_logger().info(f'Heard: "{text}"')
        self.pub_raw.publish(String(data=text))

        normalized = re.sub(r"[^\w\s]", "", text.lower())

        # Home request takes priority over tool request so "go home"
        # never gets mis-parsed as a tool name.
        for phrase in self.home_phrases:
            if phrase in normalized:
                payload = f'phrase={phrase}'
                self.pub_home.publish(String(data=payload))
                self.get_logger().info(f'Home request — {payload}')
                return

        result = self._parse_request(text)

        if result:
            phrase, tool, fiducial = result
            payload = f'phrase={phrase}|tool={tool}|fiducial={fiducial}'
            self.pub_request.publish(String(data=payload))
            self.get_logger().info(f'Tool request — {payload}')
        else:
            self.get_logger().debug(f'No valid command+tool pair in: "{text}"')

    # ── Parsing logic ─────────────────────────────────────────────────────────

    def _parse_request(self, text: str) -> tuple[str, str, int] | None:
        """
        Return (command_phrase, tool_name, fiducial_id) if the utterance
        contains a command phrase followed by a known tool word.
        Returns None if either is missing or in the wrong order.
        """
        # Normalize: lowercase and remove punctuation
        normalized = re.sub(r"[^\w\s]", "", text.lower())

        for phrase in self.command_phrases:
            idx = normalized.find(phrase)
            if idx == -1:
                continue

            # Only look at text that comes after the command phrase
            after = normalized[idx + len(phrase):]

            # Split into whole words to avoid partial matches (e.g. "hammering")
            words_after = after.split()

            for tool, fiducial in self.tools.items():
                if tool in words_after:
                    return phrase, tool, fiducial

        return None

    # ── Speech-to-text backend ────────────────────────────────────────────────

    def _transcribe(self, recogniser: sr.Recognizer, audio: sr.AudioData) -> str | None:
        """
        Google STT (free tier, no key needed).
        To swap backends (Whisper, Vosk, etc.) replace only this method body.
        """
        try:
            return recogniser.recognize_google(audio, language=self.language)
        except sr.UnknownValueError:
            return None  # unintelligible audio — normal, not an error
        except sr.RequestError as e:
            self.get_logger().warn(f'STT API error: {e}')
            return None

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        if hasattr(self, '_stop_listening'):
            self._stop_listening(wait_for_stop=False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
