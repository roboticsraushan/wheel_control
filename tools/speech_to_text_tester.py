#!/usr/bin/env python3
"""speech_to_text_tester.py

Small CLI utility to exercise the VoiceProcessor speech-to-text component
from the `voice_llm_navigator` package.

Usage examples:
  # typing mode (interactive)
  python3 tools/speech_to_text_tester.py --mode stdin

  # typing mode with piped input
  printf "go to the chair\n" | python3 tools/speech_to_text_tester.py --mode stdin

  # microphone mode (if SpeechRecognition and a mic are available)
  python3 tools/speech_to_text_tester.py --mode mic --timeout 6

  # file mode (wav file)
  python3 tools/speech_to_text_tester.py --mode file --file my_audio.wav

This script is intentionally lightweight so you can quickly tune ambient/energy
parameters and validate recognition quality.
"""

import argparse
import os
import sys
import time


def add_src_to_path():
    # Ensure we can import the package when run from the repository root
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    src_pkg = os.path.join(repo_root, 'src', 'voice_llm_navigator')
    if os.path.isdir(src_pkg) and src_pkg not in sys.path:
        sys.path.insert(0, src_pkg)


def parse_args():
    p = argparse.ArgumentParser(description='Speech-to-text tester for VoiceProcessor')
    p.add_argument('--mode', choices=['stdin', 'mic', 'file'], default='stdin', help='Input mode')
    p.add_argument('--file', help='WAV file for file mode')
    p.add_argument('--timeout', type=float, default=5.0, help='Listen timeout (seconds)')
    p.add_argument('--ambient-duration', type=float, default=1.0, help='Ambient noise duration')
    p.add_argument('--energy-threshold', type=int, default=None, help='Energy threshold (int)')
    p.add_argument('--pause-threshold', type=float, default=0.6, help='Pause threshold')
    p.add_argument('--phrase-time-limit', type=float, default=None, help='Phrase time limit (seconds)')
    p.add_argument('--microphone-index', type=int, default=None, help='Microphone device index')
    p.add_argument('--iterations', type=int, default=1, help='Number of listen iterations')
    p.add_argument('--verbose', action='store_true')
    return p.parse_args()


def mic_mode(args, VoiceProcessor):
    vp = VoiceProcessor(microphone_index=args.microphone_index, force_stdin=False,
                        ambient_duration=args.ambient_duration,
                        energy_threshold=args.energy_threshold,
                        pause_threshold=args.pause_threshold,
                        phrase_time_limit=args.phrase_time_limit)
    print('[tester] Using microphone mode. Press Ctrl+C to quit.')
    for i in range(args.iterations):
        print(f'[tester] Listening (iteration {i+1}/{args.iterations})...')
        text = vp.listen_once(timeout=args.timeout)
        print('[tester] Recognized ->', repr(text))
        time.sleep(0.2)


def stdin_mode(args, VoiceProcessor):
    vp = VoiceProcessor(force_stdin=True,
                        ambient_duration=args.ambient_duration,
                        energy_threshold=args.energy_threshold,
                        pause_threshold=args.pause_threshold,
                        phrase_time_limit=args.phrase_time_limit)
    print('[tester] Typing mode. Type a line and press enter (empty line to skip).')
    for i in range(args.iterations):
        print(f'[tester] Iteration {i+1}/{args.iterations}')
        text = vp.listen_once(timeout=args.timeout)
        print('[tester] Recognized ->', repr(text))


def file_mode(args):
    try:
        import speech_recognition as sr
    except Exception as e:
        print('[tester] speech_recognition not available:', e)
        return
    if not args.file:
        print('[tester] --file is required in file mode')
        return
    if not os.path.exists(args.file):
        print('[tester] File not found:', args.file)
        return

    r = sr.Recognizer()
    try:
        with sr.AudioFile(args.file) as source:
            audio = r.record(source)
        try:
            text = r.recognize_sphinx(audio)
            print('[tester] PocketSphinx ->', text)
        except Exception:
            try:
                text = r.recognize_google(audio)
                print('[tester] Google Web Speech ->', text)
            except Exception as e:
                print('[tester] Recognition failed:', e)
    except Exception as e:
        print('[tester] Failed to read audio file:', e)


def main():
    add_src_to_path()
    args = parse_args()

    try:
        # Import VoiceProcessor after fixing sys.path
        from voice_llm_navigator.voice_processor import VoiceProcessor, HAS_SR
    except Exception as e:
        print('[tester] Failed to import VoiceProcessor from package:', e)
        print('[tester] Make sure you run this from the repository root and the package source is at src/voice_llm_navigator')
        sys.exit(2)

    if args.verbose:
        print('[tester] HAS_SR =', HAS_SR)
        print('[tester] args =', args)

    if args.mode == 'stdin':
        stdin_mode(args, VoiceProcessor)
    elif args.mode == 'mic':
        if not HAS_SR:
            print('[tester] SpeechRecognition not available; mic mode will not work.')
            sys.exit(1)
        mic_mode(args, VoiceProcessor)
    elif args.mode == 'file':
        file_mode(args)


if __name__ == '__main__':
    main()
