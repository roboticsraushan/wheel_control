#!/usr/bin/env python3
"""Simple test: listen with the VoiceProcessor, send utterance to LocalLLMClient (Ollama), print result.

Usage:
  python3 tools/voice_ollama_test.py --device 4 --wake-word nova --model llama-3.1-8b

The script will:
 - listen once (or fallback to stdin)
 - optionally require a wake-word to be present in the utterance
 - call the Ollama-backed LocalLLMClient to parse intent
 - print the returned intent JSON

Notes:
 - Requires Ollama CLI and the chosen model installed locally (LocalLLMClient will raise if missing).
 - Uses the project's VoiceProcessor and LocalLLMClient implementations.
"""

import argparse
import sys
import json

import speech_recognition as sr

from voice_llm_navigator.voice_processor import VoiceProcessor
from voice_llm_navigator.local_llm_client import LocalLLMClient


def main():
    p = argparse.ArgumentParser(description="Voice -> Ollama test (single-shot)")
    p.add_argument("--device", type=int, default=None, help="Microphone device index (see sr.Microphone.list_microphone_names())")
    p.add_argument("--mic", action="store_true", help="Use microphone capture instead of stdin (default: stdin)")
    p.add_argument("--wake-word", type=str, default=None, help="Optional wake word (substring match)")
    p.add_argument("--model", type=str, default=None, help="Ollama model name (e.g. llama-3.1-8b or ollama:llama-3.1-8b)")
    p.add_argument("--timeout", type=float, default=6.0, help="listen timeout in seconds")
    p.add_argument("--list-mics", action="store_true", help="List available microphone names (and exit)")
    p.add_argument("--stdin", action="store_true", help="Force stdin text input instead of attempting microphone capture")
    p.add_argument("--no-wake-fail", action="store_true", help="If set, don't abort when wake-word is missing (just warn)")
    args = p.parse_args()
    if args.list_mics:
        try:
            names = sr.Microphone.list_microphone_names()
            print("Available microphones:")
            for i, n in enumerate(names):
                print(f"  {i}: {n}")
        except Exception as e:
            print("Failed to list microphones:", e)
        return 0

    # Default to text (stdin) to avoid audio issues during debugging.
    # Use --mic to force microphone capture.
    if args.stdin or not args.mic:
        try:
            text = input("Type your command: ").strip()
        except EOFError:
            text = ""
    else:
        vp = VoiceProcessor(microphone_index=args.device, wake_word=args.wake_word)
        print(f"Listening (timeout={args.timeout}s). Say your command or type if fallback...")
        text = vp.listen_once(timeout=args.timeout)
    if not text:
        print("No speech detected or recognition failed (empty). Exiting.")
        return 2

    print("Recognized utterance:", text)

    if args.wake_word:
        # If we instantiated a VoiceProcessor (mic path) use its helper; otherwise do a simple substring check
        if 'vp' in locals():
            has_wake = vp.detect_wakeword(text)
        else:
            has_wake = args.wake_word.lower() in text.lower()

        print(f"Wake word ('{args.wake_word}') present: {has_wake}")
        if not has_wake and not args.no_wake_fail:
            print("Wake word not detected; aborting. Use --no-wake-fail to continue anyway.")
            return 3

    # Prepare local LLM client (this will raise if Ollama CLI not found)
    try:
        client = LocalLLMClient(system_prompt=None, model=args.model)
    except Exception as e:
        print("LocalLLMClient initialization failed (is Ollama installed and model pulled?).")
        print("Error:", e)
        return 4

    # No detected objects available in this simple test; pass empty list
    detected_objects = []
    parsed = client.parse_command(text, detected_objects)

    print("Parsed intent (dict):")
    print(json.dumps(parsed, indent=2))

    return 0


if __name__ == '__main__':
    sys.exit(main())
