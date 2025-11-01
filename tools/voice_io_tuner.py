#!/usr/bin/env python3
"""voice_io_tuner.py

Interactive utility to tune and test speech-to-text (STT) and text-to-speech (TTS)
on a development machine (laptop). It reuses `VoiceProcessor` from the
`voice_llm_navigator` package for microphone handling and provides a simple
TTS option using `pyttsx3` when available.

Features:
- Choose STT or TTS mode from a menu
- STT: list microphones, set parameters (ambient_duration, energy_threshold,
  pause_threshold, phrase_time_limit), run ambient calibration, and listen
- TTS: choose rate and volume, speak test phrases
- Loop mode: speak a prompt and listen for a spoken reply

This file is intended to be a single-file tool you can run from the repo root:
  python3 tools/voice_io_tuner.py

If dependencies are missing the script will explain how to install them.
"""

import argparse
import os
import sys
import time
import subprocess
import shlex


def add_src_to_path():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    src_pkg = os.path.join(repo_root, 'src', 'voice_llm_navigator')
    if os.path.isdir(src_pkg) and src_pkg not in sys.path:
        sys.path.insert(0, src_pkg)


def parse_args():
    p = argparse.ArgumentParser(description='Voice I/O tuner (STT + TTS)')
    p.add_argument('--mode', choices=['interactive', 'stt', 'tts', 'chat'], default='interactive', help='Start mode')
    p.add_argument('--mode-stt-backend', choices=['mic', 'stdin'], default='mic', help='STT backend')
    p.add_argument('--iterations', type=int, default=1, help='Number of listen iterations')
    p.add_argument('--text', default='Hello, this is a speaker test.', help='Sample text for TTS')
    p.add_argument('--piper-model', default=os.path.expanduser('~/.local/share/piper/voices/en_US-lessac-medium.onnx'), help='Path to Piper ONNX model file')
    p.add_argument('--piper-config', default=os.path.expanduser('~/.local/share/piper/voices/en_US-lessac-medium.onnx.json'), help='Path to Piper model config JSON')
    p.add_argument('--ollama-model', default='llama2', help='Local Ollama model name to use for replies (if ollama is installed)')
    return p.parse_args()


def try_imports():
    add_src_to_path()
    # Import VoiceProcessor from package
    try:
        from voice_llm_navigator.voice_processor import VoiceProcessor, HAS_SR
    except Exception as e:
        print('[tuner] Warning: failed to import VoiceProcessor from package:', e)
        VoiceProcessor = None
        HAS_SR = False
    # Import Piper TTS if available
    try:
        # piper-tts python package may be named 'piper_tts' or 'piper'
        try:
            import piper_tts as _piper
        except Exception:
            import piper as _piper
        PIPER_AVAILABLE = True
        piper_mod = _piper
    except Exception:
        piper_mod = None
        PIPER_AVAILABLE = False

    return VoiceProcessor, HAS_SR, piper_mod, PIPER_AVAILABLE


def list_microphones():
    try:
        import speech_recognition as sr
        names = sr.Microphone.list_microphone_names()
        for i, n in enumerate(names):
            print(f'{i}: {n}')
    except Exception as e:
        print('[tuner] Could not list microphones:', e)


def stt_flow(VoiceProcessor, HAS_SR, iterations=1, backend='mic'):
    if VoiceProcessor is None:
        print('[tuner] VoiceProcessor not available. Cannot run STT.')
        return

    print('\n--- STT (speech-to-text) mode ---')
    print('SpeechRecognition available:', HAS_SR)
    if HAS_SR:
        print('Available microphones:')
        list_microphones()

    # Get tuning params from user
    try:
        mic_idx = input('Microphone index (enter for default/None): ').strip()
        mic_idx = int(mic_idx) if mic_idx != '' else None
    except Exception:
        mic_idx = None

    try:
        ambient = float(input('Ambient duration (seconds) [1.0]: ') or '1.0')
    except Exception:
        ambient = 1.0
    try:
        energy = input('Energy threshold (int) [auto]: ').strip()
        energy = int(energy) if energy != '' else None
    except Exception:
        energy = None
    try:
        pause = float(input('Pause threshold (seconds) [0.6]: ') or '0.6')
    except Exception:
        pause = 0.6
    try:
        phrase_limit = input('Phrase time limit (seconds) [none]: ').strip()
        phrase_limit = float(phrase_limit) if phrase_limit != '' else None
    except Exception:
        phrase_limit = None

    print('\nStarting listen test. Press Ctrl+C to stop early.')
    vp = VoiceProcessor(microphone_index=mic_idx, force_stdin=(backend == 'stdin'),
                        ambient_duration=ambient, energy_threshold=energy,
                        pause_threshold=pause, phrase_time_limit=phrase_limit)

    for i in range(iterations):
        print(f'Listening iteration {i+1}/{iterations}...')
        text = vp.listen_once(timeout=10.0)
        print('Recognized:', repr(text))
        time.sleep(0.2)


def tts_flow(piper_mod, PIPER_AVAILABLE, sample_text=None, piper_model=None, piper_config=None):
    print('\n--- TTS (text-to-speech) mode (Piper only) ---')
    if not PIPER_AVAILABLE:
        print('Piper TTS not installed. Install with: pip install piper-tts')
        return

    sample = sample_text or input('Text to speak (press enter for default): ').strip() or 'Hello. This is a speaker test.'

    # Try high-level API first
    try:
        if hasattr(piper_mod, 'speak'):
            print('Speaking (Piper API):', sample)
            piper_mod.speak(sample, voice='en_US-lessac-medium')
            return
        elif hasattr(piper_mod, 'tts') and hasattr(piper_mod.tts, 'speak'):
            print('Speaking (Piper API):', sample)
            piper_mod.tts.speak(sample, voice='en_US-lessac-medium')
            return
    except Exception as e:
        print('[tuner] Piper API synth failed:', e)

    # Fallback to Piper CLI
    model_path = piper_model or os.path.expanduser('~/.local/share/piper/voices/en_US-lessac-medium.onnx')
    config_path = piper_config or os.path.expanduser('~/.local/share/piper/voices/en_US-lessac-medium.onnx.json')
    out = _synthesize_with_piper_cli(sample, model_path, config_path, out_path='/tmp/voiceio_tuner_tts.wav')
    if out:
        try:
            subprocess.run(['paplay', out], check=True)
        except Exception:
            try:
                subprocess.run(['aplay', out], check=True)
            except Exception as e:
                print('[tuner] Could not play TTS output automatically. File at:', out, 'Error:', e)


def _synthesize_with_piper_cli(text, model_path, config_path, out_path='/tmp/tts.wav'):
    """Synthesize using the piper CLI as a fallback when no high-level API is available."""
    model_path = os.path.expanduser(model_path)
    config_path = os.path.expanduser(config_path)
    cmd = [
        'piper', '-m', model_path, '-c', config_path, '-f', out_path, '--text', text
    ]
    try:
        print('[tuner] Running Piper CLI:', ' '.join(cmd))
        subprocess.run(cmd, check=True)
        return out_path
    except Exception as e:
        print('[tuner] Piper CLI failed:', e)
        return None


def _get_reply_from_ollama(prompt, model='llama2', timeout=30):
    """Query local Ollama for a reply. Try CLI first, then HTTP API fallback.

    Returns the reply string or None on failure.
    """
    # Try CLI if available
    try:
        from shutil import which
        if which('ollama'):
            # Try positional prompt first (newer ollama accepts PROMPT as positional arg)
            cmd1 = ['ollama', 'run', model, prompt]
            try:
                proc = subprocess.run(cmd1, capture_output=True, text=True, timeout=timeout)
                if proc.returncode == 0:
                    out = proc.stdout.strip()
                    if out:
                        return out
                else:
                    # Try piping the prompt to ollama run MODEL
                    proc2 = subprocess.run(['ollama', 'run', model], input=prompt, capture_output=True, text=True, timeout=timeout)
                    if proc2.returncode == 0:
                        out2 = proc2.stdout.strip()
                        if out2:
                            return out2
                    else:
                        stderr = (proc.stderr or '') + '\n' + (proc2.stderr or '')
                        print('[tuner] ollama CLI returned error:', stderr.strip())
            except Exception as e:
                print('[tuner] ollama CLI call failed:', e)
    except Exception:
        pass

    # HTTP fallback (if requests available and Ollama daemon running)
    try:
        import requests
        url = 'http://127.0.0.1:11434/api/generate'
        payload = {'model': model, 'prompt': prompt}
        r = requests.post(url, json=payload, timeout=timeout)
        if r.ok:
            try:
                data = r.json()
                # Try common fields
                for key in ('text', 'generated', 'content', 'result'):
                    if key in data:
                        return str(data[key])
                # Fallback: stringify
                return str(data)
            except Exception:
                return r.text
        else:
            print('[tuner] ollama HTTP request failed:', r.status_code, r.text[:200])
    except Exception as e:
        print('[tuner] ollama HTTP fallback failed:', e)

    return None


def chatbot_flow(VoiceProcessor, HAS_SR, piper_mod, PIPER_AVAILABLE, piper_model, piper_config, ollama_model='llama2'):
    """Run an interactive chatbot: listen, transcribe, generate reply, synthesize and play.

    This mode uses Piper only for synthesis (high-level API if available, otherwise Piper CLI).
    """
    print('\n--- Chatbot mode: speak and receive a spoken reply ---')
    print('Say "exit" at any time to quit the chatbot.')

    # Ensure Piper is available (we don't use pyttsx3)
    if not PIPER_AVAILABLE:
        print('[tuner] Piper TTS not available; install piper-tts to use chatbot mode.')
        return

    # Initialize VoiceProcessor for listening
    if VoiceProcessor is None:
        print('[tuner] VoiceProcessor not available; chatbot cannot run.')
        return

    vp = VoiceProcessor()

    while True:
        print('\nListening... (speak now)')
        text = vp.listen_once(timeout=15.0)
        if not text:
            print('[tuner] No speech detected; try again.')
            continue
        print('[tuner] Heard:', text)
        if text.strip().lower() in ('exit', 'quit', 'bye'):
            print('Exiting chatbot.')
            break

        # Generate a reply using Ollama (local LLM) if available, otherwise echo back
        reply = None
        try:
            reply = _get_reply_from_ollama(text.strip(), model=ollama_model)
        except Exception as e:
            print('[tuner] Ollama call failed:', e)

        if not reply:
            # fallback to simple echo
            reply = f'I heard: {text.strip()}'
        print('[tuner] Reply text:', reply)

        # Synthesize reply via Piper (API or CLI)
        out_wav = None
        try:
            if piper_mod and hasattr(piper_mod, 'speak'):
                print('[tuner] Synthesizing with piper.speak')
                piper_mod.speak(reply, voice='en_US-lessac-medium')
            elif piper_mod and hasattr(piper_mod, 'tts') and hasattr(piper_mod.tts, 'speak'):
                print('[tuner] Synthesizing with piper.tts.speak')
                piper_mod.tts.speak(reply, voice='en_US-lessac-medium')
            else:
                out_wav = _synthesize_with_piper_cli(reply, piper_model, piper_config, out_path='/tmp/voiceio_tuner_reply.wav')
        except Exception as e:
            print('[tuner] Piper synth failed:', e)
            out_wav = _synthesize_with_piper_cli(reply, piper_model, piper_config, out_path='/tmp/voiceio_tuner_reply.wav')

        # Play output if file was produced by CLI
        if out_wav:
            # try paplay (PulseAudio), then aplay (ALSA)
            try:
                subprocess.run(['paplay', out_wav], check=True)
            except Exception:
                try:
                    # Try default aplay
                    subprocess.run(['aplay', out_wav], check=True)
                except Exception as e:
                    print('[tuner] Failed to play synthesized audio:', e)



def interactive_menu(VoiceProcessor, HAS_SR, piper_mod, PIPER_AVAILABLE):
    while True:
        print('\nVoice I/O Tuner — choose an option:')
        print('  1) Test STT (microphone)')
        print('  2) Test STT (typing / stdin)')
        print('  3) Test TTS (speaker)')
        print('  4) Loop: speak prompt and listen for reply')
        print('  6) Chatbot: speak and receive a spoken reply (continuous)')
        print('  5) Exit')
        choice = input('Select (1-5): ').strip()
        if choice == '1':
            stt_flow(VoiceProcessor, HAS_SR, iterations=9999, backend='mic')
        elif choice == '2':
            stt_flow(VoiceProcessor, HAS_SR, iterations=9999, backend='stdin')
        elif choice == '3':
            # Piper-only TTS
            if not PIPER_AVAILABLE:
                print('Piper TTS not available; install piper-tts or check your PATH.')
                continue
            tts_flow(piper_mod, PIPER_AVAILABLE)
        elif choice == '4':
            # prompt then listen (Piper)
            if not PIPER_AVAILABLE:
                print('Piper TTS not available; install piper-tts to run loop mode.')
                continue
            prompt = input('Prompt to speak: ').strip() or 'What is your command?'
            # speak prompt using Piper
            try:
                if piper_mod and hasattr(piper_mod, 'speak'):
                    piper_mod.speak(prompt, voice='en_US-lessac-medium')
                elif piper_mod and hasattr(piper_mod, 'tts') and hasattr(piper_mod.tts, 'speak'):
                    piper_mod.tts.speak(prompt, voice='en_US-lessac-medium')
                else:
                    _synthesize_with_piper_cli(prompt, None, None, out_path='/tmp/voiceio_prompt.wav')
            except Exception as e:
                print('[tuner] Failed to synthesize prompt:', e)
            # listen once
            stt_flow(VoiceProcessor, HAS_SR, iterations=1, backend='mic')
        elif choice == '6':
            # Chatbot continuous loop
                args = parse_args()  # get defaults for model paths
                chatbot_flow(VoiceProcessor, HAS_SR, piper_mod, PIPER_AVAILABLE, args.piper_model, args.piper_config, args.ollama_model)
        elif choice == '5':
            print('Bye')
            break
        else:
            print('Invalid choice')


def main():
    args = parse_args()
    VoiceProcessor, HAS_SR, piper_mod, PIPER_AVAILABLE = try_imports()
    if args.mode == 'interactive':
        interactive_menu(VoiceProcessor, HAS_SR, piper_mod, PIPER_AVAILABLE)
    elif args.mode == 'stt':
        if args.mode_stt_backend == 'stdin':
            stt_flow(VoiceProcessor, HAS_SR, iterations=args.iterations, backend='stdin')
        else:
            stt_flow(VoiceProcessor, HAS_SR, iterations=args.iterations, backend='mic')
    elif args.mode == 'tts':
        tts_flow(piper_mod, PIPER_AVAILABLE, sample_text=args.text, piper_model=args.piper_model, piper_config=args.piper_config)
    elif args.mode == 'chat':
        chatbot_flow(VoiceProcessor, HAS_SR, piper_mod, PIPER_AVAILABLE, args.piper_model, args.piper_config, args.ollama_model)


if __name__ == '__main__':
    main()
