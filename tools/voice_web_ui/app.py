#!/usr/bin/env python3
"""Small Flask server for a lightweight voice web UI.

Serves a single-page UI that can:
- Record microphone audio (MediaRecorder)
- Upload audio to /api/transcribe -> returns transcription (uses faster_whisper if available)
- Send text to /api/chat -> queries local Ollama (CLI or HTTP) then synthesizes reply with Piper CLI
- /api/speak -> synthesize given text with Piper CLI and return audio file

Place this file at tools/voice_web_ui/app.py and the frontend assets in templates/ and static/.

Dependencies: Flask, (optional) faster-whisper, requests, ffmpeg (binary), piper CLI, ollama CLI/daemon
"""

import os
import subprocess
import tempfile
import uuid
import shlex
from flask import Flask, render_template, request, jsonify, send_file
from werkzeug.utils import safe_join

app = Flask(__name__, template_folder='templates', static_folder='static')

# Temporary directory for generated files
TMPDIR = os.path.join(tempfile.gettempdir(), 'voice_web_ui')
os.makedirs(TMPDIR, exist_ok=True)

# Configurable defaults (match voice_io_tuner defaults)
DEFAULT_PIPER_MODEL = os.path.expanduser('~/.local/share/piper/voices/en_US-lessac-medium.onnx')
DEFAULT_PIPER_CONFIG = os.path.expanduser('~/.local/share/piper/voices/en_US-lessac-medium.onnx.json')

# Lazy-loaded faster-whisper model holder
_fw_model = None


def _convert_to_wav(in_path, out_path):
    """Convert input audio to WAV using ffmpeg. Returns True on success."""
    try:
        subprocess.run(['ffmpeg', '-y', '-i', in_path, out_path], check=True, capture_output=True)
        return True
    except Exception as e:
        print('[webui] ffmpeg convert failed:', e)
        return False


def _load_faster_whisper(model_name='tiny'):
    global _fw_model
    if _fw_model is not None:
        return _fw_model
    try:
        from faster_whisper import WhisperModel
        # prefer cpu to be conservative; user can modify
        _fw_model = WhisperModel(model_name, device='cpu')
        return _fw_model
    except Exception as e:
        print('[webui] faster_whisper not available:', e)
        return None


def _transcribe_file(wav_path, model_name='tiny'):
    m = _load_faster_whisper(model_name)
    if m is None:
        return None, 'faster_whisper unavailable'
    try:
        segments, info = m.transcribe(wav_path)
        # concatenate segments
        text = ' '.join(s.text.strip() for s in segments if hasattr(s, 'text'))
        return text, None
    except Exception as e:
        return None, str(e)


def _synthesize_with_piper_cli(text, model_path=None, config_path=None):
    model_path = model_path or DEFAULT_PIPER_MODEL
    config_path = config_path or DEFAULT_PIPER_CONFIG
    out_name = f'tts-{uuid.uuid4().hex}.wav'
    out_path = os.path.join(TMPDIR, out_name)
    cmd = ['piper', '-m', model_path, '-c', config_path, '-f', out_path, '--text', text]
    try:
        print('[webui] Running Piper CLI:', ' '.join(shlex.quote(x) for x in cmd))
        subprocess.run(cmd, check=True, capture_output=True, timeout=60)
        print(f'[webui] Piper produced: {out_path}')
        return out_path
    except Exception as e:
        print('[webui] Piper CLI failed:', e)
        return None


def _get_reply_from_ollama(prompt, model='llama2', timeout=60):
    print(f'[webui] Querying Ollama with model={model}, prompt={prompt[:50]}...')
    
    # Try CLI first
    try:
        from shutil import which
        if which('ollama'):
            # try `ollama run MODEL PROMPT` (positional)
            try:
                print('[webui] Trying ollama CLI (positional)...')
                proc = subprocess.run(['ollama', 'run', model, prompt], capture_output=True, text=True, timeout=timeout)
                if proc.returncode == 0 and proc.stdout.strip():
                    reply = proc.stdout.strip()
                    print(f'[webui] Ollama CLI returned: {reply[:100]}...')
                    return reply
                else:
                    print(f'[webui] Ollama CLI failed with code {proc.returncode}, stderr: {proc.stderr[:200]}')
            except Exception as e:
                print(f'[webui] Ollama CLI positional failed: {e}')
            
            # try piping
            try:
                print('[webui] Trying ollama CLI (piped)...')
                proc2 = subprocess.run(['ollama', 'run', model], input=prompt, capture_output=True, text=True, timeout=timeout)
                if proc2.returncode == 0 and proc2.stdout.strip():
                    reply = proc2.stdout.strip()
                    print(f'[webui] Ollama CLI (piped) returned: {reply[:100]}...')
                    return reply
                else:
                    print(f'[webui] Ollama CLI piped failed with code {proc2.returncode}')
            except Exception as e:
                print(f'[webui] Ollama CLI piped failed: {e}')
    except Exception as e:
        print(f'[webui] Ollama CLI check failed: {e}')

    # HTTP fallback - handle streaming response
    try:
        import requests
        import json
        url = 'http://127.0.0.1:11434/api/generate'
        payload = {'model': model, 'prompt': prompt, 'stream': False}
        print(f'[webui] Trying Ollama HTTP API at {url}...')
        r = requests.post(url, json=payload, timeout=timeout)
        if r.ok:
            try:
                data = r.json()
                # Ollama API returns {'response': 'text here', 'done': true}
                if 'response' in data:
                    reply = data['response'].strip()
                    print(f'[webui] Ollama HTTP returned: {reply[:100]}...')
                    return reply
                print(f'[webui] Ollama HTTP unexpected format: {str(data)[:200]}')
            except Exception as e:
                print(f'[webui] Ollama HTTP parse failed: {e}, raw: {r.text[:200]}')
        else:
            print(f'[webui] Ollama HTTP request failed: {r.status_code} {r.text[:200]}')
    except Exception as e:
        print(f'[webui] Ollama HTTP fallback failed: {e}')

    print('[webui] All Ollama methods failed, returning None')
    return None


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/transcribe', methods=['POST'])
def api_transcribe():
    # Expect multipart form 'file'
    if 'file' not in request.files:
        return jsonify({'error': 'no file uploaded'}), 400
    f = request.files['file']
    fname = f.filename or f'upload-{uuid.uuid4().hex}'
    raw_path = os.path.join(TMPDIR, f'raw-{uuid.uuid4().hex}-{fname}')
    f.save(raw_path)

    # Ensure wav
    wav_path = os.path.join(TMPDIR, f'conv-{uuid.uuid4().hex}.wav')
    converted = _convert_to_wav(raw_path, wav_path)
    if not converted:
        # If input is already wav and ffmpeg failed, try to treat original as wav
        if raw_path.lower().endswith('.wav'):
            wav_path = raw_path
        else:
            return jsonify({'error': 'ffmpeg conversion failed and input is not WAV'}), 500

    text, err = _transcribe_file(wav_path)
    if err:
        return jsonify({'error': err}), 500
    return jsonify({'text': text})


@app.route('/api/speak', methods=['POST'])
def api_speak():
    data = request.get_json(force=True)
    if not data or 'text' not in data:
        return jsonify({'error': 'text required'}), 400
    text = data['text']
    out = _synthesize_with_piper_cli(text)
    if not out:
        return jsonify({'error': 'piper failed or not installed'}), 500
    return send_file(out, mimetype='audio/wav', as_attachment=False)


@app.route('/api/chat', methods=['POST'])
def api_chat():
    data = request.get_json(force=True)
    if not data or 'text' not in data:
        return jsonify({'error': 'text required'}), 400
    user_text = data['text']
    # Use llama3.1:8b as default since that's what's installed
    model = data.get('model', 'llama3.1:8b')
    print(f'[webui] /api/chat called with text: {user_text[:100]}')
    reply = _get_reply_from_ollama(user_text, model=model)
    if not reply:
        print('[webui] No reply from Ollama, using fallback echo')
        reply = f'I heard: {user_text}'
    # synthesize reply
    print(f'[webui] Synthesizing reply: {reply[:100]}...')
    out = _synthesize_with_piper_cli(reply)
    if not out:
        print('[webui] Piper synthesis failed, returning reply without audio')
        return jsonify({'reply': reply, 'audio': None})
    # return URL path to audio
    # we will provide /tts/<fname> endpoint
    fname = os.path.basename(out)
    # return absolute URL so browsers can reliably fetch it
    base = request.url_root.rstrip('/')
    audio_url = f"{base}/tts/{fname}"
    print(f'[webui] Chat reply ready, audio at: {audio_url}')
    return jsonify({'reply': reply, 'audio': audio_url})


@app.route('/tts/<path:filename>')
def tts_file(filename):
    # serve from TMPDIR
    safe_path = safe_join(TMPDIR, filename)
    if not safe_path or not os.path.exists(safe_path):
        return 'Not found', 404
    return send_file(safe_path, mimetype='audio/wav')


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=5003)
    p.add_argument('--debug', action='store_true')
    args = p.parse_args()
    print('Starting voice web UI on', args.host, args.port)
    app.run(host=args.host, port=args.port, debug=args.debug)
