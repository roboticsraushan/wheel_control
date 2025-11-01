#!/usr/bin/env bash
set -euo pipefail

# mic_test.sh
# Quick helper to list audio capture devices and record a short WAV file.
# Usage:
#   ./tools/mic_test.sh [DURATION_SECONDS] [OUT_PATH]
# Examples:
#   ./tools/mic_test.sh          # record 5s to /tmp/mic_test.wav
#   ./tools/mic_test.sh 3 /tmp/out.wav

DURATION=${1:-5}
OUT=${2:-/tmp/mic_test.wav}

echo "Mic test starting — duration=${DURATION}s, out=${OUT}"
echo
echo "--- Pulse/pipewire sources (pactl) ---"
if command -v pactl >/dev/null 2>&1; then
  pactl list short sources || true
else
  echo "pactl not found"
fi
echo
echo "--- ALSA capture devices (arecord -l) ---"
if command -v arecord >/dev/null 2>&1; then
  arecord -l || true
else
  echo "arecord not found"
fi
echo

echo "Recorder selection..."

if command -v arecord >/dev/null 2>&1; then
  echo "Using arecord (ALSA) — recording ${DURATION}s to ${OUT}"
  # Try standard CD format (44100Hz, stereo, 16-bit)
  if arecord -f cd -d "${DURATION}" "${OUT}"; then
    echo "Saved ${OUT}"
    exit 0
  else
    echo "arecord attempt failed, will try other backends" >&2
  fi
fi

if command -v ffmpeg >/dev/null 2>&1; then
  echo "Using ffmpeg..."
  # Prefer PulseAudio if available
  if command -v pactl >/dev/null 2>&1 && pactl info >/dev/null 2>&1; then
    echo "Recording via PulseAudio (default source) for ${DURATION}s"
    if ffmpeg -y -hide_banner -loglevel error -f pulse -i default -t "${DURATION}" "${OUT}"; then
      echo "Saved ${OUT}"
      exit 0
    else
      echo "ffmpeg (pulse) failed, trying alsa" >&2
    fi
  fi
  # Try ALSA default device
  if ffmpeg -y -hide_banner -loglevel error -f alsa -i default -t "${DURATION}" "${OUT}"; then
    echo "Saved ${OUT}"
    exit 0
  else
    echo "ffmpeg (alsa) failed" >&2
  fi
fi

echo "Falling back to Python sounddevice (if available)..."
if command -v python3 >/dev/null 2>&1; then
  python3 - <<PY
import sys
try:
    import sounddevice as sd
    from scipy.io.wavfile import write
except Exception as e:
    print('Missing Python deps:', e, file=sys.stderr)
    sys.exit(2)
fs = 44100
dur = ${DURATION}
print(f'Recording (python) for {dur}s...')
rec = sd.rec(int(dur * fs), samplerate=fs, channels=1)
sd.wait()
write('${OUT}', fs, rec)
print('Saved ${OUT}')
PY
  rc=$?
  if [ "$rc" = "0" ]; then
    exit 0
  fi
fi

echo "All recorder attempts failed. Install one of: alsa-utils (arecord), ffmpeg, or python packages: sounddevice+scipy" >&2
exit 1
