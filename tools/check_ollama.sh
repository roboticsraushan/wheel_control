#!/usr/bin/env bash
set -euo pipefail

echo "Checking ollama installation..."
if ! command -v ollama >/dev/null 2>&1; then
  echo "ERROR: ollama CLI not found in PATH"
  exit 2
fi

echo "ollama version:"
ollama -v || true

echo "Available models:"
ollama list || true

# Try a tiny run to verify model responds (non-blocking if model missing)
MODEL="${1:-}"
if [ -z "${MODEL}" ]; then
  echo "No model specified. Provide model name as first arg, e.g. 'llama-3.1-8b'"
  exit 0
fi

# Prompt handling: second arg is used as prompt; use '-' to read from stdin; fallback to default JSON
PROMPT_ARG="${2:-}"
DEFAULT_PROMPT='{"intent": "test"}'
if [ "${PROMPT_ARG}" = "-" ]; then
  echo "Reading prompt from stdin..."
  PROMPT=$(cat -)
  SOURCE="stdin"
elif [ -n "${PROMPT_ARG}" ]; then
  PROMPT="${PROMPT_ARG}"
  SOURCE="arg"
else
  PROMPT="${DEFAULT_PROMPT}"
  SOURCE="default"
fi

echo "Testing model '${MODEL}' with prompt (source=${SOURCE}):"
echo "${PROMPT}" | sed -n '1,3p'

set +e
# Try positional prompt first (newer ollama accepts PROMPT as positional arg)
echo "Attempting positional prompt..."
ollama run "${MODEL}" "${PROMPT}" 2>&1 | sed -n '1,5p'
rc=$?
if [ $rc -ne 0 ]; then
  echo "Positional prompt failed (exit $rc). Trying to pipe prompt to stdin..."
  echo "${PROMPT}" | ollama run "${MODEL}" 2>&1 | sed -n '1,5p'
  rc=$?
fi
set -e
if [ $rc -ne 0 ]; then
  echo "Model run failed (exit $rc). Ensure model '${MODEL}' is installed and compatible with your hardware."
  exit $rc
fi

echo "Model run appears successful (truncated output shown)."
exit 0
