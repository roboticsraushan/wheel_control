"""local_llm_client.py

LocalLLMClient with optional Ollama integration.

Behavior:
- If Ollama CLI is available, send a brief system+user prompt asking the model to
  return a small JSON describing the intent (e.g. {"intent":"go_to","target_label":"chair"}).
- If Ollama is not available or the call fails, fall back to the previous rule-based parser.

This design avoids hard runtime dependencies on heavy libraries; it relies on the
`ollama` CLI being installed and accessible in PATH when used locally.
"""
from typing import List, Dict, Optional
import json
import shutil
import subprocess
import shlex


class LocalLLMClient:
    def __init__(self, system_prompt: Optional[str] = None, model: Optional[str] = None, timeout: float = 5.0):
        self.system_prompt = system_prompt or 'Parse navigation commands and return intent and target as JSON.'
        # model string expected in Ollama format, e.g. 'llama-3.1-8b' or 'ollama:llama-3.1-8b'
        self.model = model or 'llama-3.1-8b'
        self.timeout = timeout
        # detect ollama CLI
        self.has_ollama = shutil.which('ollama') is not None
        if not self.has_ollama:
            # Do not raise here — allow fallback to rule-based parser so the node
            # can run in environments without Ollama (useful for testing/dev).
            # If Ollama is desired, user should install it and set `model` in
            # `config/llm_config.yaml` as documented in the README.
            try:
                import warnings
                warnings.warn(
                    "Ollama CLI not found in PATH; falling back to heuristic parser.\n"
                    "Install Ollama and the desired model to enable LLM parsing.",
                    UserWarning,
                )
            except Exception:
                # Keep silent if warnings import fails for some reason
                pass

    def _make_prompt(self, text: str, detected_objects: List[Dict]) -> str:
        labels = [str(o.get('label')) for o in detected_objects if o.get('label')]
        objs_line = ', '.join(labels) if labels else 'none'
        prompt = (
            f"System: {self.system_prompt}\n"
            f"Detected objects: {objs_line}\n"
            f"User utterance: \"{text}\"\n"
            "Return a single valid JSON object only (no surrounding text) with one of the following shapes:\n"
            "- {\"intent\": \"go_to\", \"target_label\": \"chair\"}\n"
            "- {\"intent\": \"list_objects\", \"objects\": [\"chair\", \"table\"]}\n"
            "- {\"intent\": \"record_scene\"}\n"
            "- {\"intent\": \"stop\"}\n"
            "- {\"intent\": \"unknown\", \"raw\": \"...\"}\n"
            "Make sure target_label (if present) exactly matches one of the detected object labels.\n"
            "Use 'record_scene' intent for commands about recording or saving what the robot sees."
        )
        return prompt

    def _call_ollama(self, prompt: str) -> Optional[str]:
        """Call the ollama CLI with the prompt and return stdout text, or None on failure."""
        if not self.has_ollama:
            return None
        # Build command: ollama run <model> --prompt "..."
        # First attempt: legacy approach passing prompt as a flag (works on some CLI versions)
        cmd_flag = ['ollama', 'run', self.model, '--prompt', prompt]
        try:
            proc = subprocess.run(cmd_flag, capture_output=True, text=True, timeout=self.timeout)
        except Exception:
            proc = None

        # If the flag approach failed due to unknown flag or returned non-zero, retry by piping prompt to stdin
        if proc is None or proc.returncode != 0:
            stderr = (proc.stderr or '') if proc is not None else ''
            if 'unknown flag' in stderr.lower() or '--prompt' in stderr.lower() or proc is None or proc.returncode != 0:
                # Retry by sending the prompt on stdin (most compatible)
                try:
                    proc2 = subprocess.run(['ollama', 'run', self.model], input=prompt, capture_output=True, text=True, timeout=self.timeout)
                    if proc2.returncode != 0:
                        stderr2 = (proc2.stderr or '').strip()
                        return f"""__OLLAMA_ERROR__\nreturncode: {proc2.returncode}\nstderr: {stderr2}"""
                    return proc2.stdout.strip()
                except Exception:
                    return None
            else:
                # Some other error - return stderr for debugging
                return f"""__OLLAMA_ERROR__\nreturncode: {proc.returncode}\nstderr: {stderr.strip()}"""

        # Success on first attempt
        return proc.stdout.strip()

    def parse_command(self, text: str, detected_objects: List[Dict]) -> Dict:
        """Parse user utterance into an intent dict.

        Tries Ollama first; if unavailable or output can't be parsed, falls back to the
        heuristic parser.
        """
        t = (text or '').strip()
        if not t:
            return {'intent': 'none', 'reason': 'empty', 'raw': text}

        # quick heuristics for stop/cancel
        low = t.lower()
        if 'stop' in low or 'cancel' in low or 'abort' in low:
            return {'intent': 'stop', 'raw': text}
        
        # check for record scene commands
        if any(phrase in low for phrase in ['record', 'save', 'remember']) and \
           any(phrase in low for phrase in ['items', 'objects', 'things', 'see', 'here', 'this']):
            return {'intent': 'record_scene', 'raw': text}

        # try Ollama if available
        if self.has_ollama:
            prompt = self._make_prompt(text, detected_objects)
            out = self._call_ollama(prompt)
            # Note: treat any non-None output as useful (could be empty string)
            if out is not None:
                # try to extract JSON from the model output
                try:
                    # Some models may print extra whitespace; try to find first { ... }
                    start = out.find('{')
                    end = out.rfind('}')
                    if start != -1 and end != -1 and end > start:
                        json_text = out[start:end+1]
                        parsed = json.loads(json_text)
                        # basic validation: ensure intent present
                        if 'intent' in parsed:
                            return parsed
                except Exception:
                    pass
                # If we couldn't parse JSON, return the raw model text so caller can inspect it
                return {'intent': 'llm_text', 'text': out, 'raw': text}

        # fallback heuristic parser (same logic as original MVP)
        # ask what you see
        if 'what' in low and 'see' in low:
            labels = [o.get('label') for o in detected_objects if o.get('label')]
            return {'intent': 'list_objects', 'objects': labels, 'raw': text}

        # try to match a label substring
        labels = [o.get('label', '').lower() for o in detected_objects if o.get('label')]
        for lbl in labels:
            if lbl and lbl in low:
                return {'intent': 'go_to', 'target_label': lbl, 'raw': text}

        return {'intent': 'unknown', 'raw': text}

