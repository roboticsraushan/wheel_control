"""voice_processor.py

Minimal voice processor wrapper used by the voice navigator.

Features (MVP):
- Uses SpeechRecognition (if installed) with PocketSphinx (offline) preferred,
  and Google Web Speech as a secondary recognizer.
- Falls back to stdin input when a microphone or SpeechRecognition is unavailable
  (useful for testing or headless setups).
- Optional simple wake-word support (substring match).

The class provides a blocking `listen_once()` method and a convenience
`detect_wakeword()` helper.
"""

from typing import Optional
import logging

try:
    import speech_recognition as sr
    HAS_SR = True
except Exception:
    HAS_SR = False
# Note: faster-whisper import is performed lazily inside the class initializer
WhisperModel = None
HAS_FASTER_WHISPER = False

LOG = logging.getLogger("voice_processor")


class VoiceProcessor:
    def __init__(self, microphone_index: Optional[int] = None, push_to_talk: bool = False, wake_word: Optional[str] = None, force_stdin: bool = False, ambient_duration: float = 1.0, energy_threshold: Optional[int] = None, pause_threshold: float = 0.6, phrase_time_limit: Optional[float] = None, whisper_model: str = 'base', whisper_device: str = 'cpu', whisper_beam_size: int = 5):
        """Create a VoiceProcessor.

        microphone_index: optional device index passed to SpeechRecognition.Microphone
        push_to_talk: reserved for future (MVP uses blocking listen_once())
        wake_word: optional lowercase substring to require before accepting a command
        """
        self.push_to_talk = push_to_talk
        self.microphone_index = microphone_index
        # If force_stdin is True, always use the stdin fallback even when
        # SpeechRecognition is available. This is useful for typing commands in
        # development or headless environments.
        self.force_stdin = force_stdin
        # tuning params for SpeechRecognition recognizer
        self.ambient_duration = float(ambient_duration) if ambient_duration is not None else 1.0
        self.energy_threshold = int(energy_threshold) if energy_threshold is not None else None
        self.pause_threshold = float(pause_threshold) if pause_threshold is not None else 0.6
        self.phrase_time_limit = float(phrase_time_limit) if phrase_time_limit is not None else None
        self.wake_word = wake_word.lower() if wake_word else None

        if HAS_SR:
            self.recognizer = sr.Recognizer()
            self.microphone = None
            try:
                self.microphone = sr.Microphone(device_index=microphone_index)
                LOG.debug("Microphone initialized (index=%s)", microphone_index)
            except Exception as e:
                LOG.warning("Microphone not available: %s", e)
                self.microphone = None
        else:
            LOG.info("SpeechRecognition not available; falling back to stdin input")
        # Initialize faster-whisper model if available and requested
        self.whisper_model_name = whisper_model
        self.whisper_device = whisper_device
        self.whisper_beam_size = whisper_beam_size
        self.whisper = None
        # Lazily import/initialize faster-whisper only if a model name is provided
        if self.whisper_model_name:
            try:
                from faster_whisper import WhisperModel as _WhisperModel
                try:
                    self.whisper = _WhisperModel(self.whisper_model_name, device=self.whisper_device)
                    LOG.debug("Initialized faster-whisper model: %s on %s", self.whisper_model_name, self.whisper_device)
                except Exception as e:
                    LOG.warning("Failed to initialize faster-whisper model: %s", e)
                    self.whisper = None
            except Exception as e:
                LOG.warning("faster-whisper not available: %s", e)
                self.whisper = None

    def listen_once(self, timeout: Optional[float] = None) -> Optional[str]:
        """Blocking listen call that returns recognized text (lowercased) or None.

        If a wake_word was configured, the returned text will be the full recognized
        phrase (caller can use detect_wakeword()).
        """
        # Use SpeechRecognition only when available, a microphone is present,
        # and the caller hasn't requested forcing stdin typing mode.
        if (not getattr(self, 'force_stdin', False)) and HAS_SR and getattr(self, 'microphone', None) is not None:
            with self.microphone as source:
                try:
                    # ambient adjustment: increase duration slightly for noisy environments
                    self.recognizer.adjust_for_ambient_noise(source, duration=self.ambient_duration)
                except Exception:
                    pass
                # tune recognizer thresholds to reduce spurious short results
                try:
                    if getattr(self, 'energy_threshold', None) is not None:
                        self.recognizer.energy_threshold = self.energy_threshold
                    # pause_threshold helps grouping words into phrases
                    self.recognizer.pause_threshold = self.pause_threshold
                except Exception:
                    pass
                try:
                    # phrase_time_limit caps how long we listen for a single utterance
                    listen_kwargs = {}
                    if timeout is not None:
                        listen_kwargs['timeout'] = timeout
                    if getattr(self, 'phrase_time_limit', None) is not None:
                        listen_kwargs['phrase_time_limit'] = self.phrase_time_limit
                    audio = self.recognizer.listen(source, **listen_kwargs)
                except Exception as e:
                    LOG.debug("listen failed: %s", e)
                    return None

            # Use faster-whisper (local) when available. Otherwise fall back to
            # Google's Web Speech API.
            try:
                if self.whisper is not None:
                    # write audio bytes to a temporary WAV file and transcribe
                    import tempfile
                    import os
                    wav_bytes = audio.get_wav_data()
                    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tf:
                        tf.write(wav_bytes)
                        tmp_path = tf.name
                    try:
                        segments, info = self.whisper.transcribe(tmp_path, beam_size=self.whisper_beam_size)
                        # join segment texts
                        text = ' '.join([seg.text for seg in segments])
                        try:
                            os.unlink(tmp_path)
                        except Exception:
                            pass
                        return text.lower() if text else None
                    except Exception as e:
                        LOG.debug('faster-whisper transcribe failed: %s', e)
                        try:
                            os.unlink(tmp_path)
                        except Exception:
                            pass

                # fallback: use Google Web Speech via SpeechRecognition
                try:
                    text = self.recognizer.recognize_google(audio)
                    return text.lower()
                except Exception as e:
                    LOG.debug("recognition failed: %s", e)
                    return None
            except Exception as e:
                LOG.debug('recognition pipeline error: %s', e)
                return None

        # Fallback: stdin typing (useful in dev / headless)
        try:
            print('[voice_processor] SpeechRecognition not available or microphone missing.')
            print('[voice_processor] Type the simulated spoken command (or empty to skip):')
            text = input('> ').strip()
            return text.lower() if text else None
        except Exception as e:
            LOG.debug("stdin fallback failed: %s", e)
            return None

    def detect_wakeword(self, text: Optional[str]) -> bool:
        """Return True if the configured wake_word appears in text.

        If no wake_word is configured this returns True when text is non-empty.
        """
        if not text:
            return False
        if not self.wake_word:
            return True
        return self.wake_word in text.lower()


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    vp = VoiceProcessor(wake_word='nova')
    print('Say something (or type) — wake word set to "nova"')
    text = vp.listen_once()
    print('Recognized:', text)
    print('Wake word present:', vp.detect_wakeword(text))
