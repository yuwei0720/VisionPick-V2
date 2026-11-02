from __future__ import annotations

import logging
import tempfile
import wave
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .gemini_live_transcriber import LiveTranscriptionError, transcribe_microphone_live
from .gemini_transcriber import transcribe_audio_file

logger = logging.getLogger(__name__)


def capture_and_transcribe(
    cfg: dict[str, Any],
    *,
    record_seconds: float | None = None,
    backend: str | None = None,
    record_fn: Callable[..., Path] | None = None,
    transcribe_fn: Callable[..., str] | None = None,
    live_transcribe_fn: Callable[..., str] | None = None,
    on_interim: Callable[[str], None] | None = None,
    on_fallback_retry: Callable[[float], None] | None = None,
) -> str:
    """Capture one voice command and return text for the existing pipeline.

    ``gemini_live`` is the default backend. It streams microphone PCM directly
    to ``gemini-3.5-transcribe-live``. If configured, failures fall back to the
    existing file-based ``gemini-3.5-transcribe`` backend. Stage 0 only returns
    text; it never parses semantics, starts perception, or authorizes motion.
    """

    voice_cfg = dict(cfg.get("voice", {}) or {})
    duration = float(
        record_seconds
        if record_seconds is not None
        else voice_cfg.get("record_seconds", 5.0)
    )
    if duration <= 0:
        raise ValueError("voice.record_seconds 必須大於 0")

    sample_rate = int(voice_cfg.get("sample_rate_hz", 16000))
    channels = int(voice_cfg.get("channels", 1))
    if sample_rate <= 0:
        raise ValueError("voice.sample_rate_hz 必須大於 0")
    if channels != 1:
        raise ValueError("Voice MVP 目前只支援單聲道 channels=1")

    selected_backend = str(
        backend if backend is not None else voice_cfg.get("backend", "gemini_live")
    ).strip().lower()
    fallback_backend = str(
        voice_cfg.get("fallback_backend", "none") or "none"
    ).strip().lower()

    if selected_backend not in {"gemini_live", "gemini_file"}:
        raise ValueError("voice.backend 必須是 gemini_live 或 gemini_file")
    if fallback_backend not in {"gemini_file", "none"}:
        raise ValueError("voice.fallback_backend 必須是 gemini_file 或 none")

    vocabulary = build_custom_vocabulary(cfg)
    language_codes = voice_cfg.get("language_codes", [])

    if selected_backend == "gemini_file":
        transcript = _capture_file_backend(
            duration=duration,
            sample_rate=sample_rate,
            channels=channels,
            language_codes=language_codes,
            vocabulary=vocabulary,
            voice_cfg=voice_cfg,
            record_fn=record_fn,
            transcribe_fn=transcribe_fn,
        )
        return _validate_transcript(transcript)

    def _handle_interim(text: str) -> None:
        logger.debug("VOICE INTERIM: %s", text)
        if on_interim is not None:
            try:
                on_interim(text)
            except Exception:
                pass

    live_fn = live_transcribe_fn or transcribe_microphone_live
    try:
        transcript = live_fn(
            duration_seconds=duration,
            sample_rate_hz=sample_rate,
            channels=channels,
            chunk_ms=int(voice_cfg.get("live_chunk_ms", 100)),
            model=str(voice_cfg.get("live_model", "gemini-3.5-transcribe-live")),
            language_codes=language_codes,
            custom_vocabulary=vocabulary,
            mode=str(voice_cfg.get("live_mode", "VERBATIM")),
            final_timeout_seconds=float(
                voice_cfg.get("live_final_timeout_s", 6.0)
            ),
            on_interim=_handle_interim,
        )
        return _validate_transcript(transcript)
    except Exception as exc:
        if fallback_backend != "gemini_file":
            raise

        logger.warning(
            "VOICE: gemini_live failed (%s); falling back to gemini_file.",
            exc,
        )

        # If Live captured PCM before failing, reuse exactly the same utterance
        # so the operator does not have to repeat a robot command.
        if isinstance(exc, LiveTranscriptionError) and exc.pcm_bytes:
            transcript = _transcribe_pcm_with_file_backend(
                exc.pcm_bytes,
                sample_rate=sample_rate,
                channels=channels,
                language_codes=language_codes,
                vocabulary=vocabulary,
                voice_cfg=voice_cfg,
                transcribe_fn=transcribe_fn,
            )
        else:
            logger.warning(
                "VOICE: Live failed before reusable audio was captured; "
                "fallback will record the command again for %.1f seconds.",
                duration,
            )
            if on_fallback_retry is not None:
                try:
                    on_fallback_retry(duration)
                except Exception:
                    pass
            transcript = _capture_file_backend(
                duration=duration,
                sample_rate=sample_rate,
                channels=channels,
                language_codes=language_codes,
                vocabulary=vocabulary,
                voice_cfg=voice_cfg,
                record_fn=record_fn,
                transcribe_fn=transcribe_fn,
            )
        return _validate_transcript(transcript)


def _capture_file_backend(
    *,
    duration: float,
    sample_rate: int,
    channels: int,
    language_codes: Any,
    vocabulary: list[str],
    voice_cfg: dict[str, Any],
    record_fn: Callable[..., Path] | None,
    transcribe_fn: Callable[..., str] | None,
) -> str:
    recorder = record_fn or record_microphone_wav
    transcriber = transcribe_fn or transcribe_audio_file

    with tempfile.TemporaryDirectory(prefix="edgeai_voice_") as tmpdir:
        wav_path = Path(tmpdir) / "command.wav"
        recorder(
            wav_path,
            duration_seconds=duration,
            sample_rate_hz=sample_rate,
            channels=channels,
        )
        return transcriber(
            wav_path,
            model=str(voice_cfg.get("file_model", voice_cfg.get("model", "gemini-3.5-transcribe"))),
            language_codes=language_codes,
            custom_vocabulary=vocabulary,
        )


def _transcribe_pcm_with_file_backend(
    pcm_bytes: bytes,
    *,
    sample_rate: int,
    channels: int,
    language_codes: Any,
    vocabulary: list[str],
    voice_cfg: dict[str, Any],
    transcribe_fn: Callable[..., str] | None,
) -> str:
    transcriber = transcribe_fn or transcribe_audio_file
    with tempfile.TemporaryDirectory(prefix="edgeai_voice_fallback_") as tmpdir:
        wav_path = Path(tmpdir) / "command_fallback.wav"
        write_pcm_wav(
            wav_path,
            pcm_bytes,
            sample_rate_hz=sample_rate,
            channels=channels,
        )
        return transcriber(
            wav_path,
            model=str(voice_cfg.get("file_model", voice_cfg.get("model", "gemini-3.5-transcribe"))),
            language_codes=language_codes,
            custom_vocabulary=vocabulary,
        )


def _validate_transcript(transcript: Any) -> str:
    text = str(transcript or "").strip()
    if not text:
        raise RuntimeError("語音轉錄結果為空，停止進入 Semantic Pipeline")
    return text


def record_microphone_wav(
    path: str | Path,
    *,
    duration_seconds: float,
    sample_rate_hz: int = 16000,
    channels: int = 1,
) -> Path:
    """Capture a fixed-duration 16-bit PCM WAV from the default microphone."""

    try:
        import sounddevice as sd
    except ImportError as exc:
        raise RuntimeError(
            "Voice MVP 需要 sounddevice；請先安裝 requirements.txt"
        ) from exc

    frames = int(round(float(duration_seconds) * int(sample_rate_hz)))
    if frames <= 0:
        raise ValueError("錄音 frame 數必須大於 0")

    recording = sd.rec(
        frames,
        samplerate=int(sample_rate_hz),
        channels=int(channels),
        dtype="int16",
    )
    sd.wait()

    pcm = np.asarray(recording, dtype=np.int16)
    if pcm.ndim == 1:
        pcm = pcm[:, None]

    return write_pcm_wav(
        path,
        pcm.tobytes(order="C"),
        sample_rate_hz=sample_rate_hz,
        channels=channels,
    )


def write_pcm_wav(
    path: str | Path,
    pcm_bytes: bytes,
    *,
    sample_rate_hz: int,
    channels: int,
) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(output), "wb") as wav:
        wav.setnchannels(int(channels))
        wav.setsampwidth(2)  # int16
        wav.setframerate(int(sample_rate_hz))
        wav.writeframes(bytes(pcm_bytes))
    return output


def build_custom_vocabulary(cfg: dict[str, Any]) -> list[str]:
    """Build domain hints from config without creating a second ontology."""

    voice_cfg = dict(cfg.get("voice", {}) or {})
    values: list[str] = []
    values.extend(str(x) for x in voice_cfg.get("custom_vocabulary", []) or [])

    aliases = cfg.get("target", {}).get("aliases", {}) or {}
    if isinstance(aliases, dict):
        values.extend(str(k) for k in aliases.keys())
        values.extend(str(v) for v in aliases.values())

    out: list[str] = []
    seen: set[str] = set()
    for raw in values:
        item = raw.strip()
        if not item or item in seen:
            continue
        seen.add(item)
        out.append(item)
    return out
