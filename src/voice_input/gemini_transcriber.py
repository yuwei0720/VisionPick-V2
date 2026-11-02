from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable


def transcribe_audio_file(
    path: str | Path,
    *,
    model: str = "gemini-3.5-transcribe",
    language_codes: Iterable[str] | None = None,
    custom_vocabulary: Iterable[str] | None = None,
    client: Any = None,
) -> str:
    """Transcribe one local audio file with Gemini Transcribe.

    The Google Gen AI import is intentionally lazy so normal ``--target`` and
    non-voice startup paths do not gain an import-time dependency on the
    transcription backend.
    """

    audio_path = Path(path)
    if not audio_path.is_file():
        raise FileNotFoundError(f"找不到錄音檔: {audio_path}")

    if client is None:
        try:
            from google import genai
        except ImportError as exc:
            raise RuntimeError(
                "Voice MVP 需要 google-genai；請先安裝 requirements.txt"
            ) from exc
        client = genai.Client()

    transcription_config: dict[str, Any] = {
        # Preserve the spoken command as literally as possible. The existing
        # semantic parser/validator remains responsible for interpretation.
        "mode": "verbatim",
    }

    if language_codes is not None:
        transcription_config["language_codes"] = [
            str(code).strip() for code in language_codes if str(code).strip()
        ]

    vocabulary = _dedupe_nonempty(custom_vocabulary or [])
    if vocabulary:
        transcription_config["custom_vocabulary"] = vocabulary

    uploaded = client.files.upload(file=str(audio_path))
    try:
        interaction = client.interactions.create(
            model=model,
            input=[
                {
                    "type": "audio",
                    "uri": uploaded.uri,
                    "mime_type": uploaded.mime_type,
                }
            ],
            generation_config={
                "transcription_config": transcription_config,
            },
        )
        transcript = str(getattr(interaction, "output_text", "") or "").strip()
        if not transcript:
            raise RuntimeError("Gemini Transcribe 沒有回傳可用文字")
        return transcript
    finally:
        # Gemini File API uploads are temporary, but delete immediately so a
        # robot command recording is not retained for the default 48-hour TTL.
        uploaded_name = getattr(uploaded, "name", None)
        if uploaded_name:
            try:
                client.files.delete(name=uploaded_name)
            except Exception:
                # Cleanup failure must not mask a successful transcription or
                # the primary API exception.
                pass


def _dedupe_nonempty(values: Iterable[str]) -> list[str]:
    out: list[str] = []
    seen: set[str] = set()
    for value in values:
        item = str(value).strip()
        if not item or item in seen:
            continue
        seen.add(item)
        out.append(item)
    return out
