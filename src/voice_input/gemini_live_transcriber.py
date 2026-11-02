from __future__ import annotations

import asyncio
import contextlib
from typing import Any, Callable, Iterable


class LiveTranscriptionError(RuntimeError):
    """Live transcription failed; ``pcm_bytes`` may be reused by a fallback."""

    def __init__(self, message: str, *, pcm_bytes: bytes = b""):
        super().__init__(message)
        self.pcm_bytes = bytes(pcm_bytes)


def transcribe_microphone_live(
    *,
    duration_seconds: float,
    sample_rate_hz: int = 16000,
    channels: int = 1,
    chunk_ms: int = 100,
    model: str = "gemini-3.5-transcribe-live",
    language_codes: Iterable[str] | None = None,
    custom_vocabulary: Iterable[str] | None = None,
    mode: str = "VERBATIM",
    final_timeout_seconds: float = 6.0,
    client: Any = None,
    types_module: Any = None,
    stream_factory: Callable[..., Any] | None = None,
    on_interim: Callable[[str], None] | None = None,
) -> str:
    """Stream one fixed-duration microphone command to Gemini Live Transcribe.

    Audio is raw signed 16-bit PCM, mono, 16 kHz, sent in ~100 ms chunks as
    recommended by the Gemini Live transcription documentation. The returned
    value contains only finalized ``input_transcription`` text.

    The fixed duration intentionally preserves the current Voice MVP UX. A
    later iteration can replace this boundary with local VAD / push-to-talk
    without changing the downstream semantic pipeline.
    """

    return asyncio.run(
        _transcribe_microphone_live_async(
            duration_seconds=duration_seconds,
            sample_rate_hz=sample_rate_hz,
            channels=channels,
            chunk_ms=chunk_ms,
            model=model,
            language_codes=language_codes,
            custom_vocabulary=custom_vocabulary,
            mode=mode,
            final_timeout_seconds=final_timeout_seconds,
            client=client,
            types_module=types_module,
            stream_factory=stream_factory,
            on_interim=on_interim,
        )
    )


async def _transcribe_microphone_live_async(
    *,
    duration_seconds: float,
    sample_rate_hz: int,
    channels: int,
    chunk_ms: int,
    model: str,
    language_codes: Iterable[str] | None,
    custom_vocabulary: Iterable[str] | None,
    mode: str,
    final_timeout_seconds: float,
    client: Any,
    types_module: Any,
    stream_factory: Callable[..., Any] | None,
    on_interim: Callable[[str], None] | None,
) -> str:
    if duration_seconds <= 0:
        raise ValueError("voice.record_seconds 必須大於 0")
    if sample_rate_hz != 16000:
        raise ValueError("Gemini Live Transcribe MVP 目前固定使用 16000 Hz")
    if channels != 1:
        raise ValueError("Gemini Live Transcribe MVP 目前只支援單聲道 channels=1")
    if chunk_ms < 50 or chunk_ms > 250:
        raise ValueError("voice.live_chunk_ms 建議介於 50~250 ms")
    if final_timeout_seconds <= 0:
        raise ValueError("voice.live_final_timeout_s 必須大於 0")

    if client is None or types_module is None:
        try:
            from google import genai
            from google.genai import types
        except ImportError as exc:
            raise RuntimeError(
                "Gemini Live Voice 需要 google-genai；請先安裝 requirements.txt"
            ) from exc
        if client is None:
            client = genai.Client()
        if types_module is None:
            types_module = types

    if stream_factory is None:
        try:
            import sounddevice as sd
        except ImportError as exc:
            raise RuntimeError(
                "Gemini Live Voice 需要 sounddevice；請先安裝 requirements.txt"
            ) from exc
        stream_factory = sd.RawInputStream

    language_list = _dedupe_nonempty(language_codes or [])
    vocabulary = _dedupe_nonempty(custom_vocabulary or [])
    mode_value = str(mode or "VERBATIM").strip().upper()
    if mode_value not in {"VERBATIM", "SMART"}:
        raise ValueError("voice.live_mode 必須是 VERBATIM 或 SMART")

    audio_cfg: dict[str, Any] = {
        "language_codes": language_list,
        "mode": mode_value,
    }
    if vocabulary:
        audio_cfg["custom_vocabulary"] = vocabulary

    # Use a plain config mapping rather than requiring the newest generated
    # SDK config classes. google-genai accepts the same snake_case mapping and
    # this keeps the project less sensitive to minor SDK type revisions.
    config: dict[str, Any] = {
        "response_modalities": ["TEXT"],
        "input_audio_transcription": audio_cfg,
    }

    frame_count = max(1, int(round(sample_rate_hz * chunk_ms / 1000.0)))
    total_frames = int(round(duration_seconds * sample_rate_hz))
    pcm_buffer = bytearray()
    final_parts: list[str] = []

    try:
        async with client.aio.live.connect(model=model, config=config) as session:
            receiver = asyncio.create_task(
                _receive_final_transcripts(
                    session,
                    final_parts=final_parts,
                    on_interim=on_interim,
                )
            )

            sent_frames = 0
            try:
                with stream_factory(
                    samplerate=sample_rate_hz,
                    channels=channels,
                    dtype="int16",
                    blocksize=frame_count,
                ) as stream:
                    while sent_frames < total_frames:
                        frames_to_read = min(frame_count, total_frames - sent_frames)
                        data, overflowed = await asyncio.to_thread(
                            stream.read, frames_to_read
                        )
                        chunk = bytes(data)
                        if not chunk:
                            raise RuntimeError("麥克風沒有回傳音訊資料")
                        pcm_buffer.extend(chunk)
                        sent_frames += frames_to_read
                        if overflowed:
                            # Keep the command usable; PortAudio overflow means
                            # one chunk may have been dropped, not that the
                            # whole turn must be discarded.
                            pass
                        await session.send_realtime_input(
                            audio=types_module.Blob(
                                data=chunk,
                                mime_type=f"audio/pcm;rate={sample_rate_hz}",
                            )
                        )

                # Fixed-duration MVP: explicitly flush the final speech turn.
                await session.send_realtime_input(audio_stream_end=True)

                try:
                    await asyncio.wait_for(
                        receiver,
                        timeout=float(final_timeout_seconds),
                    )
                except TimeoutError:
                    receiver.cancel()
                    with contextlib.suppress(asyncio.CancelledError):
                        await receiver
                    if not final_parts:
                        raise RuntimeError("Gemini Live Transcribe 等待 finalized transcript 逾時")
            finally:
                if not receiver.done():
                    receiver.cancel()
                    with contextlib.suppress(asyncio.CancelledError):
                        await receiver
    except LiveTranscriptionError:
        raise
    except Exception as exc:
        raise LiveTranscriptionError(
            f"Gemini Live Transcribe 失敗: {exc}",
            pcm_bytes=bytes(pcm_buffer),
        ) from exc

    transcript = " ".join(part.strip() for part in final_parts if part.strip()).strip()
    if not transcript:
        raise LiveTranscriptionError(
            "Gemini Live Transcribe 沒有回傳 finalized transcript",
            pcm_bytes=bytes(pcm_buffer),
        )
    return transcript


async def _receive_final_transcripts(
    session: Any,
    *,
    final_parts: list[str],
    on_interim: Callable[[str], None] | None,
) -> None:
    """Collect one Live API turn and stop as soon as it is finalized.

    ``google-genai`` keeps ``session.receive()`` open for the Live session even
    after a transcription turn has completed.  Waiting for the iterator itself
    to end therefore adds the full timeout to an otherwise completed command.
    Treat ``input_transcription`` as finalized text and return once the server
    also signals ``generation_complete``/``turn_complete``.
    """

    saw_final = False
    saw_complete = False
    async for response in session.receive():
        server_content = getattr(response, "server_content", None)
        if not server_content:
            continue

        interim = getattr(server_content, "interim_input_transcription", None)
        interim_text = str(getattr(interim, "text", "") or "").strip()
        if interim_text and on_interim is not None:
            on_interim(interim_text)

        final = getattr(server_content, "input_transcription", None)
        final_text = str(getattr(final, "text", "") or "").strip()
        if final_text:
            # One fixed-duration command maps to one transcription turn.  Avoid
            # duplicate copies if an SDK/service revision repeats the final.
            if not final_parts or final_parts[-1] != final_text:
                final_parts.append(final_text)
            saw_final = True

        generation_complete = bool(
            getattr(server_content, "generation_complete", False)
        )
        turn_complete = bool(getattr(server_content, "turn_complete", False))
        saw_complete = saw_complete or generation_complete or turn_complete
        if saw_final and saw_complete:
            return


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


def transcribe_pcm_live(chunks, *, model="gemini-3.5-transcribe-live", language_codes=None,
                        custom_vocabulary=None, mode="VERBATIM", final_timeout_seconds=10.,
                        on_update=None, client=None, stop_event=None, cancelled=None, session_timeout_s=60.):
    """Duplex PCM streaming; callbacks contain committed text plus replaceable interim.

    ``chunks`` yields raw mono int16/16 kHz blocks and ends on stop. No audio file
    is created and no file-transcription fallback is invoked.
    """
    operation = _transcribe_pcm_live_async(chunks, model=model, language_codes=language_codes,
        custom_vocabulary=custom_vocabulary, mode=mode, final_timeout_seconds=final_timeout_seconds,
        on_update=on_update, client=client, stop_event=stop_event)
    return asyncio.run(_guard_pcm_session(operation,stop_event,cancelled,session_timeout_s))


async def _guard_pcm_session(operation, stop_event, cancelled, timeout_s):
    task = asyncio.create_task(operation)
    deadline = asyncio.get_running_loop().time()+float(timeout_s)
    try:
        while not task.done():
            if cancelled is not None and cancelled():
                raise ValueError('voice_cancelled')
            if asyncio.get_running_loop().time() >= deadline:
                raise TimeoutError('voice_live_session_timeout')
            await asyncio.wait((task,),timeout=.05)
        return await task
    finally:
        if stop_event is not None:
            stop_event.set()
        if not task.done():
            task.cancel()
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await task


async def _transcribe_pcm_live_async(chunks, *, model, language_codes, custom_vocabulary, mode,
                                     final_timeout_seconds, on_update, client, stop_event):
    if mode not in {"VERBATIM", "SMART"}:
        raise ValueError("voice_invalid_live_mode")
    if not 0 < float(final_timeout_seconds) <= 30:
        raise ValueError("voice_invalid_final_timeout")
    if client is None:
        from google import genai
        client = genai.Client()
    config = {"response_modalities":["TEXT"], "input_audio_transcription":{
        "mode":mode, "language_codes":_dedupe_nonempty(language_codes or []),
        "custom_vocabulary":_dedupe_nonempty(custom_vocabulary or [])}}
    parts, interim = [], ""
    ending = asyncio.Event()
    def publish():
        committed = " ".join(parts)
        text = " ".join(x for x in (committed, interim) if x)
        if len(text) > 4000:
            raise ValueError("voice_transcript_too_long")
        if on_update:
            on_update(committed, interim)
    async with client.aio.live.connect(model=model, config=config) as session:
        async def receive():
            nonlocal interim
            # SDK receive() ends at each turn. Keep receiving subsequent turns
            # until the sender flushes the stream and a final turn is complete.
            while True:
                received = False
                async for response in session.receive():
                    received = True
                    content = getattr(response, "server_content", None)
                    if content is None:
                        continue
                    preview = getattr(content, "interim_input_transcription", None)
                    final = getattr(content, "input_transcription", None)
                    if preview is not None:
                        interim = str(getattr(preview, "text", "") or "").strip()
                        publish()
                    if final is not None:
                        text = str(getattr(final, "text", "") or "").strip()
                        if text:
                            parts.append(text)
                        interim = ""
                        publish()
                    if ending.is_set() and parts and (getattr(content,"turn_complete",False)
                            or getattr(content,"generation_complete",False)):
                        return
                if not received:
                    return  # Closed transport; never spin on an empty iterator.
        async def send():
            iterator = iter(chunks)
            sent_audio = False
            while True:
                chunk = await asyncio.to_thread(next, iterator, None)
                if chunk is None:
                    break
                if not isinstance(chunk,bytes) or not chunk or len(chunk)%2:
                    raise ValueError("voice_invalid_pcm_chunk")
                await session.send_realtime_input(audio={"data":chunk,"mime_type":"audio/pcm;rate=16000"})
                sent_audio = True
            if not sent_audio:
                raise ValueError('voice_empty_audio')
            ending.set()
            await session.send_realtime_input(audio_stream_end=True)
        receiver = asyncio.create_task(receive())
        sender = asyncio.create_task(send())
        try:
            done, _ = await asyncio.wait((sender,receiver),return_when=asyncio.FIRST_COMPLETED)
            if receiver in done:
                await receiver  # Propagate an early provider error immediately.
                if not sender.done():
                    raise ConnectionError("voice_live_stream_closed_early")
            await sender
            await asyncio.wait_for(receiver,timeout=float(final_timeout_seconds))
        finally:
            if stop_event is not None:
                stop_event.set()
            for task in (sender,receiver):
                if not task.done():
                    task.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await task
    transcript = " ".join(parts).strip()
    if not transcript:
        raise ValueError("voice_empty_transcript")
    return transcript
