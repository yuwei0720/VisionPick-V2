"""Optional advanced serial/Modbus tools for the local operator console.

The normal robot path remains ``pymycobot``.  This module is an operator-facing
maintenance tool for enumerating serial devices and, when explicitly opened,
working with generic RS-232 style byte streams or RS-485/Modbus RTU frames.
It never auto-opens ports and never sends data during device discovery.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import threading
import time
import zlib


class SerialToolError(RuntimeError):
    pass


def _serial_module():
    try:
        import serial
        from serial.tools.list_ports import comports
    except ImportError as exc:  # pragma: no cover - depends on host
        raise SerialToolError("pyserial 未安裝；請安裝 requirements.txt 後再使用序列通訊工具") from exc
    return serial, comports


def enumerate_ports():
    """Return rich, JSON-safe metadata for currently enumerated serial ports."""
    try:
        _, comports = _serial_module()
    except SerialToolError:
        return []
    result = []
    for p in comports():
        result.append({
            "device": getattr(p, "device", ""),
            "name": getattr(p, "name", None),
            "description": getattr(p, "description", None),
            "manufacturer": getattr(p, "manufacturer", None),
            "product": getattr(p, "product", None),
            "serial_number": getattr(p, "serial_number", None),
            "vid": getattr(p, "vid", None),
            "pid": getattr(p, "pid", None),
            "hwid": getattr(p, "hwid", None),
        })
    return result


def crc8(data: bytes) -> bytes:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return bytes([crc])


def crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def crc32(data: bytes) -> bytes:
    return (zlib.crc32(data) & 0xFFFFFFFF).to_bytes(4, "little")


def crc64(data: bytes) -> bytes:
    value = 0
    polynomial = 0x42F0E1EBA9EA3693
    for b in data:
        value ^= b << 56
        for _ in range(8):
            value = ((value << 1) ^ polynomial) & 0xFFFFFFFFFFFFFFFF if value & 0x8000000000000000 else (value << 1) & 0xFFFFFFFFFFFFFFFF
    return value.to_bytes(8, "little")


CRC = {"none": lambda data: b"", "CRC8": crc8, "CRC16": crc16_modbus, "CRC32": crc32, "CRC64": crc64}


def build_modbus_request(slave_id: int, function_code: int, address: int, quantity: int, crc_type="CRC16") -> bytes:
    """Build the same six-byte Modbus request body used by the legacy test tool.

    This helper intentionally does not claim every function code has the same
    PDU shape.  It is suitable for the read-style ID/FC/address/quantity form
    used by the existing tool and common FC01/02/03/04 diagnostics.
    """
    for name, value, low, high in (
        ("slave_id", slave_id, 0, 255),
        ("function_code", function_code, 0, 255),
        ("address", address, 0, 65535),
        ("quantity", quantity, 0, 65535),
    ):
        if type(value) is not int or not low <= value <= high:
            raise ValueError(f"{name} 必須介於 {low}–{high}")
    if crc_type not in CRC:
        raise ValueError("不支援的 CRC")
    body = bytes([slave_id, function_code]) + address.to_bytes(2, "big") + quantity.to_bytes(2, "big")
    return body + CRC[crc_type](body)


def parse_hex_bytes(text: str) -> bytes:
    cleaned = "".join(str(text).replace("0x", "").replace("0X", "").split())
    if len(cleaned) % 2:
        raise ValueError("HEX 位元組數必須為偶數")
    try:
        return bytes.fromhex(cleaned)
    except ValueError as exc:
        raise ValueError("HEX 格式無效") from exc


@dataclass
class ChannelConfig:
    channel: str
    port: str
    baudrate: int
    bytesize: int = 8
    parity: str = "N"
    stopbits: float = 1.0
    timeout: float = 0.1

    def validate(self):
        if self.channel not in {"rs232", "rs485"}: raise ValueError("channel 必須為 rs232 或 rs485")
        if not self.port or len(self.port) > 128: raise ValueError("port 無效")
        if not 300 <= int(self.baudrate) <= 4_000_000: raise ValueError("baudrate 超出允許範圍")
        if self.bytesize not in {5, 6, 7, 8}: raise ValueError("data bits 無效")
        if self.parity not in {"N", "E", "O"}: raise ValueError("parity 無效")
        if float(self.stopbits) not in {1.0, 1.5, 2.0}: raise ValueError("stop bits 無效")
        if not 0.01 <= float(self.timeout) <= 5.0: raise ValueError("timeout 無效")
        return self


class SerialToolManager:
    """Two explicit operator-opened channels with collision prevention."""

    def __init__(self):
        self._lock = threading.RLock()
        self._channels = {"rs232": None, "rs485": None}
        self._configs = {"rs232": None, "rs485": None}
        self._rx = {"rs232": bytearray(), "rs485": bytearray()}

    def _status_one(self, channel):
        handle = self._channels[channel]
        cfg = self._configs[channel]
        return {
            "channel": channel,
            "open": bool(handle is not None and getattr(handle, "is_open", True)),
            "config": asdict(cfg) if cfg else None,
            "rx_waiting": len(self._rx[channel]),
        }

    def status(self):
        with self._lock:
            return {name: self._status_one(name) for name in self._channels}

    def open(self, config: ChannelConfig, *, reserved_ports=()):
        cfg = config.validate()
        with self._lock:
            if cfg.port in set(reserved_ports):
                raise SerialToolError(f"{cfg.port} 目前保留給正式機械臂控制；請先停止 Robot run 或選擇其他埠")
            other = "rs485" if cfg.channel == "rs232" else "rs232"
            other_cfg = self._configs[other]
            if self._channels[other] is not None and other_cfg and other_cfg.port == cfg.port:
                raise SerialToolError(f"{cfg.port} 已被 {other.upper()} 工具開啟")
            if self._channels[cfg.channel] is not None:
                raise SerialToolError(f"{cfg.channel.upper()} 已開啟；請先中斷")
            serial, _ = _serial_module()
            pmap = {"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN, "O": serial.PARITY_ODD}
            smap = {1.0: serial.STOPBITS_ONE, 1.5: serial.STOPBITS_ONE_POINT_FIVE, 2.0: serial.STOPBITS_TWO}
            handle = serial.Serial(
                port=cfg.port,
                baudrate=int(cfg.baudrate),
                bytesize=int(cfg.bytesize),
                parity=pmap[cfg.parity],
                stopbits=smap[float(cfg.stopbits)],
                timeout=float(cfg.timeout),
            )
            self._channels[cfg.channel] = handle
            self._configs[cfg.channel] = cfg
            self._rx[cfg.channel].clear()
            return self._status_one(cfg.channel)

    def close(self, channel):
        if channel not in self._channels: raise ValueError("未知 channel")
        with self._lock:
            handle = self._channels[channel]
            if handle is not None:
                try: handle.close()
                finally: self._channels[channel] = None
            return self._status_one(channel)

    def _handle(self, channel):
        if channel not in self._channels: raise ValueError("未知 channel")
        handle = self._channels[channel]
        if handle is None or not getattr(handle, "is_open", True):
            raise SerialToolError(f"{channel.upper()} 尚未建立連線")
        return handle

    def send(self, channel, data: bytes):
        if not isinstance(data, (bytes, bytearray)) or len(data) > 4096:
            raise ValueError("單次傳送必須為 1–4096 bytes")
        with self._lock:
            handle = self._handle(channel)
            count = handle.write(bytes(data))
            return {"sent": int(count if count is not None else len(data)), "tx_hex": bytes(data).hex(" ").upper()}

    def poll(self, channel, *, limit=4096):
        with self._lock:
            handle = self._handle(channel)
            waiting = min(int(getattr(handle, "in_waiting", 0) or 0), int(limit))
            if waiting:
                self._rx[channel].extend(handle.read(waiting))
            data = bytes(self._rx[channel][-limit:])
            return {"rx_hex": data.hex(" ").upper(), "rx_text": data.decode("utf-8", errors="replace"), "bytes": len(data)}

    def clear(self, channel):
        with self._lock:
            self._rx[channel].clear()
            return {"cleared": True}

    def close_all(self):
        for name in tuple(self._channels):
            try: self.close(name)
            except Exception: pass
