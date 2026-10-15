"""STM32 motor-controller abstraction.

Serial protocol (matches STM32 firmware in main_v3.c):
  Jetson → STM32 (1 byte):
    F = forward, B = reverse, L = left, R = right, S = stop
    X = spin clockwise, A = spin anti-clockwise
    1..5 = speed (20% / 40% / 60% / 80% / 100%)
  STM32 → Jetson:
    'A'                                    = ACK for the most recent command
    'E' + 4 bytes (LE int16 × 2)           = encoder telemetry (left, right)
    'V' + ASCII version + '\\n'            = boot version (e.g. "V3\\n")

The class exposes:
  - send(cmd): non-blocking enqueue + watchdog update
  - snapshot() : latest ACK age, encoder counts/deltas, firmware version
  - start()/close() : lifecycle
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial

from config import STM32_PORT, BAUD_RATE, STM32_ACK_TIMEOUT_MS

log = logging.getLogger('agv.stm32')


# ── Protocol constants ──────────────────────────────────────────────
VALID_DIRECTIONS: frozenset[str] = frozenset('FBLRSXA')
VALID_SPEEDS:     frozenset[str] = frozenset('12345')

# STM32 hardware timer counters are 16-bit and free-running.
ENCODER_FULL_SCALE = 65536
ENCODER_HALF_SCALE = 32768

# Boot-version line is "V" + up to N chars + "\n". We cap the buffer so
# a stuck firmware can't make us read forever.
MAX_VERSION_LEN = 16


# ── State container ─────────────────────────────────────────────────
@dataclass
class Stm32State:
    last_ack_monotonic: float = 0.0
    encoder_left:         int = 0       # raw value (wraps every 65 536)
    encoder_right:        int = 0       # raw value (wraps every 65 536)
    encoder_delta_left:   int = 0       # signed ticks since last boot
    encoder_delta_right:  int = 0       # signed ticks since last boot
    firmware_version:     str  = ''     # populated from "V3\n" boot msg
    connected:            bool = False
    running:              bool = True
    last_error:           Optional[str] = None
    last_cmd:             Optional[str] = None
    last_speed:           Optional[str] = None


@dataclass
class Stm32Report:
    port:       str
    opened:     bool
    error:      Optional[str] = None
    latency_ms: float = 0.0


# ── Main class ──────────────────────────────────────────────────────
class STM32:
    """Threaded reader + blocking writer for the motor-controller board."""

    def __init__(self, port: Optional[str] = None, baud: Optional[int] = None) -> None:
        self.port = port or STM32_PORT
        self.baud = baud or BAUD_RATE
        self._serial: Optional[serial.Serial] = None
        self._state = Stm32State()
        self._lock = threading.Lock()
        self._reader: Optional[threading.Thread] = None

        # Internal encoder baseline (raw counter values). Not exposed in
        # the snapshot — used only to compute wrap-around-safe deltas.
        self._last_raw_left:  Optional[int] = None
        self._last_raw_right: Optional[int] = None

    # ── context manager ──────────────────────────────────────────────
    def __enter__(self) -> 'STM32':
        self.connect()
        return self

    def __exit__(self, *_args) -> None:
        self.close()

    # ── lifecycle ─────────────────────────────────────────────────────
    def connect(self) -> Stm32Report:
        t0 = time.monotonic()
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=1)
            # The STM32 sends "V3\n" within a few hundred ms of boot.
            # Give it a moment to finish its own UART init.
            time.sleep(1)
            with self._lock:
                self._state.running              = True
                self._state.last_ack_monotonic   = time.monotonic()
                self._state.connected            = False
                self._state.encoder_left         = 0
                self._state.encoder_right        = 0
                self._state.encoder_delta_left   = 0
                self._state.encoder_delta_right  = 0
                self._state.firmware_version     = ''
                self._last_raw_left              = None
                self._last_raw_right             = None
            self._reader = threading.Thread(
                target=self._reader_loop, daemon=True, name='stm32-reader',
            )
            self._reader.start()
            log.info('STM32 connected on %s', self.port)
            return Stm32Report(self.port, True,
                               latency_ms=round((time.monotonic() - t0) * 1000, 1))
        except Exception as e:
            log.error('Cannot connect STM32 on %s: %s', self.port, e)
            log.error('AGV will not move without STM32 connection.')
            return Stm32Report(self.port, False,
                               error=f'{type(e).__name__}: {e}',
                               latency_ms=round((time.monotonic() - t0) * 1000, 1))

    def close(self) -> None:
        with self._lock:
            self._state.running = False
        if self._reader:
            self._reader.join(timeout=2)
            self._reader = None
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        with self._lock:
            self._state.connected = False

    def isOpen(self) -> bool:
        return self._serial is not None

    # ── command output ────────────────────────────────────────────────
    def send(self, cmd: str, speed: Optional[str] = None) -> bool:
        """Send a direction command (and optional speed byte) to the STM32."""
        if not self._serial:
            return False
        try:
            if speed is not None and speed in VALID_SPEEDS:
                self._serial.write(speed.encode())
                with self._lock:
                    self._state.last_speed = speed
            self._serial.write(cmd.encode())
            with self._lock:
                self._state.last_cmd = cmd
            return True
        except serial.SerialException as exc:
            log.warning('[STM32] write failed: %s', exc)
            with self._lock:
                self._state.connected = False
                self._state.last_error = f'SerialException: {exc}'
            return False

    # ── state access ─────────────────────────────────────────────────
    def snapshot(self) -> Stm32State:
        with self._lock:
            return Stm32State(
                last_ack_monotonic = self._state.last_ack_monotonic,
                encoder_left        = self._state.encoder_left,
                encoder_right       = self._state.encoder_right,
                encoder_delta_left  = self._state.encoder_delta_left,
                encoder_delta_right = self._state.encoder_delta_right,
                firmware_version    = self._state.firmware_version,
                connected           = self._state.connected,
                running             = self._state.running,
                last_error          = self._state.last_error,
                last_cmd            = self._state.last_cmd,
                last_speed          = self._state.last_speed,
            )

    def ack_age_ms(self) -> float:
        """Milliseconds since the last 'A' byte from the STM32."""
        with self._lock:
            return (time.monotonic() - self._state.last_ack_monotonic) * 1000.0

    def link_alive(self) -> bool:
        """True if STM32 is reachable and ACKed within the timeout."""
        return self.ack_age_ms() <= STM32_ACK_TIMEOUT_MS

    # ── compatibility shims ──────────────────────────────────────────
    @property
    def connected(self) -> bool:
        with self._lock: return self._state.connected

    @property
    def encoder_left(self) -> int:
        with self._lock: return self._state.encoder_left

    @property
    def encoder_right(self) -> int:
        with self._lock: return self._state.encoder_right

    @property
    def encoder_delta_left(self) -> int:
        with self._lock: return self._state.encoder_delta_left

    @property
    def encoder_delta_right(self) -> int:
        with self._lock: return self._state.encoder_delta_right

    @property
    def firmware_version(self) -> str:
        with self._lock: return self._state.firmware_version

    @property
    def last_cmd(self) -> Optional[str]:
        with self._lock: return self._state.last_cmd

    # ── internal helpers ──────────────────────────────────────────────
    @staticmethod
    def _int16_le(lo: int, hi: int) -> int:
        """Decode a little-endian signed 16-bit integer."""
        val = (hi << 8) | lo
        if val >= ENCODER_HALF_SCALE:
            val -= ENCODER_FULL_SCALE
        return val

    @staticmethod
    def _compute_delta(current: int, last: Optional[int]) -> int:
        """Wrap-around-safe delta between two raw 16-bit counter readings."""
        if last is None:
            return 0
        diff = (current - last) % ENCODER_FULL_SCALE
        if diff >= ENCODER_HALF_SCALE:
            diff -= ENCODER_FULL_SCALE
        return diff

    # ── background reader ─────────────────────────────────────────────
    def _reader_loop(self) -> None:
        log.info('[STM32] reader thread started')
        pending_encoder_bytes = 0
        encoder_buffer        = bytearray()
        collecting_version    = False
        version_buffer        = bytearray()

        while True:
            with self._lock:
                if not self._state.running:
                    break
            try:
                if not self._serial:
                    time.sleep(0.5)
                    continue

                raw = self._serial.read(1)
                if not raw:
                    continue
                b = raw[0]

                # ── Mid-encoder frame: collect 4 bytes after 'E' ──
                if pending_encoder_bytes > 0:
                    encoder_buffer.append(b)
                    pending_encoder_bytes -= 1
                    if pending_encoder_bytes == 0 and len(encoder_buffer) == 4:
                        left  = self._int16_le(encoder_buffer[0], encoder_buffer[1])
                        right = self._int16_le(encoder_buffer[2], encoder_buffer[3])
                        with self._lock:
                            self._state.encoder_left  = left
                            self._state.encoder_right = right
                            self._state.encoder_delta_left  += self._compute_delta(
                                left, self._last_raw_left)
                            self._state.encoder_delta_right += self._compute_delta(
                                right, self._last_raw_right)
                            self._last_raw_left  = left
                            self._last_raw_right = right
                        encoder_buffer.clear()
                    continue

                # ── Mid-version line: collect bytes until '\n' ──
                if collecting_version:
                    if b == ord('\n'):
                        with self._lock:
                            self._state.firmware_version = version_buffer.decode(
                                'ascii', errors='replace')
                            # Reset deltas so encoder counts are
                            # measured from this boot onward.
                            self._state.encoder_delta_left  = 0
                            self._state.encoder_delta_right = 0
                            self._last_raw_left  = None
                            self._last_raw_right = None
                        log.info('[STM32] firmware version: %r',
                                 self._state.firmware_version)
                        collecting_version = False
                        version_buffer.clear()
                    elif len(version_buffer) < MAX_VERSION_LEN:
                        version_buffer.append(b)
                    # else: silently drop overflow bytes
                    continue

                # ── First-byte dispatch ──
                if b == ord('A'):
                    with self._lock:
                        self._state.last_ack_monotonic = time.monotonic()
                        self._state.connected = True
                elif b == ord('E'):
                    pending_encoder_bytes = 4
                    encoder_buffer.clear()
                elif b == ord('V'):
                    collecting_version = True
                    version_buffer.clear()
                # Any other byte is silently ignored.

            except (serial.SerialException, OSError) as exc:
                log.warning('[STM32] reader disconnected: %s', exc)
                with self._lock:
                    self._state.connected = False
                    self._state.last_error = f'{type(exc).__name__}: {exc}'
                time.sleep(0.5)

        log.info('[STM32] reader thread exiting')
