"""OS advisory lock shared by supported CLI/UI processes using the same port."""
from pathlib import Path
import hashlib
import os
import tempfile


class RobotLease:
    def __init__(self, port):
        key = hashlib.sha256(str(port).strip().lower().encode()).hexdigest()[:20]
        self.path = Path(tempfile.gettempdir()) / f"edgeai-robot-{key}.lock"
        self.stream = None

    def acquire(self):
        self.stream = self.path.open("a+b")
        try:
            if os.name == "nt":
                import msvcrt
                self.stream.seek(0, 2)
                if not self.stream.tell(): self.stream.write(b"0"); self.stream.flush()
                self.stream.seek(0)
                msvcrt.locking(self.stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, BlockingIOError) as exc:
            self.stream.close(); self.stream = None
            raise RuntimeError("Robot port is owned by another VisionPick run") from exc
        return self

    def release(self):
        if self.stream:
            if os.name == "nt":
                import msvcrt
                self.stream.seek(0); msvcrt.locking(self.stream.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
            self.stream.close(); self.stream = None

    def __enter__(self): return self.acquire()
    def __exit__(self, *args): self.release()
