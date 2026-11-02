"""Python side of the serial protocol implemented by firmware/braccio_serial.ino.

    HOME | GRIP <g> | MOVE g r p e s b      (115200 baud, newline-terminated)

The firmware replies "OK: ..." or "ERROR: ...". Braccio.moveTo() returns before the
motion ends, so we wait `move_settle_s` after every command.
"""
import time


class BraccioSerialError(RuntimeError):
    pass


class BraccioSerial:
    def __init__(self, port, baudrate=115200, timeout_s=5.0, move_settle_s=1.5):
        import serial  # pyserial
        self.ser = serial.Serial(port, baudrate, timeout=timeout_s)
        self.timeout_s = timeout_s
        self.move_settle_s = move_settle_s
        self._wait_ready()

    def _wait_ready(self):
        # Opening the port resets most Arduino boards; wait for the banner.
        deadline = time.time() + max(self.timeout_s, 5.0)
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if "ready" in line.lower():
                return
            if line.startswith("ERROR"):
                raise BraccioSerialError(line)
        # Board may already have been running: not fatal.

    def send(self, line):
        self.ser.reset_input_buffer()
        self.ser.write((line.strip() + "\n").encode())
        deadline = time.time() + self.timeout_s
        while time.time() < deadline:
            reply = self.ser.readline().decode(errors="replace").strip()
            if not reply:
                continue
            if reply.startswith("OK"):
                time.sleep(self.move_settle_s)
                return reply
            if reply.startswith("ERROR"):
                raise BraccioSerialError(f"{line!r} -> {reply}")
        raise BraccioSerialError(f"Timeout waiting for reply to {line!r}")

    def home(self):
        return self.send("HOME")

    def grip(self, g):
        return self.send(f"GRIP {g:.1f}")

    def move(self, angles):
        """angles: [grip, roll, pitch, elbow, shoulder, base] in firmware degrees."""
        if len(angles) != 6:
            raise ValueError("MOVE needs 6 angles: g r p e s b")
        return self.send("MOVE " + " ".join(f"{a:.1f}" for a in angles))

    def close(self):
        self.ser.close()


class MockSerial:
    """Records commands instead of sending them (simulation and tests)."""

    def __init__(self, *_, **__):
        self.log = []

    def send(self, line):
        self.log.append(line)
        return "OK"

    def home(self):
        return self.send("HOME")

    def grip(self, g):
        return self.send(f"GRIP {g:.1f}")

    def move(self, angles):
        if len(angles) != 6:
            raise ValueError("MOVE needs 6 angles")
        return self.send("MOVE " + " ".join(f"{a:.1f}" for a in angles))

    def close(self):
        pass
