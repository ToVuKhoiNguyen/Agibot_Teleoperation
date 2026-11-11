"""Asynchronous non-blocking JSONL queue writer."""
import queue, threading, json, time

class AsyncJsonlWriter:
    def __init__(self, file_path):
        self.file_path = file_path
        self._q = queue.Queue(maxsize=10000)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def log(self, item: dict):
        try:
            self._q.put_nowait(item)
        except queue.Full:
            pass

    def _run(self):
        try:
            with open(self.file_path, "a", encoding="utf-8") as f:
                while not self._stop.is_set() or not self._q.empty():
                    try:
                        item = self._q.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    f.write(json.dumps(item) + "\n")
                    self._q.task_done()
        except Exception as e:
            print(f"[AsyncJsonlWriter] Error writing {self.file_path}: {e}")

    def close(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.5)
