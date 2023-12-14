
import gc

class GarbageCollector:
    def __init__(self):
        self._state = 0
    def run(self):
        if self._state == 0:
            gc.collect()
        else:
            raise ValueError("Invalid state")
        yield self._state
