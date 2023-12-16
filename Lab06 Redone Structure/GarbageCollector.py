"""Houses the GarbageCollector task class."""

import gc

class GarbageCollector:
    def __init__(self):
        """This is the GarbageCollector, it CollectsGarbage."""

        self._state = 0
    def run(self):
        """Implementation as a generator function.
        """
        
        if self._state == 0:
            gc.collect()
        else:
            raise ValueError("Invalid state")
        yield self._state
