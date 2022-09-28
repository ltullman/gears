import random, time

if getattr(random, 'getrandbits', None) is None:
    def getrandbits(n):
        return random.randint(0, 2 ** n - 1)
    random.getrandbits = getrandbits

if getattr(time, 'ticks_ms', None) is None:
    def ticks_ms():
        return int(time.time() * 1000)
    time.ticks_ms = ticks_ms
