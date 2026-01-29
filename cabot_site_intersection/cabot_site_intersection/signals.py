import json
import random
import time


class SignalGenerator:
    def __init__(self, id, cycles, step, start=0, delay_stddev=0.0):
        self.id = id
        self.cycles = cycles
        self.total = sum(phase["duration"] for phase in cycles[0])
        self.tick = start
        self.step = step
        self.delay_stddev = delay_stddev
        self.next_emit_at = time.time()

    def _delay(self):
        if self.delay_stddev <= 0:
            return 0.0
        delay = random.gauss(0.0, self.delay_stddev)
        return abs(delay)

    def next(self):
        now = time.time()
        if now < self.next_emit_at:
            return None
        status = generate_intersection_status(self.id, self.cycles, self.tick, self.total)
        delay = self._delay()
        self.tick += self.step + delay
        self.next_emit_at = now + self.step + delay
        return status


class SignalGeneratorDummy001GreenFirst(SignalGenerator):
    def __init__(self, start=0, step=0.5, delay_stddev=0.0):
        cycles = [
            [
                {"state": "red", "duration": 25},
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 6},
            ],
            [
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 6},
                {"state": "red", "duration": 25},
            ],
        ]
        super().__init__("dummy-id-001", cycles, step, start, delay_stddev)


class SignalGeneratorDummy001RedFirst(SignalGenerator):
    def __init__(self, start=0, step=0.5, delay_stddev=0.0):
        cycles = [
            [
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 3},
                {"state": "red", "duration": 25},
            ],
            [
                {"state": "red", "duration": 25},
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 3},
            ],
        ]
        super().__init__("dummy-id-001", cycles, step, start, delay_stddev)


class SignalGeneratorDummy001RedFirstDelay(SignalGeneratorDummy001RedFirst):
    def __init__(self, start=0, step=0.5, delay_stddev=0.5):
        super().__init__(start=start, step=step, delay_stddev=delay_stddev)


class SignalGeneratorDummy002RedFirst(SignalGenerator):
    def __init__(self, start=0, step=0.5, delay_stddev=0.0):
        cycles = [
            [
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 8},
                {"state": "red", "duration": 25},
            ],
            [
                {"state": "red", "duration": 25},
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 8},
            ],
        ]
        super().__init__("dummy-id-001", cycles, step, start, delay_stddev)


def get_state(cycle, tick, total):
    t = tick % total
    for i, phase in enumerate(cycle):
        t -= phase["duration"]
        if t < 0:
            next_index = (i + 1) % len(cycle)
            if len(cycle) > 3 and next_index == 0:
                next_index += 1
            next_phase = cycle[next_index]
            return {
                "state": phase["state"],
                "remaining_seconds": -t,
                "next_state": next_phase["state"],
                "next_programmed_seconds": next_phase["duration"],
            }


def generate_intersection_status(id, cycles, tick, total):
    # Base structure
    status = [
        {
            "id": id,
            "signals": [
                {"group": 1},
                {"group": 2},
            ],
            "buttons": [
                {
                    "number": 1,
                    "signal_groups": [],
                    "state": "unavailable",
                    "supports": [],
                },
                {
                    "number": 2,
                    "signal_groups": [],
                    "state": "unavailable",
                    "supports": [],
                },
            ],
        }
    ]

    # Update signal states
    status[0]["signals"][0].update(get_state(cycles[0], tick, total))
    status[0]["signals"][1].update(get_state(cycles[1], tick, total))
    status[0]["timestamp"] = time.time()
    return status


if __name__ == "__main__":
    generator = SignalGeneratorDummy001RedFirstDelay05()
    for tick in range(0, 101):
        status = generator.next()
        if status is not None:
            print(json.dumps(status))
        time.sleep(0.1)
