import json
import time


class SignalGenerator:
    def __init__(self, id, cycles, step):
        self.id = id
        self.cycles = cycles
        self.total = sum(phase["duration"] for phase in cycles[0])
        self.tick = 0
        self.step = step

    def next(self):
        status = generate_intersection_status(self.id, self.cycles, self.tick, self.total)
        self.tick += self.step
        return status


class SignalGeneratorDummy001GreenFirst(SignalGenerator):
    def __init__(self):
        cycles = [
            [
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 6},
                {"state": "red", "duration": 25},
            ],
            [
                {"state": "red", "duration": 25},
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 6},
            ],
        ]
        super().__init__("dummy-id-001", cycles, 0.5)


class SignalGeneratorDummy001RedFirst(SignalGenerator):
    def __init__(self):
        cycles = [
            [
                {"state": "red", "duration": 25},
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 3},
            ],
            [
                {"state": "green", "duration": 20},
                {"state": "green_blinking", "duration": 3},
                {"state": "red", "duration": 25},
            ],
        ]
        super().__init__("dummy-id-001", cycles, 0.5)


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
    generator = SignalGeneratorDummy001GreenFirst()
    for tick in range(0, 101):
        print(json.dumps(generator.next()))
