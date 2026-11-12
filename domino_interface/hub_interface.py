"""Callable SPIKE Prime selector for Pybricks MicroPython."""
from pybricks.hubs import PrimeHub
from pybricks.parameters import Button
from pybricks.tools import StopWatch, wait

# Coordinates are (row, column), indexed by the logical piece number.
PIECE_LEDS = ((0, 0), (0, 1), (0, 2), (0, 3),
              (0, 4), (1, 4), (2, 4), (3, 4),
              (4, 4), (4, 3), (4, 2), (4, 1),
              (4, 0), (3, 0), (2, 0), (1, 0))
DIM, GROUP, FLIPPED, CURSOR, READY_LIGHT = 6, 24, 95, 100, 100
DOUBLE_CLICK_MS, DEBOUNCE_MS, POLL_INTERVAL_MS, BLINK_INTERVAL_MS = 350, 20, 10, 250
EDIT, READY, RUNNING = 'EDIT', 'READY', 'RUNNING'
LEFT, RIGHT, CENTER = 'left', 'right', 'center'


class DebouncedButton:
    def __init__(self):
        self.raw = False
        self.stable = False
        self.changed_at = 0

    def update(self, pressed, now):
        if pressed != self.raw:
            self.raw = pressed
            self.changed_at = now
        if self.raw != self.stable and now - self.changed_at >= DEBOUNCE_MS:
            self.stable = self.raw
            return 'press' if self.stable else 'release'
        return None


class DominoInterface:
    def __init__(self):
        self.state = EDIT
        self.selected = 0
        self.flipped = [False] * 16
        self.buttons = {name: DebouncedButton() for name in (LEFT, RIGHT, CENTER)}
        self.pending_time = None
        self.pending_piece = None
        self.second_press = False
        self.snapshot = None

    @property
    def flip_mask(self):
        return sum((1 << i) for i in range(16) if self.flipped[i])

    def update(self, pressed, now):
        """Advance once. Return a snapshot exactly once on transition to RUNNING."""
        if self.state == RUNNING:
            return None
        edges = {name: self.buttons[name].update(name in pressed, now)
                 for name in (LEFT, RIGHT, CENTER)}
        center = edges[CENTER]

        if self.state == EDIT:
            # A second press that began within the window owns the pending click,
            # even if it is held beyond the deadline.
            if (self.pending_time is not None and not self.second_press
                    and now - self.pending_time >= DOUBLE_CLICK_MS
                    and not (self.buttons[CENTER].raw and
                             self.buttons[CENTER].changed_at - self.pending_time < DOUBLE_CLICK_MS)):
                self.flipped[self.pending_piece] = not self.flipped[self.pending_piece]
                self.pending_time = None
                self.pending_piece = None
            if edges[LEFT] == 'press':
                self.selected = (self.selected - 1) % 16
            if edges[RIGHT] == 'press':
                self.selected = (self.selected + 1) % 16
            if center == 'press':
                if self.pending_time is not None:
                    self.second_press = True
            elif center == 'release':
                if self.second_press:
                    self.pending_time = None
                    self.pending_piece = None
                    self.second_press = False
                    self.state = READY
                else:
                    self.pending_time = now
                    self.pending_piece = self.selected
        elif center == 'release':
            # READY is entered only on a debounced release. This release can
            # never also start the robot; a new press and release are required.
            self.state = RUNNING
            self.snapshot = tuple(self.flipped)
            return self.snapshot
        return None

    def render(self, now):
        matrix = [[0] * 5 for _ in range(5)]
        for i, (row, col) in enumerate(PIECE_LEDS):
            if self.state == EDIT and i == self.selected:
                level = CURSOR if (now // BLINK_INTERVAL_MS) % 2 == 0 else 0
            elif self.flipped[i]:
                level = FLIPPED
            elif self.state == EDIT and i // 4 == self.selected // 4:
                level = GROUP
            else:
                level = DIM
            matrix[row][col] = level
        if self.state == READY:
            matrix[2][2] = READY_LIGHT
        return matrix


def interface(hub=None):
    """Return 16 integers: 1 for rotated, 0 for unrotated.

    Wait for editing and the READY start click. Pass an existing PrimeHub to
    share it with your robot program. Bluetooth becomes the stop button.
    """
    if hub is None:
        hub = PrimeHub()
    hub.system.set_stop_button(Button.BLUETOOTH)
    watch = StopWatch()
    selector = DominoInterface()
    previous = None
    while True:
        buttons = hub.buttons.pressed()
        pressed = set()
        if Button.LEFT in buttons:
            pressed.add(LEFT)
        if Button.RIGHT in buttons:
            pressed.add(RIGHT)
        if Button.CENTER in buttons:
            pressed.add(CENTER)
        now = watch.time()
        snapshot = selector.update(pressed, now)
        if snapshot is not None:
            return [1 if rotated else 0 for rotated in snapshot]
        matrix = selector.render(now)
        if matrix != previous:
            for row in range(5):
                for col in range(5):
                    if previous is None or matrix[row][col] != previous[row][col]:
                        hub.display.pixel(row, col, matrix[row][col])
            previous = matrix
        wait(POLL_INTERVAL_MS)
