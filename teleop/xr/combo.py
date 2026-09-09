"""A deliberate button combination, held, as an operator command.

The dashboard's start button lives on the host. The operator lives in a
headset, usually across the room from it, and until now every lap of

    start -> align -> follow -> fault -> acknowledge -> start -> align

needed someone at the keyboard for two of those steps. That is tolerable for
a demo and wrong for a test session, where the interesting run is the tenth
recovery rather than the first, and where taking the headset off to reach a
keyboard is itself a way to lose the state you were trying to reproduce.

So the two primary face buttons -- X on the left pad, A on the right -- mean
"start", and when a stop is latched they mean "acknowledge, then start".

It was meant to be all four. It cannot be: the Quest app binds Y + B to the
emergency stop (TeleopSession.EstopBinding), and that fires on the device and
travels as its own `estop` message, so any gesture containing Y and B is an
e-stop before it is anything else. X + A is the complement, and the one pair
on these pads with no other meaning while the session is idle.

Three decisions worth keeping:

  * Two hands, and Y+B explicitly *up*. One button that arms a humanoid is a
    button someone will lean on; two, one per hand, cannot be pressed while
    the operator is holding anything. Requiring Y and B released is not
    decoration -- it means an operator mashing all four gets the e-stop and
    unambiguously not a start, which is the right way round for that race.
    (X+A also waives the align position check, but only during alignment, and
    this gesture is read only while idle.)

  * Held, not tapped. A tap is what a dropped controller produces; a hold is a
    decision. `hold_s` is the whole safety margin here, so it is a constant
    with a reason rather than a magic number: long enough that a knock cannot
    reach it, short enough that it does not feel broken.

  * Edge-triggered, and re-armed only by release. The caller runs at 30 Hz. A
    detector that returned True while held would queue thirty starts a second
    and, worse, would re-start immediately after any stop the operator
    triggered while still holding the buttons.

The detector is deliberately ignorant of what it starts: it reports that the
gesture happened, and the caller decides what that means in the current state.
That is what lets the same gesture mean "start" and "acknowledge, then start"
without this file knowing anything about safety.
"""

from __future__ import annotations

from typing import Optional


def start_gesture_held(frame) -> bool:
    """X + A held, with Y and B released.

    The negative half matters as much as the positive one: Y+B is the device's
    emergency stop, so an operator pressing all four must get a stop and never
    a start. Testing all four fields rather than two makes that explicit here
    instead of leaving it to whichever message happens to arrive first.

    Reads the device-neutral frame, so this works on the XrLink path and the
    Vuer path alike (see teleop/xr/native_source.py and vuer_source.py).
    """
    return bool(frame.left_ctrl_aButton and frame.right_ctrl_aButton
                and not frame.left_ctrl_bButton
                and not frame.right_ctrl_bButton)


class HoldCombo:
    """Fires once when `held` has been continuously true for `hold_s`.

    Re-arms only when `held` goes false again, so one press produces exactly
    one command no matter how long it lasts or how fast the caller polls.
    """

    def __init__(self, hold_s: float = 0.75):
        self.hold_s = float(hold_s)
        self._since: Optional[float] = None
        self._fired = False

    def reset(self) -> None:
        """Forget any hold in progress.

        Used when the meaning of the gesture changes underfoot -- e.g. the
        session started by some other route -- so a hold begun under the old
        meaning cannot complete under the new one.
        """
        self._since = None
        self._fired = False

    def update(self, now: float, held: bool) -> bool:
        """Advance the detector. True exactly once per completed hold."""
        if not held:
            self._since = None
            self._fired = False
            return False
        if self._since is None:
            self._since = now
        if self._fired:
            return False
        if now - self._since >= self.hold_s:
            self._fired = True
            return True
        return False

    def progress(self, now: float) -> float:
        """0.0-1.0 through the current hold; 0.0 when idle or already fired.

        Not used to decide anything -- it exists so the operator can be shown
        that the headset noticed, which is the difference between a gesture
        that feels broken and one that feels deliberate.
        """
        if self._since is None or self._fired:
            return 0.0
        if self.hold_s <= 0:
            return 1.0
        return min(1.0, (now - self._since) / self.hold_s)
