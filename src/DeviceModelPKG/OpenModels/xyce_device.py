"""
Type stubs for the ``xyce_device`` pybind11 embedded module.

This module is injected by the Xyce simulator into the Python environment of
every device script.  It provides the classes and helpers needed to describe
the electrical interface of a user-defined Python device.

Typical usage::

    from xyce_device import *          # makes all names available globally

    class MyDevice:
        def __init__(self):
            self.vin  = Input(0)
            self.vout = VoltageOutput(1)

        def update(self, time: float) -> None:
            self.vout.set_value(self.vin.get_v() * 2.0)
"""

from __future__ import annotations

from typing import Callable, Final, List, Tuple

# ---------------------------------------------------------------------------
# Event constants used with Input.trigger()
# ---------------------------------------------------------------------------

RISING: Final[int] = 1
"""Trigger fires when the signal crosses the threshold from below."""

FALLING: Final[int] = -1
"""Trigger fires when the signal crosses the threshold from above."""

BOTH: Final[int] = 0
"""Trigger fires on both rising and falling crossings."""


# ---------------------------------------------------------------------------
# Input node
# ---------------------------------------------------------------------------

class Input:
    """Represents a read-only voltage input node of the device.

    The node index corresponds to the position in the netlist pin list
    (0-based).  Voltage values are updated by Xyce before every call to
    ``update()``.

    Args:
        index: Pin index (0-based) as declared in the netlist.

    Example::

        vin = Input(0)
        print(vin.get_v())   # → current node voltage in Volts
    """

    def __init__(self, index: int) -> None: ...

    def get_v(self) -> float:
        """Return the current voltage at this node in Volts."""
        ...

    def trigger(
        self,
        callback: Callable[[], None],
        event: int,
        val: float,
    ) -> None:
        """Register a threshold-crossing callback on this input.

        The callback is invoked (with no arguments) during ``update()``
        whenever the voltage crosses *val* in the specified direction.

        Args:
            callback: Zero-argument callable to invoke when the event fires.
            event:    Direction of the crossing.
                      ``+1`` / ``RISING``  – low-to-high crossing.
                      ``-1`` / ``FALLING`` – high-to-low crossing.
                      ``0``  / ``BOTH``    – either direction.
            val:      Threshold voltage in Volts.

        Example::

            vin.trigger(callback=on_rising,  event=RISING,  val=1.5)
            vin.trigger(callback=on_falling, event=FALLING, val=1.5)
        """
        ...


# ---------------------------------------------------------------------------
# Output nodes
# ---------------------------------------------------------------------------

class ResistorOutput:
    """A digital/PWM output modelled as a pull-up/pull-down resistor pair.

    The output drives a logic 0 or logic 1 by connecting a resistor *r*
    between the output node and either *vlow* (state 0) or *vhigh*
    (state 1).

    Args:
        index: Pin index (0-based) of the output node.
        r:     Series resistance in Ohms.
        vhigh: :class:`Input` node used as the high-side supply.
        vlow:  :class:`Input` node used as the low-side supply / ground.

    Example::

        vdd = Input(0)
        vss = Input(1)
        out = ResistorOutput(2, 1000.0, vdd, vss)
        out.set_state(1)   # pull output toward VDD through 1 kΩ
    """

    def __init__(
        self,
        index: int,
        r: float,
        vhigh: Input,
        vlow: Input,
    ) -> None: ...

    def set_state(self, state: int) -> None:
        """Drive the output to a new digital state.

        A short (≈100 ps) transition ramp is inserted automatically so that
        Xyce can resolve the edge accurately.

        Args:
            state: ``1`` to pull toward *vhigh*, ``0`` to pull toward *vlow*.
        """
        ...

    def get_state(self, current_time: float) -> int:
        """Return the logical output state at *current_time*.

        Takes PWM duty cycle and pattern schedules into account.

        Args:
            current_time: Simulation time in seconds.

        Returns:
            ``1`` if the output is high, ``0`` if low.
        """
        ...

    def set_pwm(self, duty: float, period: float) -> None:
        """Drive the output with a PWM signal starting at the current time.

        The duty cycle and period are applied immediately.  Call
        :meth:`set_state` to stop PWM mode.

        Args:
            duty:   Duty cycle in the range ``[0.0, 1.0]``.
            period: PWM period in seconds (must be > 0).

        Example::

            out.set_pwm(duty=0.5, period=1e-6)   # 50 % at 1 MHz
        """
        ...

    def pattern(self, arg: str) -> None:
        """Drive the output according to a time-annotated pattern string.

        The pattern string is a comma-separated list of ``dt=<value>``,
        ``t=<value>``, and integer state (``0`` / ``1``) tokens.  Repeat
        groups ``[seq]*N`` are expanded automatically.  Xyce value suffixes
        (``u``, ``n``, ``p``, ``m``, ``k``, ``Meg``) are supported.

        Args:
            arg: Pattern descriptor string.

        Example::

            # Hold 0, then high for 1 µs, low for 3.4 µs – repeat 4 times
            out.pattern("0, dt=5u, [1, dt=1u, 0, dt=3.4u]*4, 0")
        """
        ...

    def get_i(self) -> float:
        """Return the current flowing through the output resistor in Amperes."""
        ...


class VoltageOutput:
    """An ideal voltage source output node.

    The node is driven to the value set via :meth:`set_value` or animated
    with :meth:`transition_to`.

    Args:
        index: Pin index (0-based) of the output node.

    Example::

        vout = VoltageOutput(1)
        vout.set_value(3.3)          # immediately 3.3 V
        vout.transition_to(0.0, 1e-9)  # ramp to 0 V over 1 ns
    """

    def __init__(self, index: int) -> None: ...

    def set_value(self, value: float) -> None:
        """Set the output voltage instantly.

        Args:
            value: Target voltage in Volts.
        """
        ...

    def transition_to(self, v: float, dt: float) -> None:
        """Linearly ramp the output voltage to *v* over *dt* seconds.

        Appropriate breakpoints are registered with Xyce automatically.

        Args:
            v:  Target voltage in Volts.
            dt: Ramp duration in seconds.  Must be > 0.
        """
        ...


class CurrentOutput:
    """An ideal current source output node.

    The injected current is set via :meth:`set_value` or animated with
    :meth:`transition_to`.

    Args:
        index: Pin index (0-based) of the output node.

    Example::

        iout = CurrentOutput(2)
        iout.set_value(1e-3)           # 1 mA immediately
        iout.transition_to(2e-3, 5e-9)  # ramp to 2 mA over 5 ns
    """

    def __init__(self, index: int) -> None: ...

    def set_value(self, i: float) -> None:
        """Set the output current instantly.

        Args:
            i: Target current in Amperes.
        """
        ...

    def transition_to(self, i: float, dt: float) -> None:
        """Linearly ramp the output current to *i* over *dt* seconds.

        Appropriate breakpoints are registered with Xyce automatically.

        Args:
            i:  Target current in Amperes.
            dt: Ramp duration in seconds.  Must be > 0.
        """
        ...


# ---------------------------------------------------------------------------
# Device helper (rarely used directly from device scripts)
# ---------------------------------------------------------------------------

class Device:
    """Low-level handle to the active Xyce device instance.

    In most device scripts you do **not** need to instantiate this class
    directly.  Use the module-level :func:`add_breakpoint` helper instead.
    """

    def __init__(self) -> None: ...

    def add_breakpoint(self, t: float) -> None:
        """Ask Xyce to stop at simulation time *t*.

        Same effect as the module-level :func:`add_breakpoint`.

        Args:
            t: Absolute simulation time in seconds.
        """
        ...

    def delay(
        self,
        callback: Callable[[], None],
        time_delay_s: float,
    ) -> None:
        """Schedule *callback* to be called after *time_delay_s* seconds.

        The callback is executed during the simulation step that first
        reaches or exceeds ``current_time + time_delay_s``.

        Args:
            callback:     Zero-argument callable.
            time_delay_s: Delay in seconds relative to the current
                          simulation time.  Clamped to 0 if negative.
        """
        ...

    delay_callbacks: List[Tuple[float, Callable[[], None]]]
    """Read-only list of pending ``(fire_time, callback)`` tuples."""


# ---------------------------------------------------------------------------
# Module-level helpers (injected into device script globals by Xyce)
# ---------------------------------------------------------------------------

def add_breakpoint(t: float) -> None:
    """Ask Xyce to guarantee a solver step at absolute time *t*.

    Use this to ensure that state transitions (e.g. PWM edges, digital
    toggles) are captured accurately.

    Args:
        t: Absolute simulation time in seconds.

    Example::

        add_breakpoint(self.last_toggle + self.period)
    """
    ...


def set_next_breakpoint(t: float) -> None:
    """Alias for :func:`add_breakpoint` kept for backwards compatibility."""
    ...
