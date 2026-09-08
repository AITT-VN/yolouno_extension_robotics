# Line-following simulator

Runs the **real** `drivebase.py` and `line_sensor.py` under CPython on a
kinematic model of a two-wheel robot, so the line-following controller can
be exercised without a robot: curves, a sharp corner, a crossing bar, the end
of the line and a T junction.

* `shims.py` - stand-ins for `machine`, `utility`, `setting`, `pcf8574`; puts
  `ticks_ms/us` and `asyncio.sleep_ms` on a virtual clock that also advances
  the physics.
* `world.py` - robot (motor lag, dead band, wheel odometry), polyline track,
  4/5-eye reflectance model (digital bits and 12-bit raw values).
* `linesim.py` - old-vs-new comparison on the demo track at several speeds.
* `t_test.py` - follow to a T bar, turn until the branch, follow the branch.
* `trace.py`, `trace2.py`, `turn_trace.py` - frame-by-frame traces.
* `sweep.py`, `robust2.py` - gain and robot-parameter sweeps (slow).
* `straight_test.py` - the precise `straight()` move with encoders.

    cd tools/linesim
    python3 linesim.py
    python3 t_test.py

Numbers are relative, not absolute: the motor model has no static friction,
so pivot pulses look bigger than on a real robot. A full `linesim.py` run
takes several minutes.
