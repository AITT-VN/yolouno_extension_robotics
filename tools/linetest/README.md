# Hardware line-following tests

Scripts used to validate line following on a Yolo UNO robot from the
command line (`mpremote`). The robot must be untethered while it drives, so
the driving tests are one-shot `main.py` programs:

1. copy the program to the board as `main.py` and create the flag file
   `/run_once`;
2. unplug USB, put the robot on the line, press RESET; the program waits 5 s,
   drives, and writes `/line_log.txt` (kept in RAM until the end: flash
   writes and USB writes both stall the control loop);
3. plug USB back in and read `/line_log.txt`. The flag is consumed at start,
   so the board does not drive again when it is plugged in.

* `main_linetest.py` - follow to a crossing, then follow for 1.5 s.
* `main_crosstest.py` - follow to a crossing, turn right until the next
  line, follow the branch for 2 s.
* `line5_check.py` - 60 readings of the 5-channel array standing still.
* `line5_sweep.py` - readings while the array is swept across the line
  (learns and saves the analog calibration).
* `line5_slide.py` - per-eye contrast while the robot is slid across the line.
* `loop_rate.py` - control-loop timing on the board.
* `push.py` - copies files through `exec` chunks when `mpremote cp` keeps
  resetting the board.

Motor ports, wheel size, speeds and the sensor-to-axle offset at the top of
the `main_*.py` files belong to the robot they were last used on; adjust them.

    python3 -m mpremote resume connect /dev/cu.usbmodem101 cp main_crosstest.py :main.py \
        + exec "open('/run_once','w').write('1')"
    # ... drive ...
    python3 -m mpremote resume connect /dev/cu.usbmodem101 cp :line_log.txt run.txt
