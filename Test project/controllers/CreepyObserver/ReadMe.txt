CreepyObserver.py

This controls an external robot to check that the comms work.
In the test program this robot prints whenever it hears of a new block from the other robot.
It uses the same comms encoder/decoder as the other robot.

Comms.py
Was an attempt to move comms functions into an external module
However there was a problem with the struct package so for ease this file is not used