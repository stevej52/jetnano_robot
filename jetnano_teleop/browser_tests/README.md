# Browser tests for the driving page

Two end-to-end checks of `web_teleop`'s page, run in a real headless browser
(Playwright with Google Chrome) against the running robot, from the host PC:

    python3 -m venv ~/venv-tools && ~/venv-tools/bin/pip install playwright
    ~/venv-tools/bin/python stick_test.py          # the joystick: drag, spring back, rim = full deflection
    ~/venv-tools/bin/python guard_switch_test.py   # the obstacle guard's ON/OFF switch

They drive the page, not the wheels - run them with the drive rail off or
the robot on blocks. `stick_test.py` found the keep-alive bug (unread POST
bodies) on 2026-09-23.
