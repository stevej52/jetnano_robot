# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
"""One copy of each bringup launch per machine.

Two copies of robot.launch.py on the same computer double every node: two EKFs
fusing the same IMU, two drivers opening the lidar's serial port (both die), two
writers on the PCA9685's I2C bus. It happened on 2026-09-23 from a command that
was sent twice over SSH, and nothing complained except the lidar.

only_one(name, actions) wraps a launch's actions so that they start only if this
process can take an exclusive lock on /tmp/<name>.launch.lock. The lock is a
flock, held for as long as the launch process lives and released by the kernel
when it exits however it exits, so there is no stale lock to clean up after a
crash. A second launch prints who holds the lock and shuts itself down without
starting anything.

    from jetnano_bringup.launch_lock import only_one

    return LaunchDescription([
        DeclareLaunchArgument(...),
        only_one('jetnano_drive', [
            Node(...),
            Node(...),
        ]),
    ])

Each launch file has its own lock, so the bench habit of running sensors,
odometry and description as separate launches still works; what is refused is a
second copy of the same launch, including a standalone sensors.launch.py while
robot.launch.py (which includes it) is up.
"""

import fcntl
import os

from launch.actions import LogInfo, OpaqueFunction, Shutdown

_HELD = {}  # name -> open file descriptor, kept for the life of the process


def _lock_path(name):
    return os.path.join('/tmp', f'{name}.launch.lock')


def _try_lock(name):
    """Return None if the lock is now ours, else a description of the holder."""
    path = _lock_path(name)
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
    try:
        os.chmod(path, 0o666)  # any user on the machine may hold or read it
    except OSError:
        pass
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        holder = os.read(fd, 64).decode(errors='replace').strip() or 'unknown pid'
        os.close(fd)
        return holder
    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode())
    _HELD[name] = fd
    return None


def only_one(name, actions):
    """Start `actions` only if no other launch on this machine holds `name`."""
    def _guard(context):
        holder = _try_lock(name)
        if holder is None:
            return actions
        return [
            LogInfo(msg=f'{name}: already running on this machine (pid {holder}); '
                        f'not starting a second copy. Lock: {_lock_path(name)}'),
            Shutdown(reason=f'{name} is already running (pid {holder})'),
        ]
    return OpaqueFunction(function=_guard)
