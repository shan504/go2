#!/usr/bin/env python3
"""Explicit BalanceStand preparation, then the same bounded Sport motion test."""
import time

import rclpy

from drivecheck import ManualDrive, main


class BalancedDrive(ManualDrive):
    DESCRIPTION = ('MANUAL prepared Sport diagnostic, not navigation: BalanceStand '
                   'API1002 acknowledged; vx=0.15m/s, vy=0, yaw=0, 20Hz, 2s; automatic StopMove.')

    def __init__(self):
        super().__init__()
        self.balance_attempted = False

    def prepare_drive(self, spin):
        self.balance_attempted = True
        self.call_request(1002, '{}', spin)
        print('BalanceStand API1002 code=0; waiting 0.5s before the motion guard is read again.', flush=True)
        deadline = time.monotonic()+0.5
        while time.monotonic()<deadline:
            spin(0.02)

    def stop_drive(self, spin):
        if self.balance_attempted and not self.moves and rclpy.ok():
            # Preparing standing state is a control write even if no Move was
            # sent. Stop on rejection, lost acknowledgement or interruption.
            self.publish_request(1003)
            deadline = time.monotonic()+0.2
            while time.monotonic()<deadline:
                spin(0.02)
            print('StopMove API1003 sent after preparation; no Move was sent.', flush=True)
        else:
            super().stop_drive(spin)

    def summary(self):
        lines = [super().summary()]
        replies = [code for key, code in self.replied.items() if key[1] == 1002]
        lines.append(f'BalanceStand: attempted={self.balance_attempted} matched_response_codes={replies}')
        return '\n'.join(lines)


if __name__ == '__main__':
    raise SystemExit(main(BalancedDrive))
