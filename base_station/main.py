from enum import Enum

from manual_console import ManualConsole
from autonomous_console import AutonomousConsole

HOSTNAME = "172.20.10.7"
MANUAL_PORT = 5500
AUTO_PORT = 5501


class Mode(Enum):
    NULL = 0
    AUTO = 1
    MANUAL = 2

class RemoteConsole:
    def __init__(self, hostname, port_auto, port_manual):
        self.hostname = hostname
        self.port_auto = port_auto
        self.port_manual = port_manual

        self.mode = Mode.NULL

    def start(self):
        while True:
            if self.mode == Mode.NULL:
                self.mode = self._mode_menu()
                continue

            elif self.mode == Mode.AUTO:
                a = AutonomousConsole(self.hostname, self.port_auto)
                a.run()
                self.mode = Mode.NULL
                continue

            elif self.mode == Mode.MANUAL:
                m = ManualConsole(self.hostname, self.port_manual)
                m.run()
                self.mode = Mode.NULL
                continue
    
    def _mode_menu(self):
        print("\n\nROBOT CONTROLLER\n")
        print("Select a mode: ")
        for m in Mode:
            if m.value == 0:
                continue
            print(f"{m.value}. {m.name}")
        print("0. Exit")
        print()

        while True:
            try:
                self.mode = Mode(int(input("> ")))
                if self.mode == Mode.NULL:
                    exit(0)
                return self.mode
            
            except ValueError:
                continue

if __name__ == "__main__":
    r = RemoteConsole(HOSTNAME, AUTO_PORT, MANUAL_PORT)
    r.start()

# commands
# rot <degrees>
# mv <distance> 

# spd <lin_speed> <angl_speed>
