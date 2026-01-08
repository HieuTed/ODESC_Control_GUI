import time
from ODrive_controller import ODriveThread

def print_help():
    print("""
Available commands:
  O / o           : Set offset (current position = zero reference)
  C / c           : Enter closed-loop control
  I / i           : Set IDLE (disable torque)
  Kp[value]       : Set proportional gain (e.g. Kp5.0)
  Kd[value]       : Set derivative gain   (e.g. Kd0.2)
  P[value]        : Set position target in degree (vel = acc = 0)
  q               : Quit program
""")

def main():
    ctrl = ODriveThread()
    ctrl.start()

    print("Simple ODrive Console Controller")
    print_help()

    try:
        while True:
            cmd = input(">> ").strip()

            if not cmd:
                continue

            # Quit
            if cmd.lower() == "q":
                print("Exiting...")
                break

            # Offset
            elif cmd.lower() == "o":
                ctrl.set_offset()
                print("Offset set.")

            # Closed-loop
            elif cmd.lower() == "c":
                if ctrl.isOffset:
                    ctrl.enter_closed_loop()
                else:
                    print("Offset not set yet.")

            # IDLE
            elif cmd.lower() == "i":
                ctrl.return_IDLE()
                print("Set to IDLE.")

            # Set Kp
            elif cmd.startswith("Kp") or cmd.startswith("kp"):
                try:
                    value = float(cmd[2:])
                    ctrl.Kp = value
                    print(f"Kp set to {value}")
                except ValueError:
                    print("Invalid Kp value.")

            # Set Kd
            elif cmd.startswith("Kd") or cmd.startswith("kd"):
                try:
                    value = float(cmd[2:])
                    ctrl.Kd = value
                    print(f"Kd set to {value}")
                except ValueError:
                    print("Invalid Kd value.")

            # Position command
            elif cmd.startswith("P") or cmd.startswith("p"):
                try:
                    pos = float(cmd[1:])
                    ctrl.update_target(pos, 0.0, 0.0)
                    print(f"Position target set to {pos} deg")
                except ValueError:
                    print("Invalid position value.")

            else:
                print("Unknown command.")
                print_help()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt.")

    finally:
        ctrl.stop()
        ctrl.join()
        print("Controller stopped.")

if __name__ == "__main__":
    main()
