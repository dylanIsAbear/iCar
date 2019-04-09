import Serial
import threading

serial = Serial.Command(0, 0, '')
# Command object

thread_cmd = threading.Thread(target=serial.send_command())
# Keep sending command ignoring the command

thread_cmd.start()
# Send command


