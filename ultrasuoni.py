import RPi.GPIO as GPIO
import time
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO_TRIGGER = 4
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)

ECHOS = {"ECHO_LEFT": 26, "ECHO_RIGHT": 16}
for e in ECHOS:
  GPIO.setup(ECHOS[e], GPIO.IN)

def distance(e):
  new_reading = False
  counter = 0
  GPIO.output(GPIO_TRIGGER, True)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER, False)

  while GPIO.input(e) == 0:
    pass
    counter += 1
    if counter == 5000:
      new_reading = True
      break
  start_time = time.time()

  if new_reading:
    return False

  while GPIO.input(e) == 1:
    pass
  end_time = time.time()

  time_elapsed = end_time - start_time
  dist = (time_elapsed * 34300) / 2
  return dist

def run_distance():
  sensors = []
  for e in ECHOS:
    dist = distance(ECHOS[e])
    sensors.append("{}: {}".format(e, dist))
    time.sleep(1)
  return sensors  

if __name__ == "__main__":
  result = []
  result = run_distance()
  print(result[0]+'\n')
  print(result[1])
