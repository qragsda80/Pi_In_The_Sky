# Joystick control for drone to fly

int motor1pin = ()
int motor2pin = ()
int motor3pin = ()
int motor4pin = ()
int defaultSpeed = ()

LR = ()
UD = ()
# make analog (or whatever) values into voltage values or map the values to how the ramge motors operate

motor1speed = defaultSpeed + (UD - LR)
motor2speed = defaultSpeed + (UD + LR)
motor3speed = defaultSpeed + (UD + LR)
motor4speed = defaultSpeed + (UD - LR)

# make code to send speeds to motors



