from micropython import const

# motor ports
ALL = const(63)
M1 = const(1)
M2 = const(2)
M3 = const(4)
M4 = const(8)
E1 = const(16)
E2 = const(32)

STEPPER1 = const(0)
STEPPER2 = const(1)

DIR_CW = const(1)
DIR_CCW = const(-1)

S1 = const(0)
S2 = const(1)
S3 = const(2)
S4 = const(3)

# drivetrain mode
MODE_2WD = const(0)
MODE_4WD = const(1)
MODE_MECANUM = const(2)

# stop method
STOP = const(0)
BRAKE = const(1)

# move unit
SECOND = const(0)
DEGREE = const(1)
CM = const(2)
INCH = const(3)

# direction
DIR_FW = const(0) # forward
DIR_RF = const(1) # right forward
DIR_R = const(2) # turn right
DIR_RB = const(3) # right backward
DIR_BW = const(4) # backward
DIR_LB = const(5) # left backward
DIR_L = const(6) # turn left
DIR_LF = const(7) # left forward
DIR_SL = const(8) # side left
DIR_SR = const(9) # side right

# gamepad buttons
BTN_UP = 'U'
BTN_DOWN = 'D'
BTN_LEFT = 'L'
BTN_RIGHT = 'R'

BTN_SQUARE = 'SQ'
BTN_TRIANGLE = 'TR'
BTN_CROSS = 'CR'
BTN_CIRCLE = 'CI'

BTN_L1 = 'L1'
BTN_R1 = 'R1'
BTN_L2 = 'L2'
BTN_R2 = 'R2'

BTN_M1 = 'M1'
BTN_M2 = 'M2'
BTN_THUMBL = 'THUMBL'
BTN_THUMBR = 'THUMBR'

AL = 'AL'
ALX = 'ALX'
ALY = 'ALY'
AL_DIR = 'AL_DIR'
AL_DISTANCE = 'AL_DISTANCE'
AR = 'AR'
ARX = 'ARX'
ARY = 'ARY'
AR_DIR = 'AR_DIR'
AR_DISTANCE = 'AR_DISTANCE'

# line sensor status
LINE_LEFT3 = const(-3)
LINE_LEFT2 = const(-2)
LINE_LEFT = const(-1)
LINE_CENTER = const(0)
LINE_RIGHT = const(1)
LINE_RIGHT2 = const(2)
LINE_RIGHT3 = const(3)
LINE_CROSS = const(4)
LINE_END = const(5)