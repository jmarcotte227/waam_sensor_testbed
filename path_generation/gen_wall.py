import numpy as np

# geometry parameters
WALL_LEN =      70 # mm
LAYER_HEIGHT =  4   # mm
NUM_LAYERS =    2

# velocities
JUMP_VEL =      100 # mm/s
WELD_VEL =      3  # mm/s

motion_commands = np.zeros((NUM_LAYERS*2, 4))
direction = True
# generate points starting from bottom front
for i in range(NUM_LAYERS):
    if direction:
        # start point end
        motion_commands[i*2, 0] = 0                 # x
        motion_commands[i*2, 1] = 0                 # y
        motion_commands[i*2, 2] = i*LAYER_HEIGHT    # z

        # opposite end
        motion_commands[i*2+1, 0] = WALL_LEN          # x
        motion_commands[i*2+1, 1] = 0                 # y
        motion_commands[i*2+1, 2] = i*LAYER_HEIGHT    # z
    else:
        # opposite end
        motion_commands[i*2, 0] = WALL_LEN          # x
        motion_commands[i*2, 1] = 0                 # y
        motion_commands[i*2, 2] = i*LAYER_HEIGHT    # z

        # start point end
        motion_commands[i*2+1, 0] = 0                 # x
        motion_commands[i*2+1, 1] = 0                 # y
        motion_commands[i*2+1, 2] = i*LAYER_HEIGHT    # z

    direction = not direction

    # set velocities
    motion_commands[i*2, 3] = JUMP_VEL
    motion_commands[i*2+1, 3] = WELD_VEL

# save path
np.savetxt('paths/wall.csv', motion_commands, delimiter=',')
