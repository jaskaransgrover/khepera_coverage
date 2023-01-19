# Global Constants Pertaining to the Arena and Robots

# Robot details
ROBOT_RADIUS    = 0.15/2
ROBOT_MAX_SPEED = 0.4       # TODO: verify that this is correct


# Manually tuned arena bounds
ARENA_X_LOWER   = -56.7*2.54*0.01  
ARENA_Y_LOWER   = -37.5*2.54*0.01

ARENA_WIDTH     = 152.5*2.54*0.01  
ARENA_HEIGHT    = 80*2.54*0.01  

print(ARENA_X_LOWER, ARENA_Y_LOWER)

ARENA_X_UPPER   = ARENA_X_LOWER + ARENA_WIDTH
ARENA_Y_UPPER   = ARENA_Y_LOWER + ARENA_HEIGHT
