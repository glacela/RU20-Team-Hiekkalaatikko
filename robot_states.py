MAX_SPEED = 50

GOING_FORWARD = {
    speed_left = MAX_SPEED,
    speed_right = MAX_SPEED
}
TURN_LEFT_BROAD = {
    speed_left = 0,
    speed_right = MAX_SPEED
}
TURN_LEFT_STATIONARY = {
    speed_left = -MAX_SPEED,
    speed_right = MAX_SPEED
},
TURN_RIGHT_BROAD = {
    speed_left = MAX_SPEED,
    speed_right = 0
},
TURN_RIGHT_STATIONARY = {
    speed_left = MAX_SPEED,
    speed_right = -MAX_SPEED
}

class Transition:
    
    __init__(self, from, to):
        self.from = from
        self.to =  to

    get_from(self):
        return self.from

    get_to(self):
        return self.to


