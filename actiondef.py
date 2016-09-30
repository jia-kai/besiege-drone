def _next(cur=[0]):
    ret = 1 << cur[0]
    cur[0] += 1
    return ret

class Action(object):
    GO_FRONT = _next()
    GO_BACK = _next()
    GO_LEFT = _next()
    GO_RIGHT = _next()
    YAW_LEFT = _next()
    YAW_RIGHT = _next()
    HIGHER = _next()
    LOWER = _next()

    @classmethod
    def _all_actions(cls):
        ret = [(k, v) for k, v in cls.__dict__.items()
               if not k.startswith('_')]
        ret.sort(key=lambda x: x[1])
        return ret

KEYS = ['I', 'K',   # front/back
        'Comma', 'Period',  # left/right
        'J', 'L',   # yaw
        'U', 'O'    # altitude
        ]
