# use raw socket because there are few standard libs in LenchScripterMod

from actiondef import Action, KEYS

import socket
import marshal

engines = [
    Besiege.GetBlock("SPINNING 4"),     # front left
    Besiege.GetBlock("SPINNING 3"),     # front right
    Besiege.GetBlock("SPINNING 1"),     # rear left
    Besiege.GetBlock("SPINNING 2"),     # rear right
]
center = Besiege.GetBlock("STARTING BLOCK 1")

Besiege.UseRadians()
HOST = 'localhost'
PORT=2743

def fmt_numlist(v):
    return '[{}]'.format(', '.join(map('{:.2f}'.format, v)))

class RPCProxy:
    _conn = None
    def __init__(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._conn = s

    def close(self):
        self._conn.close()

    def step(self, position, angle, command):
        # send current state
        Besiege.Watch('position', fmt_numlist(position))
        Besiege.Watch('angle', fmt_numlist(angle))
        data = marshal.dumps([position, angle, command, Time.deltaTime])
        dlen = '{:32}'.format(len(data))
        self._conn.sendall(dlen + data)

        # recv new engine speed setting
        data = ''
        while not data.endswith('@'):
            cur = self._conn.recv(4096)
            if not cur:
                raise RuntimeError('peer shutdown')
            data += cur
        setting = map(float, data[:-1].split(';'))
        assert len(setting) == len(engines)
        Besiege.Watch('engine speed', fmt_numlist(setting))
        for i, j in zip(engines, setting):
            i.SetSliderValue('SPEED', j)


if hasattr(clr, 'drone_controller_proxy'):
    print('close old proxy')
    clr.drone_controller_proxy.close()

controller_proxy = RPCProxy()
clr.drone_controller_proxy = controller_proxy

def Update():
    command = 0
    actions = []
    for key, (name, val)  in zip(KEYS, Action._all_actions()):
        if Input.GetKey(getattr(KeyCode, key)):
            actions.append(name)
            command |= val

    Besiege.Watch('action', ' '.join(actions))

    controller_proxy.step(
        list(map(float, center.Position)),
        list(map(float, center.EulerAngles)),
        command
    )
