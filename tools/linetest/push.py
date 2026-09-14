# push files to the board through exec chunks (mpremote cp trips the watchdog on this board)
import sys, subprocess, os
import glob
PORT = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].startswith('/dev/') else (glob.glob('/dev/cu.usbmodem*') + glob.glob('/dev/ttyACM*') + ['/dev/cu.usbmodem1101'])[0]
MP = [sys.executable, '-m', 'mpremote', 'resume', 'connect', PORT]
env = dict(os.environ, PYTHONPATH=os.path.expanduser('~/esp/micropython/tools/mpremote'))
def ex(code):
    r = subprocess.run(MP + ['exec', code], capture_output=True, text=True, env=env, timeout=120)
    if r.returncode: print('ERR', r.stdout[-300:], r.stderr[-300:])
    return r.stdout.strip()
for path in [a for a in sys.argv[1:] if not a.startswith('/dev/')]:
    data = open(path, 'rb').read(); name = '/' + os.path.basename(path)
    ex("f=open(%r,'wb'); f.close()" % name)
    for i in range(0, len(data), 6000):
        chunk = data[i:i+6000]
        ex("f=open(%r,'ab'); f.write(%r); f.close()" % (name, chunk))
    print(name, 'local', len(data), 'board', ex("import os; print(os.stat(%r)[6])" % name))
