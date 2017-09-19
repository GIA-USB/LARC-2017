import os
import errno

reqPipe = 'requestPipe'
resPipe = 'responsePipe'
"""
try:
    os.mkfifo(reqPipe)
    os.mkfifo(resPipe)
except OSError as oe: 
    if oe.errno != errno.EEXIST:
        raise
"""
import subprocess, signal
import os
import sys
import time
import termios

process = subprocess.Popen(['minimu9-ahrs --output euler'],
			    stdout=subprocess.PIPE,
			    shell=True)
def readAltimu():
    out = process.stdout.readline()
    if out != '':
        return str(out)
    return ""

old = ""
resp = open(resPipe,'w')
resp.close()
req = open(reqPipe, 'w')
req.close()
while True:
    req = open(reqPipe, 'r')
    a = readAltimu()
    a = str(a)
    a = a.split()
    a = a[0] + " " + a[len(a)-2]
    data = req.read()
    if process.poll() != None:
        break
    if len(data) != 0 and data == "ok":
        req.close()
        req = open(reqPipe, 'w')
        print('Enviando: "{0}"'.format(a))
        resp = open(resPipe,'w')
        resp.write(a)
        resp.close()
        req.write("")
    req.close()
