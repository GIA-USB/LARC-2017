import os
import errno
import sys
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

req = open(reqPipe, "w")
req.write(sys.argv[1])
req.close()
while True:
    req = open(reqPipe, "w")
    req.write(sys.argv[1])
    req.close()
    print("Waiting...")
    resp = open(resPipe,'r')
    value = resp.read()
    resp.close()
    while len(value)==0:
        time.sleep(0.2)
        resp = open(resPipe,'r') 
        value = resp.read()
        resp.close()
    print(value)
    resp.close()
    resp = open(resPipe,'w')
    resp.write("")
    resp.close()
    
