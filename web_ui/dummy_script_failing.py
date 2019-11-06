import time
import sys

for i in range(3):
    print("script is running,  {}...".format(time.time()) )
    sys.stdout.flush()
    time.sleep(1)
    
exit(2)