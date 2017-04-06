
import sys

if __name__=='__main__':
    logfile = open('log.txt', 'w+')
    for s in sys.argv:
        logfile.write(s)