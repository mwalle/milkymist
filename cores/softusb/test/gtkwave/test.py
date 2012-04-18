#!/usr/bin/env python
import sys

i = 1
f = open('/tmp/test.txt', 'w')
line = sys.stdin.readline()
while True:
	f.write(str(i))
	f.write(' ')
	f.write(line)
	i+=1
	print line,
	line = sys.stdin.readline()
