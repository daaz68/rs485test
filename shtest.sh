#!/bin/bash
echo "Sending data to /dev/ttyO5 (UART5 w/ RS485 cap)"
for COUNT in 1 2 3 4 5 ; do
	echo "String test ${COUNT}" > /dev/ttyO5
done
