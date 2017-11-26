#!/bin/bash

arduino_bin=$(readlink -f `which arduino`)
arduino_path=$(dirname "$arduino_bin")

if [ -d "$arduino_path" ]; then
	echo "$arduino_path"
else
	exit 1
fi
