#!/bin/bash

log_name=$1
cat "$log_name" | grep T1= | grep -v T1=-0.06 | grep -v T1=-20 > data/temp1.txt
cat "$log_name" | grep P= > data/pressure.txt
cat "$log_name" | grep T2= | grep -v T2=-0.06 > data/temp2.txt
cat "$log_name" | grep V= > data/voltage.txt
cat "$log_name" | grep CH= | grep -v CH=0 > data/gps_voltage.txt
truncate -s -2 data/voltage.txt
truncate -s -2 data/gps_voltage.txt
truncate -s -2 data/pressure.txt
truncate -s -2 data/temp1.txt
truncate -s -2 data/temp2.txt
