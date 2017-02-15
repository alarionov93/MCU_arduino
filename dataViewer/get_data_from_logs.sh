cat mdu.debug.log | grep T1= | grep -v T1=-0.06 | grep -v T1=-20 > data/temp1.txt
cat mdu.debug.log | grep P= > data/pressure.txt
cat mdu.debug.log | grep T2= | grep -v T2=-0.06 > data/temp2.txt
cat mdu.debug.log | grep V= > data/voltage.txt
truncate -s -2 data/voltage.txt
truncate -s -2 data/pressure.txt
truncate -s -2 data/temp1.txt
truncate -s -2 data/temp2.txt

