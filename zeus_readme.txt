
========================================
building step(see cgminer's readme):

in building configration step, you just need to run:

autoreconf -fvi

CFLAGS="-O2" ./configure

As the configure.ac is changed to support the ltc miner now.




=================================
execution  example:

cgminer.exe -o stratum+tcp://usa-1.liteguardian.com:3335 -u aaaa.256 -p x  --chips-count 256 --ltc-clk 300 -S //./COM4 



===================================
execution option readme:

--ltc-debug 
enable debug info output


--chips-count 16   
16 chips in one COM port

--ltc-clk  250
clock 250M

--nocheck-golden
init with no Golden number check

--nocheck-scrypt
Don't check the miner's result. 
Must set it on openwrt 703N version, as the scrypt.c calculates wrong on MIPS platform.

====================================
change history:

tag4:
adjust diff as pool's wish

