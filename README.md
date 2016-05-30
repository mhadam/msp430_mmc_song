msp430_mmc_song
===============
An MSP430 program that will read a raw file of sound samples from an MMC card using SPI, and the music will be output by PCM in stereo on pin 1 and 4. A ring-buffer is used to deal with latency.

The sample format is limited to 8-bit @ 44.1kHz.
