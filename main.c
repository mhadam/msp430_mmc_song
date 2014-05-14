#include <msp430g2553.h>
#include <string.h> //For mem copy
#include "MMC.h"
#include "hal_SPI.h"
#include "hal_MMC_hardware_board.h"
#include "fifo.h"

//Red onboard LED
const unsigned char LED = BIT0;

volatile unsigned long currentBlock = 0; // current block being read from mmc device
volatile unsigned int frameCount = 0;
volatile unsigned int count = 0; // size of ring buffer
volatile unsigned int bufferReady = 0; // ready to receive samples
volatile unsigned long size = 0;

volatile unsigned char result = 0;
volatile unsigned char channel = 0; // mono channel sample

volatile unsigned long offset = 0; // size of the music sample data in bytes
volatile unsigned long fileCount = 0; // number of samples read (left and right channels)

volatile unsigned char header[4] = {0}; // used to store header information (6: 32-bit fields)
volatile unsigned char load[1] = {0}; // used to store each byte spi read

void main(void) {

	WDTCTL = WDTPW + WDTHOLD;        // Stop watchdog timer

	DCOCTL = CALDCO_16MHZ;
	BCSCTL1 = CALBC1_16MHZ;

	//Reset SD card by cycling power
	//Credit to Boris Cherkasskiy and his blog post on Launchpad+MMC
	P2DIR |= BIT2;
	P2OUT &= ~BIT2;
	__delay_cycles(1600000);	// 100ms @ 16MHz
	P2OUT |= BIT2;
	__delay_cycles(1600000);	// 100ms @ 16MHz

	//Initialize MMC library
	while (MMC_SUCCESS != mmcInit());
	//mmcInit();
	//Verify card ready
	while (MMC_SUCCESS != mmcPing());
	//Check the memory card size
	size = mmcReadCardSize();

	P1DIR |= LED;
	P1OUT &= ~LED;

	result = mmcMountBlock(0, 512); // mount first sector

	int i = 0;

	if (result == MMC_SUCCESS) {
		for (i = 0; i < 2; i++) {
			spiReadFrame((void*)header, 4); // read first two words in header into the header buffer
		}
	}

	if (result == MMC_SUCCESS) {
		for (i = 0; i < 4; i++) {
			spiReadFrame((void*)(header+(3-i)), 1); // read file data offset into buffer backwards (account for big endian)
		}
	}

	memcpy((void *)&offset, (void*)(header), 4); // set file offset

	for (i = 0; i < 3; i++) {
		spiReadFrame((void*)header, 4); // read the rest of the header off
	}

	// set timers and interrupts now that the header is read
	TA0CCR0 = 255; // volume of sample (8 bit)
	TA0CCR1 = 0;
	TA0CTL = TASSEL_2 + MC_1 + ID_1; // Timer A0 with SMCLK, count UP, div by 2

	TA0CCTL0 = CCIE; // enable TACCR0 interrupt
	TA0CCTL1 = OUTMOD_7; // enable TACCR1 interrupt w/ toggle/reset

	P1DIR |= BIT6; P1SEL |= BIT6; // set sound output pin

	while (fifo_in_ready()) { // fill the buffer
		spiReadFrame((unsigned char *)load, 1);
		fifo_in(load[0]);
		count++;
		fileCount++;
		frameCount++;
	}

	_BIS_SR(GIE); // enable interrupts and sleep


	while(1){
		if (fifo_in_ready()) { // make sure the buffer isn't overloaded
			spiReadFrame((unsigned char *)load, 1);
			fifo_in(load[0] + 128);

			//count++; // account for 8 samples added to buffer
			fileCount++; // account for 8 samples towards end of data
			frameCount++; // an 8 byte frame has been read

			if (frameCount == 512) {
				currentBlock += 1; // move onto next block since all samples have been read
				mmcUnmountBlock(); // unmount old block

				if (fileCount == offset) { // check for end of file
					currentBlock = 0;
					fileCount = 0;
				}

				//CS_HIGH(); // maybe this will help?
				//spiSendByte(0xFF);

				result = mmcMountBlock((long)(currentBlock*512), 512); // mount next block
				frameCount = 0; // reset frame count
			}
		}else{
			//LPM0; // sleep since the buffer is full
		}
	}
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 (void) {
	if (fifo_out_ready()){
		channel = fifo_out(); // take a sample from the ring buffer
		TACCR1 = channel;
		//__bic_SR_register_on_exit(CPUOFF);
	}
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1 (void) {
}
