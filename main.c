#include <msp430g2553.h>
#include <string.h> //For mem copy
#include "MMC.h"
#include "hal_SPI.h"
#include "hal_MMC_hardware_board.h"

#define MAX_BUF 40

struct RingBuffer {
	//Where to add data
	unsigned int head;
	//Where to read data from
	unsigned int tail;
	unsigned char buffer[MAX_BUF];
	//Need to distinguish between empty and full
	unsigned char empty;
};

//Red onboard LED
const unsigned char LED = BIT0;

volatile struct RingBuffer txBuffer;
volatile unsigned long currentBlock = 0; // current block being read from mmc device
volatile unsigned int frameCount = 0;
volatile unsigned int count = 0; // size of ring buffer
volatile unsigned int bufferReady = 0; // ready to receive samples
volatile unsigned long size = 0;

volatile unsigned char result = 0;
volatile unsigned char lchannel = 0; // left channel sample
volatile unsigned char rchannel = 0; // right channel

volatile unsigned long offset = 0; // size of the music sample data in bytes
volatile unsigned long fileCount = 0; // number of samples read (left and right channels)

volatile unsigned char header[4] = {0}; // used to store header information (6: 32-bit fields)
volatile unsigned char load[8] = {0}; // used to store each byte spi read

void pushBuffer(unsigned char value) {
	// Cannot push if the buffer is full
	if (!txBuffer.empty && txBuffer.head == txBuffer.tail) { return; }
	//Push back the length
	txBuffer.buffer[txBuffer.head] = value;
	//Reset head if it increments to MAX_BUF (size of the buffer)
	if (MAX_BUF == ++txBuffer.head) {
		txBuffer.head = 0;
	}
	//Just pushed a value, cannot be empty
	txBuffer.empty = 0;
}

unsigned char popBuffer() {
	//This is an error, trying to over-read the buffer
	if (txBuffer.empty) {
		return 0;
	}
	unsigned char value = txBuffer.buffer[txBuffer.tail];
	if (MAX_BUF == ++txBuffer.tail) {
		txBuffer.tail = 0;
	}

	//See if we just emptied the buffer
	if (txBuffer.head == txBuffer.tail) {
		txBuffer.empty = 1;
	}
	return value;
}

void main(void) {

	WDTCTL = WDTPW + WDTHOLD;        // Stop watchdog timer

	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	//BCSCTL3 |= XCAP_3; // set ext crystal osc.

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
	TA1CCR0 = 255; // volume of sample (8 bit)
	TA1CCR1 = 1; // important so that code in TACCR1 executes
	TA1CCR2 = 1;
	TA1CTL = TASSEL_2 + MC_1 + ID_1; // Timer A0 with SMCLK, count UP, div by 2

	TA1CCTL0 = CCIE; // enable TACCR0 interrupt
	TA1CCTL1 = OUTMOD_7 + CCIE; // enable TACCR1 interrupt w/ toggle/reset
	TA1CCTL2 = OUTMOD_7 + CCIE;

	P2DIR |= BIT4 + BIT1; P2SEL |= BIT4 + BIT1; // set sound output pin

	for (i = 0; i < 32; i++) { // fill the buffer
		spiReadFrame((void *)load, 1);
		pushBuffer(load[0]);
	}

	count = 32; // buffer is full now
	frameCount = 9; // 3 (header) + 4 (fill buffer)
	fileCount = 32;

	_BIS_SR(GIE); // enable interrupts and sleep

	while(1){
		if (count < 32) { // make sure the buffer isn't overloaded
			spiReadFrame((void *)load, 8);
			for (i = 0; i < 8; i++) {
				pushBuffer(load[i] + 128);
			}

			count += 8; // account for 8 samples added to buffer
			fileCount += 8; // account for 8 samples towards end of data
			frameCount++; // an 8 byte frame has been read

			if (frameCount == 64) {
				currentBlock += 512; // move onto next block since all samples have been read
				mmcUnmountBlock(); // unmount old block

				if (fileCount >= offset) { // check for end of file
					currentBlock = 0;
					fileCount = 0;
				}

				CS_HIGH(); // this apparently speeds up the unmount/mount transition
				spiSendByte(0xFF);

				result = mmcMountBlock(currentBlock, 512); // mount next block
				frameCount = 0; // reset frame count
			}
		}else{
			LPM0; // sleep since the buffer is full
		}
	}
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0 (void) {
	if (count > 1) {
		lchannel = popBuffer(); // take a sample from the ring buffer
		rchannel = popBuffer(); // take a sample from the ring buffer
		count = count - 2; // buffer is one smaller
	}
	LPM0_EXIT; // wake processor
}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1 (void) {
	switch (TA1IV){ // interrupt flag reset by reading TAIV
	case 0:
		break;
	case 2: // TACCR1
		if (lchannel == 0) {
			lchannel = 1;
		}
		TA1CCR1 = lchannel; // set left channel sample
		break;
	case 4: // TACCR2
		if (rchannel == 0) {
			rchannel == 1;
		}
		TA1CCR2 = rchannel; // set right channel sample
		break;
	default:
		break;
	}
}
