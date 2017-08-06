/*
   The MIT License (MIT)

   Copyright (c) 2016 silverx

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
 */

/*

// original bluetooth LE idea by Dmitry Grinberg
// http://dmitry.gr/index.php?r=05.Projects&proj=11.%20Bluetooth%20LE%20fakery

// some bluetooth LE functions adapted from nrf24 code by Lijun 
// http://doc.lijun.li/misc-nrf24-ble.html
// https://github.com/lijunhw/nRF24_BLE/blob/master/Arduino/nRF24_BLE_advertizer_demo/nRF24_BLE_advertizer_demo.ino

 */
#include "config.h"
#ifdef RX_BAYANG_BIKEMIKE_TELEM

#include "binary.h"
#include "drv_spi.h"
#include "project.h"
#include "xn297.h"


#include "drv_time.h"
#include <stdio.h>
#include "defines.h"

#include "rx_bayang.h"

#include "util.h"

#define RX_MODE_NORMAL RXMODE_NORMAL
#define RX_MODE_BIND RXMODE_BIND
#define _BV( bit )   (1 << (bit))

#ifdef RX_TELEMETRY_BIKEMIKE
uint8_t send_telemetry = 0;
uint8_t flightmode     = 0;
uint8_t datamode       = 0;
uint8_t dataselect     = 1;
uint8_t dataadjust     = 1;
extern int onground;
#endif

// ble settings
#ifdef BLUETOOTH_ENABLE
// beacon interval
#define BLE_INTERVAL 30000

// this allows different quads to show up at the same time.
// 0 - 255 select a different number for each quad if you need several simultaneous
#define BLE_QUAD_NUMBER 17


// optimized one channel only (bluetooth)
// uses precalculated whitening data
// possible values: 0 / 1
#define ONE_CHANNEL 1
#endif

// radio settings

// packet period in uS
uint32_t packet_period = 0;
uint32_t packet_count  = 0; // used to calculate the packet_period

// was 250 ( uS )
#define PACKET_OFFSET 250

#ifdef USE_STOCK_TX
#undef PACKET_OFFSET
#define PACKET_OFFSET -250
#endif

// how many times to hop ahead if no reception
#define HOPPING_NUMBER 4

// because it's from the cg023 port

float rx[4];
char aux[AUXNUMBER];
char lastaux[AUXNUMBER];
char auxchange[AUXNUMBER];
char lasttrim[4];

char rfchannel[4];
int rxaddress[5] = {0};
int rxmode = 0;
int rf_chan = 0;


void bleinit( void);

void writeregs ( uint8_t data[] , uint8_t size )
{
	spi_cson();
	for ( uint8_t i = 0 ; i < size ; i++)
	{
		spi_sendbyte( data[i]);
	}
	spi_csoff();
	delay(1000);
}



void rx_init()
{


	// always on (CH_ON) channel set 1
	aux[AUXNUMBER - 2] = 1;
	// always off (CH_OFF) channel set 0
	aux[AUXNUMBER - 1] = 0;
#ifdef AUX1_START_ON
	aux[CH_AUX1] = 1;
#endif

#ifdef AUX4_START_ON
	aux[CH_AUX4] = 1;
#endif


#ifdef RADIO_XN297L

#ifndef TX_POWER
#define TX_POWER 7
#endif
	
// Gauss filter amplitude - lowest
static uint8_t demodcal[2] = { 0x39 , B00000001 };
writeregs( demodcal , sizeof(demodcal) );

// powerup defaults
//static uint8_t rfcal2[7] = { 0x3a , 0x45 , 0x21 , 0xef , 0xac , 0x3a , 0x50};
//writeregs( rfcal2 , sizeof(rfcal2) );
	
#define XN_TO_RX B10001111
#define XN_TO_TX B10000010
#define XN_POWER_RX B00000001|((TX_POWER&7)<<3)
#define XN_POWER_TX B00000001|((TX_POWER&7)<<3)

#endif



#ifdef RADIO_XN297
	static uint8_t bbcal[6] = { 0x3f , 0x4c , 0x84 , 0x6F , 0x9c , 0x20  };
	writeregs( bbcal , sizeof(bbcal) );
	// new values
	static uint8_t rfcal[8] = { 0x3e , 0xc9 , 0x9a , 0xA0 , 0x61 , 0xbb , 0xab , 0x9c  };
	writeregs( rfcal , sizeof(rfcal) );

	// 0xa7 0x03
	static uint8_t demodcal[6] = { 0x39, 0x0b, 0xdf, 0xc4, B00100111, B00000000 };
	writeregs(demodcal, sizeof(demodcal));


#define XN_TO_RX B00001111
#define XN_TO_TX B00000010
#define XN_POWER_RX B00000001 // LNA enabled
#define XN_POWER_TX B00000110 // RF_PWR 00 = -10dBm, 01 = 0dBm, 10 = 8dBm, 11 = max 10dBm 
#endif


#ifdef BLUETOOTH_ENABLE
	bleinit();
#endif

	delay(100);


	xn_writerxaddress( rxaddress);
#ifdef RX_TELEMETRY_BIKEMIKE
	xn_writetxaddress( rxaddress);
#endif
	xn_writereg( EN_AA , 0 );	// aa disabled
	xn_writereg( EN_RXADDR , 1 ); // pipe 0 only
	xn_writereg( RF_SETUP , XN_POWER_RX);
	xn_writereg( RX_PW_P0 , 15 ); // payload size
	xn_writereg( SETUP_RETR , 0 ); // no retransmissions ( redundant?)
	xn_writereg( SETUP_AW , 3 ); // address size (5 bits)
	xn_command( FLUSH_RX);
	xn_writereg( RF_CH , 0 );  // bind on channel 0

	// set above
	//	xn_writereg( 29 , 32); // feture reg , CE mode (software controlled)

#ifdef RADIO_XN297L
	xn_writereg( 0x1d, B00111000 ); // 64 bit payload , software ce
	spi_cson();
	spi_sendbyte( 0xFD); // internal CE high command
	spi_sendbyte( 0); // required for above
	spi_csoff();
#endif

#ifdef RADIO_XN297
	xn_writereg( 0x1d, B00011000 ); // 64 bit payload , software ce
#endif

	xn_writereg( 0 , XN_TO_RX ); // power up, crc enabled, rx mode
	xn_writereg( RF_SETUP , XN_POWER_RX);

	xn_writereg(STATUS, 0x70); // clear rx_dr, tx_ds
	xn_command( FLUSH_RX);

#ifdef RADIO_CHECK
	void check_radio(void);
	check_radio();
#endif	
}


void check_radio()
{	
	int	rxcheck = xn_readreg( 0x0f); // rx address pipe 5	
	// should be 0xc6
	extern void failloop( int);
	if ( rxcheck != 0xc6) failloop(3);
}

#ifdef BLUETOOTH_ENABLE
void btLeCrc( uint8_t* buf ,uint8_t len, uint8_t* dst ) {

	union
	{
		unsigned int int32;
		uint8_t u8[4];
	} myint;

	myint.int32 = 0x00aaaaaa;

	while (len--) 
	{
		uint8_t d = *(buf++);
		for ( int i=8 ; i>0 ; i--) 
		{
			uint8_t t = myint.int32&1;			
			myint.int32>>=1;
			if (t != (d & 1)) 
			{
				myint.u8[2] ^= 0xDA;
				myint.u8[1] ^= 0x60;
			}
			d >>=1;
		}
	}

	for ( int i = 0 ; i < 3 ; i++ )
		dst[i] = (myint.u8[i]);

}




// scrambling sequence for xn297
const uint8_t xn297_scramble[] = {
	0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
	0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
	0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
	0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
	0x8e, 0xc5, 0x2f};

// bit reversed of above
const uint8_t xn297_scramble_rev[] = { 
	0xc7 , 0x8d , 0xd2 , 0x57 , 0xa1 , 0x3d , 0xa7 , 0x66 ,
	0xb0 , 0x75 , 0x31 , 0x11 , 0x48 , 0x96 , 0x77 , 0xf8 , 
	0xe3 , 0x46 , 0xe9 , 0xab , 0xd0 , 0x9e , 0x53 , 0x33 , 
	0xd8 , 0xba , 0x98 , 0x8 , 0x24 , 0xcb , 0x3b , 0xfc , 
	0x71 , 0xa3 , 0xf4 , 85  , 104  , 207  , 169  , 25   ,
	108  ,  93  , 76   , 4   , 146  , 229  , 29  };

// whitening sequence for adv channel 37 (rf chan 2402)
// for speed
const uint8_t ble_whiten_37[] = { 
	0x8D , 0xd2 , 0x57 , 0xa1 , 0x3d , 0xa7 , 0x66 , 0xb0 ,
	0x75 , 0x31 , 0x11 , 0x48 , 0x96 , 0x77 , 0xf8 , 0xe3 ,
	0x46 , 0xe9 , 0xab , 0xd0 , 0x9e , 0x53 , 0x33 , 0xd8 ,
	0xba , 0x98 , 0x08 , 0x24 , 0xcb , 0x3b , 0xfc , 0x71 ,
	0xa3 , 0xf4 , 0x55 , 0x68 , 0xCF , 0xA9 , 0x19 , 0x6C ,
	0x5D , 0x4C
}; // whitening sequence channel 37 ( 0 - index ; 2 - rf channel; 37 - ble spec)


uint8_t chRf[3] = {2, 26,80};
uint8_t chLe[3] = {37,38,39};
uint8_t whitenstart[] = { 0xa6 , 0x66 , 0xe6};

/*
   uint8_t swapbits_old(uint8_t a){
// reverse the bit order in a single byte
uint8_t v = 0;
if(a & 0x80) v |= 0x01;
if(a & 0x40) v |= 0x02;
if(a & 0x20) v |= 0x04;
if(a & 0x10) v |= 0x08;
if(a & 0x08) v |= 0x10;
if(a & 0x04) v |= 0x20;
if(a & 0x02) v |= 0x40;
if(a & 0x01) v |= 0x80;
return v;
}

// from https://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith32Bits
// reverse the bit order in a single byte
uint8_t swapbits(uint8_t a){
unsigned int b = a;
b = ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
return b;
}

// cortex m3 intrinsic bitswap
uint8_t swapbits_m3(uint8_t a)
{
return (unsigned int) __rbit( (unsigned int) a); 
}
 */

void btLeWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff)
{
	// Implementing whitening with LFSR
	uint8_t m;
	while(len--)
	{
		for(m = 1; m; m <<= 1)
		{
			if(whitenCoeff & 0x80)
			{
				whitenCoeff ^= 0x11;
				(*data) ^= m;
			}
			whitenCoeff <<= 1;
		}
		data++;
	}
}
/*
   static inline uint8_t btLeWhitenStart(uint8_t chan){
//the value we actually use is what BT'd use left shifted one...makes our life easier
return swapbits(chLe[chan]) | 2;
}
 */


void btLePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan)
{
	// Assemble the packet to be transmitted
	// Length is of packet, including crc. pre-populate crc in packet with initial crc value!
	uint8_t i, dataLen = len - 3;


	packet[len - 3] = 0x55; //CRC start value: 0x555555
	packet[len - 2] = 0x55;
	packet[len - 1] = 0x55;

	btLeCrc(packet, dataLen, packet + dataLen);

	//for(i = 0; i < 3; i++, dataLen++)
	//	packet[dataLen] = swapbits(packet[dataLen]);

	if (ONE_CHANNEL)
	{	
		// faster array based whitening
		for(i = 0; i < len; i++) 
			packet[i] ^=ble_whiten_37[i];
	}
	else // lfsr based
		btLeWhiten(packet, len, whitenstart[chan]);	
}

#endif

#define RXDEBUG

unsigned long packettime;
#ifdef RXDEBUG
int channelcount[4];
int failcount;
int packetrx;
int packetpersecond;


int skipstats[12];
int afterskip[12];
//#warning "RX debug enabled"
#endif

#ifdef BLUETOOTH_ENABLE

// works with 4 and 5 addresses byte
#define XN297_ADDRESS_SIZE_BLE 5


// The MAC address of BLE advertizer -- just make one up
#define MY_MAC_0	BLE_QUAD_NUMBER
#define MY_MAC_1	0x22
#define MY_MAC_2	0x33
#define MY_MAC_3	0x44
#define MY_MAC_4	0x55
#define MY_MAC_5	0xF6


uint8_t buf[48];
int buffint[48];

uint8_t ch = 0; // RF channel for frequency hopping

int payloadsize;


void bleinit()
{
	// Set access addresses (TX address in nRF24L01) to BLE advertising 0x8E89BED6
	// Remember that both bit and byte orders are reversed for BLE packet format

	int txaddr[5];

	/*	
	// 4 byte address
	txaddr[0] = swapbits(0x8E)^xn297_scramble[3];
	txaddr[1] = swapbits(0x89)^xn297_scramble[2];
	txaddr[2] = swapbits(0xBE)^xn297_scramble[1];
	txaddr[3] = swapbits(0xD6)^xn297_scramble[0];
	txaddr[4] = 0;
	 */

	/*	
	// 4 byte address - optimized
	txaddr[0] = 0x71^0xea;
	txaddr[1] = 0x91^0x4b;
	txaddr[2] = 0x7d^0xb1;
	txaddr[3] = 0x6b^0xe3;
	txaddr[4] = 0;
	 */

	// using 5 byte address because it's shared with rx (rx protocol has 5 bytes)
	// saves changing the address size everytime we send
	txaddr[0] = 0x71^0x85;
	txaddr[1] = 0x91^0xea;
	txaddr[2] = 0x7d^0x4b;
	txaddr[3] = 0x6b^0xb1;
	txaddr[4] = 0xaa^0xe3;	// preamble


	xn_writetxaddress( txaddr );	



	//	xn_writereg( EN_AA , 0 );	// aa disabled -- duplicated
	//	xn_writereg( RF_SETUP , B00111011);  // high power xn297L only
	//	xn_writereg( SETUP_RETR , 0 ); // no retransmissions  -- duplicated
	// -- duplicated
	//	xn_writereg( SETUP_AW , XN297_ADDRESS_SIZE_BLE - 2 ); // address size (4 bytes for ble)

	//  xn_writereg( 0x1d, B00111000 ); // 64 bit payload , software ce

}

void send_beacon(void);

int loopcounter = 0;
unsigned int ble_txtime;
int ble_send = 0;
int oldchan = 0;

#define BLE_TX_TIMEOUT 10000

int beacon_sequence()
{
	static int beacon_seq_state = 0;

	switch ( beacon_seq_state )
	{
		case 0:
			// send data if enough time passed since last send
			if ( gettime() - ble_txtime > BLE_INTERVAL )
			{
				ble_send = 1;
				oldchan = rf_chan;
#ifdef RX_TELEMETRY_BIKEMIKE
				bleinit(); // only need to do this if telemetry is also enabled
#endif
				send_beacon();
				beacon_seq_state++;
			}
			break;

		case 1:
			// wait for data to finish transmitting
			if( (xn_readreg(0x17)&B00010000)  ) 
			{
				xn_writereg( 0 , XN_TO_RX ); 
				xn_writereg( RF_SETUP , XN_POWER_RX);
				xn_writereg(0x25, rfchannel[oldchan]);
				beacon_seq_state++;
				goto next;
			}
			else
			{// if it takes too long we get rid of it
				if ( gettime() - ble_txtime > BLE_TX_TIMEOUT )
				{
					xn_command( FLUSH_TX);
					xn_writereg( 0 , XN_TO_RX ); 
					xn_writereg( RF_SETUP , XN_POWER_RX);
					beacon_seq_state++;
					ble_send = 0;
				}
			}
			break;


		case 2:
next:
			// restore radio settings to protocol compatible
			// mainly channel here
			ble_send = 0;	
			if ( rxmode == 0 )
			{
				xn_writereg(0x25, 0 ); // Set channel frequency	, bind
			}
			beacon_seq_state++;

#ifdef RX_TELEMETRY_BIKEMIKE
			xn_writetxaddress( rxaddress); // only need to do this if telemetry is also enabled
#endif
			break;

		default:
			beacon_seq_state = 0;		 
			break;



	}

	return beacon_seq_state;
}

int interleave = 0;

void send_beacon()
{

	// Channel hopping
	ch++;
	if (ch>2 ) 
	{
		ch = 0;
	}
	// sending on channel 37 only to use whitening array
	if (ONE_CHANNEL) ch = 0;


	xn_writereg(RF_CH, chRf[ch]);

	uint8_t L=0;



#ifdef USE_IBEACON
	L=0;
	// ibeacon packet structure
	buf[L++] = B00100010; //PDU type, given address is random; 0x42 for Android and 0x40 for iPhone
	buf[L++] = 36; // length of payload
	buf[L++] = MY_MAC_0;
	buf[L++] = MY_MAC_1;
	buf[L++] = MY_MAC_2;
	buf[L++] = MY_MAC_3;
	buf[L++] = MY_MAC_4;
	buf[L++] = MY_MAC_5;

	// packet data unit
	buf[L++] = 2; //flags (LE-only, limited discovery mode)
	buf[L++] = 0x01;
	buf[L++] = 0x06;
	buf[L++] = 0x1A; // length of the name, including type byte
	buf[L++] = 0xff;
	buf[L++] = 0x4c;
	buf[L++] = 0x00;
	buf[L++] = 0x02;
	buf[L++] = 0x15;
	buf[L++] = 0x58;
	buf[L++] = 0x5c;
	buf[L++] = 0xde;
	buf[L++] = 0x93;
	buf[L++] = 0x1b;
	buf[L++] = 0x01;
	buf[L++] = 0x42;
	buf[L++] = 0xcc;
	buf[L++] = 0x9a;
	buf[L++] = 0x13;
	buf[L++] = 0x25;
	buf[L++] = 0x00;
	buf[L++] = 0x9b;
	buf[L++] = 0xed;
	buf[L++] = 0xc6;
	buf[L++] = 0xe5;
	buf[L++] = 0x00;
	buf[L++] = 0x00;
	buf[L++] = 0x00;
	buf[L++] = 0x00;
	buf[L++] = 0xCA; // tx power
#else


	extern float vbattfilt;
	int vbatt = vbattfilt *1000.0f;

	unsigned int time = gettime();

	time = time>>20; // divide by 1024*1024, no time for accuracy here
	time = time * 10;

	L=0;
	buf[L++] = B00100010; //PDU type, given address is random; 0x42 for Android and 0x40 for iPhone
	//buf[L++] = 0x42; //PDU type, given address is random; 0x42 for Android and 0x40 for iPhone

	// max len 27 with 5 byte address = 37 total payload bytes
	buf[L++] = 10+ 21; // length of payload
	buf[L++] = MY_MAC_0;
	buf[L++] = MY_MAC_1;
	buf[L++] = MY_MAC_2;
	buf[L++] = MY_MAC_3;
	buf[L++] = MY_MAC_4;
	buf[L++] = MY_MAC_5;

	// packet data unit
	buf[L++] = 2; //flags lenght(LE-only, limited discovery mode)
	buf[L++] = 0x01; // compulsory flags
	buf[L++] = 0x06; // flag value
	buf[L++] =  0x03;  // Length of next block
	buf[L++] =  0x03;  // Param: Service List
	buf[L++] =  0xAA;  // Eddystone ID - 16 bit 0xFEAA
	buf[L++] =  0xFE;  // Eddystone ID
	buf[L++] =  0x11;  // Length of next block
	buf[L++] =  0x16;  // Service Data
	buf[L++] =  0xAA;  // Eddystone ID
	buf[L++] =  0xFE;  // Eddystone ID
	buf[L++] =  0x20;  // TLM flag
	buf[L++] =  0x00;  // TLM version
	buf[L++] =  vbatt>>8;  // Battery voltage
	buf[L++] =  vbatt;  // Battery voltage
	buf[L++] =  0x80;  // temperature 8.8 fixed point
	buf[L++] =  0x00;  // temperature 8.8 fixed point
	buf[L++] =  0x00;  // advertisment count 0
	buf[L++] =  0x00;  // advertisment count 1
	buf[L++] =  packetpersecond>>8&0xff;  // advertisment count 2
	buf[L++] =  packetpersecond&0xff;  // advertisment count 3
	buf[L++] =  time>>24;  // powerup time 0 
	buf[L++] =  time>>16;  // powerup time 1
	buf[L++] =  time>>8;  // powerup time 2
	buf[L++] =  time;  // powerup time 3 in seconds times 10.
#endif
	L=L+3; //crc


	btLePacketEncode(buf, L, ch );

	// undo xn297 data whitening
	for (uint8_t i = 0; i < L; ++i) 
	{
		buf[i] = buf[i] ^ xn297_scramble_rev[i+ XN297_ADDRESS_SIZE_BLE] ;
	}


	for( int i = 0 ; i < L ; i++) buffint[i] = buf[i];


	xn_command( FLUSH_TX);

	xn_writereg( 0 , XN_TO_TX );
	xn_writereg( RF_SETUP , XN_POWER_TX);

	payloadsize = L;
	xn_writepayload( buffint , L );

	ble_txtime = gettime();

	return;	
}

#endif

static char checkpacket()
{
#ifdef RADIO_XN297L
	uint8_t status = xn_readreg(7);
#else
	uint8_t status = xn_command(NOP);
#endif
	if (status & _BV(MASK_RX_DR))
	{
		return 1;
	}

	return 0;
}


int rxdata[15];


float packettodata( int *  data)
{
	return ( ( ( data[0]&0x0003) * 256 + data[1] ) - 512 ) * 0.001953125 ;	
}

#ifdef RX_TELEMETRY_BIKEMIKE
extern float pidkp[PIDNUMBER];
extern float pidki[PIDNUMBER];
extern float pidkd[PIDNUMBER];
extern float apidkp[2];
extern float apidkd[2];
extern float outlimit[3];
extern float integrallimit[3];
#endif

static int decodepacket( void)
{
	if ( rxdata[0] == 165 )
	{
		int sum = 0;
		for(int i=0; i<14; i++) 
		{
			sum += rxdata[i];
		}	
		if ( (sum&0xFF) == rxdata[14] )
		{
			rx[0] = packettodata( &rxdata[4] );
			rx[1] = packettodata( &rxdata[6] );
			rx[2] = packettodata( &rxdata[10] );
			// throttle		
			rx[3] = ( (rxdata[8]&0x0003) * 256 + rxdata[9] ) * 0.000976562f;

#ifndef DISABLE_EXPO
			rx[0] = rcexpo ( rx[0] , EXPO_XY );
			rx[1] = rcexpo ( rx[1] , EXPO_XY ); 
			rx[2] = rcexpo ( rx[2] , EXPO_YAW ); 	
#endif



			// trims are 50% of controls at max
			// trims are not used as trims because they interfere
			// with dynamic trims feature of devo firmware

			//                      rx[0] = rx[0] + 0.03225 * 0.5 * (float)(((rxdata[4])>>2) - 31);
			//                      rx[1] = rx[1] + 0.03225 * 0.5 * (float)(((rxdata[6])>>2) - 31);
			//                      rx[2] = rx[2] + 0.03225 * 0.5 * (float)(((rxdata[10])>>2) - 31);

			// Instead they are used as binary aux channels
#ifdef USE_STOCK_TX
			char trims[4];
			trims[0] = rxdata[6] >> 2;
			trims[1] = rxdata[4] >> 2;
			// trims[2] = rxdata[8] >> 2; // throttle and yaw trims are not used
			// trims[3] = rxdata[10] >> 2;
			for (int i = 0; i < 2; i++)
				if (trims[i] != lasttrim[i])
				{
					aux[CH_PIT_TRIM + i] = trims[i] > lasttrim[i];
					lasttrim[i] = trims[i];
				}
#else			
			// this share the same numbers to the above CH_PIT_TRIM etc		 					
			aux[CH_VID] = (rxdata[2] & 0x10) ? 1 : 0;

			aux[CH_PIC] = (rxdata[2] & 0x20) ? 1 : 0;						
#endif

			aux[CH_INV] = (rxdata[3] & 0x80)?1:0; // inverted flag

			aux[CH_FLIP] = (rxdata[2] & 0x08) ? 1 : 0;

			aux[CH_EXPERT] = (rxdata[1] == 0xfa) ? 1 : 0;

			aux[CH_HEADFREE] = (rxdata[2] & 0x02) ? 1 : 0;

			aux[CH_RTH] = (rxdata[2] & 0x01) ? 1 : 0;	// rth channel
#ifdef RX_TELEMETRY_BIKEMIKE
			send_telemetry = (rxdata[2] & 0x04) ? 1: 0; // custom telemetry enable flag

			flightmode = (rxdata[3]) & 0x3;
			static uint8_t new_dataselect = 0;
			new_dataselect = (rxdata[3]>>2) & 0x7;
			static uint8_t new_dataadjust = 0;
			new_dataadjust =(rxdata[3]>>5) & 0x3;


			// data mode  = 0, 1, 2, 3 (pid roll/pitch, pid yaw, angle p&i, limits)
			uint8_t num_data_modes = 5;
			if (dataselect != new_dataselect)
			{
				dataselect = new_dataselect;
				if (dataselect == 0)
				{
					datamode = (datamode + (num_data_modes-1))%num_data_modes;
				}
				else if (dataselect == 4)
				{
					datamode = (datamode + 1)%num_data_modes;
				}
			}

			if (dataadjust != new_dataadjust)
			{
				dataadjust = new_dataadjust;
				if (dataselect != 0 && dataselect != 4)
				{
					dataadjust = new_dataadjust;

					float multiplier = 1.f;

					if (0 == dataadjust)
					{
						multiplier = 1/1.1f;
					}
					else if (2 == dataadjust)
					{
						multiplier = 1.1f;
					}


					// data mode (pid roll/pitch, pid yaw, angle p&i, limits)
					if (0 == datamode)
					{
						if (1 == dataselect)
						{
							pidkp[0] *= multiplier;
							pidkp[1] *= multiplier;
						}
						else if (2 == dataselect)
						{
							pidki[0] *= multiplier;
							pidki[1] *= multiplier;
						}
						else if (3 == dataselect)
						{
							pidkd[0] *= multiplier;
							pidkd[1] *= multiplier;
						}
					}
					else if (1 == datamode)
					{
						if (1 == dataselect)
						{
							pidkp[2] *= multiplier;
						}
						else if (2 == dataselect)
						{
							pidki[2] *= multiplier;
						}
						else if (3 == dataselect)
						{
							pidkd[2] *= multiplier;
						}
					}
					else if (2 == datamode)
					{
						if (1 == dataselect)
						{
							apidkp[0] *= multiplier;
						}
						else if (2 == dataselect)
						{
							apidkd[0] *= multiplier;
						}
						else if (3 == dataselect)
						{
						}
					}
					else if (3 == datamode)
					{
						if (1 == dataselect)
						{
							outlimit[0] *= multiplier;
							integrallimit[0] *= multiplier;
						}
						else if (2 == dataselect)
						{
							outlimit[1] *= multiplier;
							integrallimit[1] *= multiplier;
						}
						else if (3 == dataselect)
						{
							outlimit[2] *= multiplier;
							integrallimit[2] *= multiplier;
						}
					}
				}
			}
#endif


			for ( int i = 0 ; i < AUXNUMBER - 2 ; i++)
			{
				auxchange[i] = 0;
				if ( lastaux[i] != aux[i] ) auxchange[i] = 1;
				lastaux[i] = aux[i];
			}

			return 1;	// valid packet	
		}
		return 0; // sum fail
	}
	return 0; // first byte different
}



void nextchannel()
{
	rf_chan++;
	rf_chan%=4;
	xn_writereg(0x25, rfchannel[rf_chan]);
	xn_writereg(STATUS, 0x70); // clear rx_dr, tx_ds
	xn_command( FLUSH_RX);
	xn_command( FLUSH_TX);
}



unsigned long lastrxtime;
unsigned long failsafetime;
unsigned long secondtimer;

int failsafe = 0;


unsigned int skipchannel = 0;
int lastrxchan;
int timingfail = 0;

#ifdef RX_TELEMETRY_BIKEMIKE


enum {
	RADIO_STATE_RECEIVE,
	RADIO_STATE_TRANSMIT,
	RADIO_STATE_TRANSMITTING,
};

uint8_t radio_state = RADIO_STATE_RECEIVE;

uint32_t telemetry_packets_sent = 0;
void radio_set_tx()
{
#ifdef RADIO_XN297L
	xn_writereg(CONFIG, _BV(EN_PM) | _BV(PWR_UP)|_BV(EN_CRC)|_BV(CRCO));
#else
	xn_writereg(CONFIG, _BV(PWR_UP)|_BV(EN_CRC)|_BV(CRCO));
#endif
	xn_writereg(STATUS, 0x70); // clear rx_dr, tx_ds
	xn_command(FLUSH_TX);
	xn_writereg( RF_SETUP , XN_POWER_TX);
}

void radio_set_rx()
{
#ifdef RADIO_XN297L
	xn_writereg(CONFIG, _BV(EN_PM) | _BV(PWR_UP)|_BV(PRIM_RX)|_BV(EN_CRC)|_BV(CRCO));
#else
	xn_writereg(CONFIG, _BV(PWR_UP)|_BV(PRIM_RX)|_BV(EN_CRC)|_BV(CRCO));
#endif
	xn_writereg(STATUS, 0x70); // clear rx_dr, tx_ds
	xn_command(FLUSH_RX);
	xn_writereg( RF_SETUP , XN_POWER_RX);
} 


extern float vbattfilt;

typedef union {
	uint16_t v;
	uint8_t  bytes[2];
} val;
uint32_t txtime = 0;
uint32_t txtime_elapse = 0;


// voltage, up time, flight time, flight mode, pid, accelerometer
void send_telemetry_packet()
{
	val v;
	uint32_t uptime = gettime();
	static uint32_t flighttime = 0;
	static uint32_t throttle_on_time = 0;

	static uint32_t packet_count_time  = 0;
	static uint16_t packets_per_second = 300;
	static uint16_t packet_count_next  = 30;

	++packet_count_next;

	if (packet_count_time < uptime)
	{
		// calculate moving average
		packets_per_second = packets_per_second + packet_count_next - packets_per_second/10;
		packet_count_time += 100000; // add 100ms
		packet_count_next = 0;
	}


	if (onground)
	{
		throttle_on_time = uptime;
	}
	else
	{
		flighttime += uptime - throttle_on_time;
		throttle_on_time = uptime;
	}

	v.v = (uint16_t)(vbattfilt*100);
	extern uint32_t loops_per_second;
	extern unsigned long elapsedtime;
	//FIXME:
	extern float times[3];

	rxdata[0] = 0xa9;       // packet id
	rxdata[1] = v.bytes[0]; // battery voltage
	rxdata[2] = v.bytes[1]; // battery voltage
	rxdata[3] = (uint8_t)(packets_per_second>>1); // pps / 2 divide by 2
	v.v = (uint16_t)(uptime / 1000000);
	rxdata[4] = v.bytes[0];
	rxdata[6] = v.bytes[1] & 0x0F;
	v.v = (uint16_t)(flighttime / 1000000);
	rxdata[5] = v.bytes[0];
	rxdata[6] |= v.bytes[1] << 4;

	// flight mode (bit 0-1),
	// data mode   (bit 2-5)
	// data select (bit 6-7)
	rxdata[7] = aux[LEVELMODE] ? 0 : 1; // flight mode (0 = level, 1 = acro, 2 = ?)
	// data mode (0=acro roll/acro pitch, 1=acro yaw, 2=level roll/pitch, )
	// not implemented: 3=rate xy, rate yaw, 4=level rate, max angle
	rxdata[7] |= (datamode&0xF) << 2;
	rxdata[7] |= ((dataselect-1)&0x3) << 6;


	float* d[3] = {0};

	if (datamode == 0)
	{
		d[0] = &pidkp[0];
		d[1] = &pidki[0];
		d[2] = &pidkd[0];
	}
	else if (datamode == 1)
	{
		d[0] = &pidkp[2];
		d[1] = &pidki[2];
		d[2] = &pidkd[2];
	}
	else if (datamode == 2)
	{
		d[0] = &apidkp[0];
		d[1] = &apidkd[0];
		d[2] = NULL;
	}
	else if (datamode == 3)
	{
		d[0] = &outlimit[0];
		d[1] = &outlimit[1];
		d[2] = &outlimit[2];
	}
	else if (datamode == 4)
	{
		d[0] = &times[0];
		d[1] = &times[1];
		d[2] = &times[2];
	}


	for (uint8_t i = 0; i < 3; ++i)
	{
		if (NULL != d[i])
		{
			v.v = (uint16_t)((*d[i])*1000);
			rxdata[8+i*2] = v.bytes[0]; // p
			rxdata[9+i*2] = v.bytes[1];
		}
		else
		{
			rxdata[8+i*2] = 0;
			rxdata[9+i*2] = 0;
		}
	}


	int sum = 0;
	for(int i=0; i<14; i++)
	{
		sum += rxdata[i];
	}
	rxdata[14] = sum&0xFF;

	radio_set_tx();
	xn_writepayload(  rxdata , 15 );
	txtime = gettime();
	radio_state = RADIO_STATE_TRANSMITTING;
}

#endif
uint32_t rx_bind_count   = 0;
uint32_t rx_packet_rate  = UINT32_MAX;
uint32_t rx_time_last  = 0;

uint8_t seq = 0;
uint32_t seq_errors = 0;

uint32_t chans_to_skip = 0;
uint32_t skipped_chans = 0;

void checkrx(void)
{
#ifdef RX_TELEMETRY_BIKEMIKE
	if (RADIO_STATE_TRANSMITTING == radio_state)
	{
#ifdef RADIO_XN297L 
		uint8_t status = xn_readreg(7);
#else
		int status = xn_command(NOP);
#endif

		if (status & _BV(MASK_TX_DS))
		{
			txtime_elapse = gettime() - txtime;
			++telemetry_packets_sent;
		}
		else
		{
			return;
		}

		radio_state = RADIO_STATE_RECEIVE;
		radio_set_rx();
		nextchannel();
		return;
	}
#endif

	int packetreceived = checkpacket();
	int pass = 0;
	if (packetreceived)
	{
		if (rxmode == RX_MODE_BIND)
		{		// rx startup , bind mode
			xn_readpayload(rxdata, 15);

			if (rxdata[0] == 0xA4 && rx_bind_count >= 255)
			{	// bind packet
				rfchannel[0] = rxdata[6];
				rfchannel[1] = rxdata[7];
				rfchannel[2] = rxdata[8];
				rfchannel[3] = rxdata[9];

				rxaddress[0] = rxdata[1];
				rxaddress[1] = rxdata[2];
				rxaddress[2] = rxdata[3];
				rxaddress[3] = rxdata[4];
				rxaddress[4] = rxdata[5];

				xn_writerxaddress(rxaddress);
				xn_writetxaddress(rxaddress);
				xn_writereg(0x25, rfchannel[rf_chan]);	// Set channel frequency 
				rxmode = RX_MODE_NORMAL;

#ifdef SERIAL
				printf(" BIND \n");
#endif
			}

			rx_bind_count++;

			xn_writereg(STATUS, 0x70); // clear rx_dr, tx_ds
			xn_command( FLUSH_RX);
		}
		else
		{		// normal mode
			if (0 != lastrxtime)
				packettime = gettime() - lastrxtime;

#ifdef RXDEBUG
			channelcount[rf_chan]++;

			if ( skipchannel&& !timingfail ) afterskip[skipchannel]++;
			if ( timingfail ) afterskip[0]++;

#endif

			unsigned long temptime = gettime();

			if (rx_packet_rate > temptime - rx_time_last)
				rx_packet_rate = temptime - rx_time_last;
			rx_time_last = temptime;


			xn_readpayload(rxdata, 15);
			pass = decodepacket();
#ifdef RX_TELEMETRY_BIKEMIKE
			if (!send_telemetry || packet_count < 100)
				nextchannel();
#else
			nextchannel();
#endif

			if (pass)
			{
#ifdef RXDEBUG
				packetrx++;
#endif
				skipchannel = 0;
				timingfail = 0;
				lastrxchan = rf_chan;
				lastrxtime = temptime;
				failsafetime = temptime;
				failsafe = 0;
			}
			else
			{
#ifdef RXDEBUG
				failcount++;
#endif
			}

			if (packet_count < 100)
			{
				if (packet_period == 0 || packet_period > packettime)
					packet_period = packettime;

				packet_count++;
			}
#ifdef RX_TELEMETRY_BIKEMIKE
			else
			{
				if (send_telemetry)
				{
#ifdef BLUETOOTH_ENABLE
					// allow bluetooth and telemetry to coexist
					int sending_beacon = beacon_sequence();
					if (!sending_beacon)
					{
						send_telemetry_packet();
						radio_state = RADIO_STATE_TRANSMITTING;
					}
#else
					send_telemetry_packet();
					radio_state = RADIO_STATE_TRANSMITTING;
#endif
				}
			}
#endif
		}		// end normal rx mode

	}			// end packet received
#ifdef BLUETOOTH_ENABLE
	beacon_sequence();
#endif		
	unsigned long time = gettime();

	if (0 == lastrxtime)
		lastrxtime = time;

#if 0
	// sequence period 12000
	if (packet_count >= 100)
	{
		if (time - lastrxtime > (HOPPING_NUMBER*packet_period + 1000) && rxmode != RX_MODE_BIND)
		{			
			//  channel with no reception   
			lastrxtime = time;
			// set channel to last with reception
			if (!timingfail) rf_chan = lastrxchan;
			// advance to next channel
			nextchannel();
			// set flag to discard packet timing
			timingfail = 1;
		}
//#if 0 //defined BLUETOOTH_ENABLE
		if ( !timingfail && !ble_send && skipchannel < HOPPING_NUMBER+1 && rxmode != RX_MODE_BIND)
		{
			unsigned int temp = time - lastrxtime ;

			if ( temp > 1000 && ( temp - (PACKET_OFFSET) )/((int) packet_period) >= (skipchannel + 1) )
			{
				nextchannel();
#ifdef RXDEBUG				
				skipstats[skipchannel]++;
#endif				
				skipchannel++;
			}
		}
//#endif
	}
#endif

	if (time - failsafetime > FAILSAFETIME)
	{	//  failsafe
		failsafe = 1;
		rx[0] = 0;
		rx[1] = 0;
		rx[2] = 0;
		rx[3] = 0;
	}
#ifdef RXDEBUG
	if (gettime() - secondtimer > 1000000)
	{
		packetpersecond = packetrx;
		packetrx = 0;
		secondtimer = gettime();
	}
#endif

}


#endif

