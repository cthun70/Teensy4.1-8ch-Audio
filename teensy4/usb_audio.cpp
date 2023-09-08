/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include "usb_dev.h"
#include "usb_audio.h"
#include "debug/printf.h"

#ifdef AUDIO_INTERFACE

bool AudioInputUSB::update_responsibility;
audio_block_t * AudioInputUSB::incoming_left;
audio_block_t * AudioInputUSB::incoming_right;
audio_block_t * AudioInputUSB::ready_left;
audio_block_t * AudioInputUSB::ready_right;
uint16_t AudioInputUSB::incoming_count;
uint8_t AudioInputUSB::receive_flag;           

struct usb_audio_features_struct AudioInputUSB::features = {0,0,FEATURE_MAX_VOLUME/2};

extern volatile uint8_t usb_high_speed;
static void rx_event(transfer_t *t);
static void tx_event(transfer_t *t);

/*static*/ transfer_t rx_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t sync_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t tx_transfer __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[AUDIO_RX_SIZE] __attribute__ ((aligned(32)));
DMAMEM static uint8_t tx_buffer[AUDIO_RX_SIZE] __attribute__ ((aligned(32)));
DMAMEM uint32_t usb_audio_sync_feedback __attribute__ ((aligned(32)));

uint8_t usb_audio_receive_setting=0;
uint8_t usb_audio_transmit_setting=0;
uint8_t usb_audio_sync_nbytes;
uint8_t usb_audio_sync_rshift;

uint32_t feedback_accumulator;

volatile uint32_t usb_audio_underrun_count;
volatile uint32_t usb_audio_overrun_count;


static void rx_event(transfer_t *t)
{
	if (t) {
		int len = AUDIO_RX_SIZE - ((rx_transfer.status >> 16) & 0x7FFF);
		printf("rx %u\n", len);
		usb_audio_receive_callback(len);
	}
	usb_prepare_transfer(&rx_transfer, rx_buffer, AUDIO_RX_SIZE, 0);
	arm_dcache_delete(&rx_buffer, AUDIO_RX_SIZE);
	usb_receive(AUDIO_RX_ENDPOINT, &rx_transfer);
}

static void sync_event(transfer_t *t)
{
	// USB 2.0 Specification, 5.12.4.2 Feedback, pages 73-75
	//printf("sync %x\n", sync_transfer.status); // too slow, can't print this much
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&sync_transfer, &usb_audio_sync_feedback, usb_audio_sync_nbytes, 0);
	arm_dcache_flush(&usb_audio_sync_feedback, usb_audio_sync_nbytes);
	usb_transmit(AUDIO_SYNC_ENDPOINT, &sync_transfer);
}

void usb_audio_configure(void)
{   
	printf("usb_audio_configure\n");    
	usb_audio_underrun_count = 0;
	usb_audio_overrun_count = 0;
	feedback_accumulator = 739875226; // 44.1 * 2^24
	if (usb_high_speed) {
		usb_audio_sync_nbytes = 4;
		usb_audio_sync_rshift = 8;
	} else {
		usb_audio_sync_nbytes = 3;
		usb_audio_sync_rshift = 10;
	}
	memset(&rx_transfer, 0, sizeof(rx_transfer));
	usb_config_rx_iso(AUDIO_RX_ENDPOINT, AUDIO_RX_SIZE, 1, rx_event);
	rx_event(NULL);
	memset(&sync_transfer, 0, sizeof(sync_transfer));
	usb_config_tx_iso(AUDIO_SYNC_ENDPOINT, usb_audio_sync_nbytes, 1, sync_event);
	sync_event(NULL);
	memset(&tx_transfer, 0, sizeof(tx_transfer));
	usb_config_tx_iso(AUDIO_TX_ENDPOINT, AUDIO_TX_SIZE, 1, tx_event);
	tx_event(NULL);
}

void AudioInputUSB::begin(void)
{
	incoming_count = 0;
	incoming_left = NULL;
	incoming_right = NULL;
	ready_left = NULL;
	ready_right = NULL;
	receive_flag = 0;
	// update_responsibility = update_setup();
	// TODO: update responsibility is tough, partly because the USB
	// interrupts aren't sychronous to the audio library block size,
	// but also because the PC may stop transmitting data, which
	// means we no longer get receive callbacks from usb.c
	update_responsibility = false;
}

static void copy_to_buffers(const uint32_t *src, int16_t *left, int16_t *right, unsigned int len)
{
	uint32_t *target = (uint32_t*) src + len; 
	while ((src < target) && (((uintptr_t) left & 0x02) != 0)) {
		uint32_t n = *src++;
		*left++ = n & 0xFFFF;
		*right++ = n >> 16;
	}

	while ((src < target - 2)) {
		uint32_t n1 = *src++;
		uint32_t n = *src++;
		*(uint32_t *)left = (n1 & 0xFFFF) | ((n & 0xFFFF) << 16);
		left+=2;
		*(uint32_t *)right = (n1 >> 16) | ((n & 0xFFFF0000)) ;
		right+=2;
	}

	while ((src < target)) {
		uint32_t n = *src++;
		*left++ = n & 0xFFFF;
		*right++ = n >> 16;
	}
}

// Called from the USB interrupt when an isochronous packet arrives
// we must completely remove it from the receive buffer before returning
//
#if 1
void usb_audio_receive_callback(unsigned int len)
{
	unsigned int count, avail;
	audio_block_t *left, *right;
	const uint32_t *data;

	AudioInputUSB::receive_flag = 1;
	len >>= 2; // 1 sample = 4 bytes: 2 left, 2 right
	data = (const uint32_t *)rx_buffer;

	count = AudioInputUSB::incoming_count;
	left = AudioInputUSB::incoming_left;
	right = AudioInputUSB::incoming_right;
	if (left == NULL) {
		left = AudioStream::allocate();
		if (left == NULL) return;
		AudioInputUSB::incoming_left = left;
	}
	if (right == NULL) {
		right = AudioStream::allocate();
		if (right == NULL) return;
		AudioInputUSB::incoming_right = right;
	}
	while (len > 0) {
		avail = AUDIO_BLOCK_SAMPLES - count;
		if (len < avail) {
			copy_to_buffers(data, left->data + count, right->data + count, len);
			AudioInputUSB::incoming_count = count + len;
			return;
		} else if (avail > 0) {
			copy_to_buffers(data, left->data + count, right->data + count, avail);
			data += avail;
			len -= avail;
			if (AudioInputUSB::ready_left || AudioInputUSB::ready_right) {
				// buffer overrun, PC sending too fast
				AudioInputUSB::incoming_count = count + avail;
				if (len > 0) {
					usb_audio_overrun_count++;
					printf("!");
					//serial_phex(len);
				}
				return;
			}
			send:
			AudioInputUSB::ready_left = left;
			AudioInputUSB::ready_right = right;
			//if (AudioInputUSB::update_responsibility) AudioStream::update_all();
			left = AudioStream::allocate();
			if (left == NULL) {
				AudioInputUSB::incoming_left = NULL;
				AudioInputUSB::incoming_right = NULL;
				AudioInputUSB::incoming_count = 0;
				return;
			}
			right = AudioStream::allocate();
			if (right == NULL) {
				AudioStream::release(left);
				AudioInputUSB::incoming_left = NULL;
				AudioInputUSB::incoming_right = NULL;
				AudioInputUSB::incoming_count = 0;
				return;
			}
			AudioInputUSB::incoming_left = left;
			AudioInputUSB::incoming_right = right;
			count = 0;
		} else {
			if (AudioInputUSB::ready_left || AudioInputUSB::ready_right) return;
			goto send; // recover from buffer overrun
		}
	}
	AudioInputUSB::incoming_count = count;
}
#endif

void AudioInputUSB::update(void)
{
	audio_block_t *left, *right;

	__disable_irq();
	left = ready_left;
	ready_left = NULL;
	right = ready_right;
	ready_right = NULL;
	uint16_t c = incoming_count;
	uint8_t f = receive_flag;
	receive_flag = 0;
	__enable_irq();
	if (f) {
		int diff = AUDIO_BLOCK_SAMPLES/2 - (int)c;
		feedback_accumulator += diff * 1;
		//uint32_t feedback = (feedback_accumulator >> 8) + diff * 100;
		//usb_audio_sync_feedback = feedback;

		//printf(diff >= 0 ? "." : "^");
	}
	//serial_phex(c);
	//serial_print(".");
	if (!left || !right) {
		usb_audio_underrun_count++;
		//printf("#"); // buffer underrun - PC sending too slow
		if (f) feedback_accumulator += 3500;
	}
	if (left) {
		transmit(left, 0);
		release(left);
	}
	if (right) {
		transmit(right, 1);
		release(right);
	}
}














#define AUDIO_FREQUENCY 48000


bool AudioOutputUSB::update_responsibility;
audio_block_t * AudioOutputUSB::outgoing[AUDIO_CHANNELS]; // being transmitted by USB
audio_block_t * AudioOutputUSB::ready[AUDIO_CHANNELS]; // next in line to be transmitted
uint16_t AudioOutputUSB::offset_1st;
int AudioOutputUSB::normal_target; 
int AudioOutputUSB::accumulator;   
int AudioOutputUSB::subtract;  
/*DMAMEM*/ uint16_t usb_audio_transmit_buffer[AUDIO_TX_SIZE/2] __attribute__ ((used, aligned(32)));


static void tx_event(transfer_t *t)
{
	int len = usb_audio_transmit_callback();
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&tx_transfer, usb_audio_transmit_buffer, len, 0);
	arm_dcache_flush_delete(usb_audio_transmit_buffer, len);
	usb_transmit(AUDIO_TX_ENDPOINT, &tx_transfer);
}


void AudioOutputUSB::begin(void)
{
	update_responsibility = false;
	for (int i =0;i<AUDIO_CHANNELS;i++)
	{
		outgoing[i] = NULL;
		ready[i] = NULL;
	}
	
	// preset sample rate fine-tuning: assumes rate is an integer number of samples per second
	normal_target = (int) (AUDIO_FREQUENCY / 1000); 		// at least this many samples per millisecond 
	accumulator = 500; 										// start half-full
	subtract = (int) AUDIO_FREQUENCY - normal_target*1000;	// accumulate error this fast
}

static void copy_from_buffers(uint32_t *dst, int16_t *Aleft, int16_t *Aright, int16_t *Bleft, int16_t *Bright, int16_t *Cleft, int16_t *Cright, int16_t *Dleft, int16_t *Dright, unsigned int len)
{
	// TODO: optimize...   
	while (len > 0) {
		*dst++ = (*Aright++ << 16) | (*Aleft++ & 0xFFFF);
		*dst++ = (*Bright++ << 16) | (*Bleft++ & 0xFFFF);
		*dst++ = (*Cright++ << 16) | (*Cleft++ & 0xFFFF);
		*dst++ = (*Dright++ << 16) | (*Dleft++ & 0xFFFF);
		len--;
	} 
}

/*
 * On update(), we just receive the audio blocks and keep a set of pointers
 * to them. The USB transmit callback will then copy them to the transmit buffer
 * and release them at some point in the future.
 */
void AudioOutputUSB::update(void)
{
	audio_block_t* chans[AUDIO_CHANNELS];
	int i;
	

	// get the audio data
	for (i=0;i<AUDIO_CHANNELS;i++)
		chans[i] = receiveReadOnly(i);
	
	if (usb_audio_transmit_setting == 0) // not transmitting: dump all audio data
	{
		for (i=0;i<AUDIO_CHANNELS;i++)
		{
			if (NULL != chans[i]) 
				release(chans[i]);
			if (NULL != outgoing[i]) 
			{
				release(outgoing[i]);
				outgoing[i] = NULL;
			}
				
			if (NULL != ready[i]) 
			{
				release(ready[i]);
				ready[i] = NULL;
			}
		}
		offset_1st = 0;
	}
	else
	{
		// ensure every channel has a real audio block, even if it's silent
		for (i=0;i<AUDIO_CHANNELS;i++)
		{
			if (NULL == chans[i]) // sent NULL: make implied silence into real data
			{
				chans[i] = allocate();
				if (NULL != chans[i])
					memset(chans[i]->data, 0, sizeof(chans[i]->data));
				else
					break; // no block available, exit early
			}
		}
		
		if (i >= AUDIO_CHANNELS) // no invalid audio, queue all for transmission
		{
			__disable_irq();
			
			if (NULL == outgoing[0]) // just (re-)starting
			{
				for (i=0;i<AUDIO_CHANNELS;i++)
					outgoing[i] = chans[i];
				offset_1st = 0;
			} 
			else if (NULL == ready[0]) 
			{
				for (i=0;i<AUDIO_CHANNELS;i++)
					ready[i] = chans[i];
			} 
			else 
			{
				// buffer overrun - PC is consuming too slowly
				for (i=0;i<AUDIO_CHANNELS;i++)
				{
					audio_block_t* discard = outgoing[i];
					outgoing[i] = ready[i];
					ready[i] = chans[i];
					release(discard);
				}
				offset_1st = 0; // TODO: discard part of this data?
			}
			__enable_irq();
		}
		else // some invalid audio, can't queue any - discard it all
		{
			for (i=0;i<AUDIO_CHANNELS;i++)
				if (NULL != chans[i])
					release(chans[i]);
			
		}
	}
}


static void interleave_from_blocks(int16_t* transmit_buffer,	//!< next free sample in USB transmit buffer
								audio_block_t** outgoing, //!< array of pointers to source audio blocks
								int chans, //!< number of entries in the array
								int offset, //!< sample# of next "fresh" sample
								int num) //!< number of samples to copy
{
	for (int j=0;j<num;j++)
	{
		for (int i=0;i<chans;i++)
			*transmit_buffer++ = outgoing[i]->data[offset];
		offset++;
	}
}
								
// Called from the USB interrupt when ready to transmit another
// isochronous packet.  If we place data into the transmit buffer,
// the return is the number of bytes.  Otherwise, return 0 means
// no data to transmit
static int count=0;
unsigned int usb_audio_transmit_callback(void)
{
	uint32_t avail, num, target = AudioOutputUSB::normal_target, offset, len=0;

	// adjust target number of samples we want to transmit, if needed
	AudioOutputUSB::accumulator -= AudioOutputUSB::subtract;
	if (AudioOutputUSB::accumulator <= 0) // underflowed
	{
		target++; // need to transmit an extra sample this time
		AudioOutputUSB::accumulator += 1000; // bump accumulator back above threshold
	}
    
	if (++count < 10) {   // TODO: dynamic adjust to match USB rate
		target = 44;
	} else {
		count = 0;
		target = 45;
	}
    
    while (len < target) // may take two iterations if not enough in outgoing[]
	{
		num = target - len; // number of samples left to transmit
		if (NULL == AudioOutputUSB::outgoing[0]) 
		{
			// buffer underrun - PC is consuming too quickly
			memset(usb_audio_transmit_buffer + len, 0, num * AUDIO_CHANNELS * sizeof AudioOutputUSB::outgoing[0]->data[0]);
			//serial_print("%");
			break;
		}
		offset = AudioOutputUSB::offset_1st;

		avail = AUDIO_BLOCK_SAMPLES - offset;
		if (num > avail) num = avail;

		//copy_from_buffers((uint32_t *)usb_audio_transmit_buffer + len,
		//	left->data + offset, right->data + offset, num);
		
		// have to cast type of transmit buffer, because although samples are actually
		// signed integers, some other modules say they're unsigned...
#if 0        
		interleave_from_blocks((int16_t*) usb_audio_transmit_buffer + len,
								AudioOutputUSB::outgoing, AUDIO_CHANNELS, offset,
								num);
#endif
        copy_from_buffers((uint32_t *)usb_audio_transmit_buffer + 4*len,AudioOutputUSB::outgoing[0]->data + offset, AudioOutputUSB::outgoing[1]->data + offset, AudioOutputUSB::outgoing[2]->data + offset, AudioOutputUSB::outgoing[3]->data + offset, AudioOutputUSB::outgoing[4]->data + offset, AudioOutputUSB::outgoing[5]->data + offset, AudioOutputUSB::outgoing[6]->data + offset, AudioOutputUSB::outgoing[7]->data + offset, num);

		len += num;
		offset += num;
		if (offset >= AUDIO_BLOCK_SAMPLES) 
		{
			for (int i=0;i<AUDIO_CHANNELS;i++)
			{
				AudioStream::release(AudioOutputUSB::outgoing[i]);
				AudioOutputUSB::outgoing[i] = AudioOutputUSB::ready[i];
				AudioOutputUSB::ready[i] = NULL;
			}
			AudioOutputUSB::offset_1st = 0;
		} 
		else 
		{
			AudioOutputUSB::offset_1st = offset;
		}
	}
	return target * 16;
}
#endif




struct setup_struct {
  union {
    struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	union {
		struct {
			uint8_t bChannel;  // 0=main, 1=left, 2=right
			uint8_t bCS;       // Control Selector
		};
		uint16_t wValue;
	};
	union {
		struct {
			uint8_t bIfEp;     // type of entity
			uint8_t bEntityId; // UnitID, TerminalID, etc.
		};
		uint16_t wIndex;
	};
	uint16_t wLength;
    };
  };
};

int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen)
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0xA1) { // should check bRequest, bChannel, and UnitID
			if (setup.bCS==0x01) { // mute
				data[0] = AudioInputUSB::features.mute;  // 1=mute, 0=unmute
				*datalen = 1;
				return 1;
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x81) { // GET_CURR
					data[0] = AudioInputUSB::features.volume & 0xFF;
					data[1] = (AudioInputUSB::features.volume>>8) & 0xFF;
				}
				else if (setup.bRequest==0x82) { // GET_MIN
					//serial_print("vol get_min\n");
					data[0] = 0;     // min level is 0
					data[1] = 0;
				}
				else if (setup.bRequest==0x83) { // GET_MAX
					data[0] = FEATURE_MAX_VOLUME;  // max level, for range of 0 to MAX
					data[1] = 0;
				}
				else if (setup.bRequest==0x84) { // GET_RES
					data[0] = 1; // increment vol by by 1
					data[1] = 0;
				}
				else { // pass over SET_MEM, etc.
					return 0;
				}
				*datalen = 2;
				return 1;
			}
	}
	return 0;
}

int usb_audio_set_feature(void *stp, uint8_t *buf) 
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0x21) { // should check bRequest, bChannel and UnitID
			if (setup.bCS==0x01) { // mute
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSB::features.mute = buf[0]; // 1=mute,0=unmute
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSB::features.volume = buf[0];
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
	}
	return 0;
}

           