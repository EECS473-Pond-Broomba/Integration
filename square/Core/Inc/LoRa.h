/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: 
 *  (1) "AS IS" WITH NO WARRANTY; 
 *  (2) TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, HopeRF SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) HopeRF
 *
 * website: www.HopeRF.com
 *          www.HopeRF.cn   
 */

/*! 
 * file       HopeDuino_LoRa.h
 * brief      driver for RFM92/95/96/98
 * hardware   HopeDuino with HopeRF's LoRa COB rf-module
 *            
 *
 * version    1.0
 * date       Jun 3 2014
 * author     QY Ruan
 */

#ifndef Lora_h
	#define Lora_h
	
	#ifndef F_CPU
		#define F_CPU 16000000UL		//for delay.h to use, & HopeDuino use 16MHz Xo
	#endif	

	#ifndef	byte
		typedef unsigned char byte;
	#endif
	
	#ifndef word
		typedef unsigned int  word;
	#endif
	
	#ifndef lword
		typedef unsigned long lword;
	#endif

	#ifndef __DELAY_BACKWARD_COMPATIBLE__
		#define __DELAY_BACKWARD_COMPATIBLE__
	#endif 

	#include "main.h"

	
	/** Hardware brief **/
	/** POR & DIO0 is must be connect **/
	//PORTB						//DDRx		PORTx		PINx
	#define DIO0	0x01		// 0		  0          1
	#define	POR		0x02		// 0          0          1

	//PORTD
	/** DIO1~DIO4 can be select to connect **/
	#define	DIO4	0x10		// 0          0          1
	#define	DIO3	0x20		// 0          0          1
	#define	DIO2    0x40		// 0          0          1
	#define	DIO1    0x80		// 0          0          1

	#ifdef RISHABH
		#define	DIO0_PORT GPIOA
		#define DIO0_PIN GPIO_PIN_1
		#define RST_PORT GPIOA
		#define RST_PIN GPIO_PIN_0
	#else
		#define	DIO0_PORT GPIOA
		#define DIO0_PIN GPIO_PIN_1
		#define RST_PORT GPIOA
		#define RST_PIN GPIO_PIN_0
	#endif
	//Output
//	#define POROut()	(GPIOC->MODER[3] |= 1)              //Set POR as an output
//	#define	PORIn()		(GPIOC->MODER[3] &= 0) 			//Set POR as an input
//	#define SetPOR()	(GPIOC[1] |= 1)				//Set POR high
//	#define	ClrPOR()	(GPIOC[1] &= 0)				//Set POR low
	
	//Input
//	#define DIO0In()	(GPIOC->MODER[3] &= (~DIO0))			//Set DIO1 as an input and DIO02-7 as output
//	#define	DIO0_H()	((GPIOC[12]&GPIOC[11]&GPIOC[10]&GPIOA[15]&GPIOC[0]&DIO0)==DIO0)
//	#define	DIO0_L()	((GPIOC[12]&GPIOC[11]&GPIOC[10]&GPIOA[15]&GPIOC[0]&DIO0)==0)
	
	typedef union  
	{
	 struct
	 	{
	  	byte FreqL: 8;
	  	byte FreqM: 8;
	  	byte FreqH: 8;
	  	byte FreqX: 8;
	 	}freq;
	 unsigned long Freq;
	}FreqStruct;

	enum modulationType {OOK, FSK, GFSK, LORA};
	
	enum moduleType {RFM92, RFM93, RFM95, RFM96, RFM97, RFM98};
	
	enum sfType {SF6, SF7, SF8, SF9, SF10, SF11, SF12};
	
	enum bwType {BW62K, BW125K, BW250K, BW500K};	//
	
	enum crType {CR4_5, CR4_6, CR4_7, CR4_8};
	
	class loraClass
	{
	public:	
	 	modulationType Modulation;					//OOK/FSK/GFSK/LORA
	 	moduleType COB;								//Chip on board
	 	
	 	//common parameter
	 	lword Frequency;							//unit: KHz
 		byte  OutputPower;							//unit: dBm   range: 2-20 [2dBm~+20dBm] 
 		word  PreambleLength;						//unit: byte
		
		bool  FixedPktLength;						//OOK/FSK/GFSK:
 		                                            //	 	true-------fixed packet length
 		                                            //   	false------variable packet length
 		                                            //LoRa:
 		                                            //	 	true-------implicit header mode
 		                                            //      false------explicit header mode
		
		bool  CrcDisable;							//OOK/FSK/GFSK:
													//		true-------CRC disable
													//		fasle------CRC enable with CCITT
													//LoRa:
													//		true-------Header indicates CRC off
													//		false------Header indicates CRC on
		byte  PayloadLength;						//PayloadLength is need to be set a value, when FixedPktLength is true. 		


 		//for OOK/FSK/GFSK parameter
 		lword SymbolTime;							//unit: ns
 		lword Devation;								//unit: KHz
 		word  BandWidth;							//unit: KHz
 		byte  SyncLength;							//unit: none, range: 1-8[Byte], value '0' is not allowed!
 		byte  SyncWord[8];

		//for LoRa parameter
		sfType SFSel;								//unit: none, range: SF6~SF12
		bwType BWSel;
		crType CRSel;
		
		void vInitialize(void);
		void vConfig(void);
		void vGoRx(void);
		void vGoStandby(void);
		void vGoSleep(void);
		bool bSendMessage(byte msg[], byte length);
		byte bGetMessage(byte msg[]);
		byte bSpiTransfer(byte dat);
		void vSpiWrite(word dat);
		byte bSpiReadByte();
		byte bSpiRead(byte addr);
		void vSpiBurstWrite(byte addr, byte ptr[], byte length);
		void vSpiBurstRead(byte addr, byte ptr[], byte length);
		void POROut(void);
		void PORIn(void);
		bool DIO0_H();
		bool DIO0_L();
		
	private:
	 	FreqStruct FrequencyValue;
	 	word BitRateValue;
		word DevationValue;
		byte BandWidthValue;
		byte SFValue;
		byte BWValue;
		byte CRValue;
		bool RsOptimize;
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		void vReset(void);
		byte bSelectBandwidth(byte rx_bw);
		byte bSelectRamping(lword symbol);
	};
#else
	#warning "LoRa.h have been defined!"

#endif
