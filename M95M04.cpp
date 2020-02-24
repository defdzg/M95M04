/*******************************************************************************

 M95M04 SPI EEPROM library
 -------------------------
 
 M95M04.cpp - M95M04 class implementation file
 
 Code written by Stefan Dzisiewski-Smith.
 
 Copyright (c) 2016, Stefan Dzisiewski-Smith
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 Mito-L:
 @TIMING control is according to the documentation of the M95M04, for V_cc > 2.5V 

*******************************************************************************/

#include "M95M04.h"
#include <Arduino.h>

M95M04_t::M95M04_t(){
}

void M95M04_t::begin(uint8_t CS_pin, uint32_t speed_Hz){
	this->CS_pin = CS_pin;
	pinMode(this->CS_pin, OUTPUT);
	digitalWrite(this->CS_pin, HIGH);
	SPI.begin();
	// SPI_MODE0 and SPI_MODE3 are both valid - SCK idles low with 0, HIGH with 3
	// idling high reduces power consumption if using a pullup on SCK
	SPI.beginTransaction(SPISettings(speed_Hz, MSBFIRST, SPI_MODE0)); 
}

inline void M95M04_t::write_enable(){
	//@TIMING: t_CHSL > 30ns, t_SLCH > 30ns
	digitalWrite(CS_pin, LOW);
	//@TIMING: t_DVCH > 10ns
	SPI.transfer(CMD_WREN);
	//@TIMING: t_CHDX > 10ns
	digitalWrite(CS_pin, HIGH);
	//@TIMING: t_SHSL > 40ns
}

inline void M95M04_t::write_disable(){
	digitalWrite(CS_pin, LOW);
	SPI.transfer(CMD_WRDI);
	digitalWrite(CS_pin, HIGH);
}

uint8_t read_status_register(){
	digitalWrite(CS_pin, LOW);
	SPI.transfer(CMD_RDSR);
	digitalWrite(CS_pin, HIGH);
}


uint8_t M95M04_t::write_byte(uint32_t address, uint8_t value){

	// returns 0 is the write was successful
	// returns 1 if the write timed out and hence was not successful

	unsigned long start_time_ms;
	uint8_t scratch;

	digitalWrite(this->CS_pin, LOW);
	SPI.transfer(CMD_RDSR);
	start_time_ms = millis();	

	do{
		scratch = SPI.transfer(0); // dummy to clock out data
	} while ((bitRead(scratch, BIT_WIP) == 1) && (millis() < start_time_ms + WRITE_TIMEOUT_MS));
	digitalWrite(this->CS_pin, HIGH); 

	if(bitRead(scratch, BIT_WIP) == 1){ // if we're still busy writing, something has gone wrong;
		return(1); // timeout
	}

	digitalWrite(this->CS_pin, LOW);
	SPI.transfer(CMD_WREN);
	digitalWrite(this->CS_pin, HIGH); 
	digitalWrite(this->CS_pin, LOW);
	SPI.transfer16((CMD_WRITE<<8) | (0x01 & (address >> 16))); // write instruction + top bit of 17-bit address
	SPI.transfer16(address & 0xFFFF); // bottom 16 bits of 17-bit address
	SPI.transfer(value);
	digitalWrite(this->CS_pin, HIGH);

	return(0); // success
}

uint8_t M95M04_t::read_byte(uint32_t address){

	// returns the value of the requested byte if the read was successful
	// returns 0 if the read timed out and was not successful (ambiguous)

	unsigned long start_time_ms;
	uint8_t scratch;

	digitalWrite(this->CS_pin, LOW);
	SPI.transfer(CMD_RDSR);
	start_time_ms = millis();	

	do{
		scratch = SPI.transfer(0); // dummy to clock out data
	} while ((bitRead(scratch, BIT_WIP) == 1) && (millis() < start_time_ms + WRITE_TIMEOUT_MS));
	digitalWrite(this->CS_pin, HIGH); 

	if(bitRead(scratch, BIT_WIP) == 1){ // if we're still busy writing, something has gone wrong;
		return(0); // timeout
	}

	digitalWrite(this->CS_pin, LOW);
	SPI.transfer16((CMD_READ<<8) | (0x01 & (address >> 16))); // write instruction + top bit of 17-bit address
	SPI.transfer16(address & 0xFFFF); // bottom 16 bits of 8-bit address
	scratch = SPI.transfer(0x00); // dummy payload to clock data out
	digitalWrite(this->CS_pin, HIGH);

	return(scratch);
}

uint8_t M95M04_t::write_array(uint32_t address, uint8_t value_array[], const uint32_t array_length){

	// returns 0 is the write was successful
	// returns 1 if the write timed out and hence was not successful

	unsigned long start_time_ms;
	uint8_t scratch;
	uint32_t i; // page counter
	uint32_t j = address; // byte address counter
	uint32_t page_start_address;

	for(i=page(address); i<=page(address+array_length-1); i++){	

		if(i == page(address)){
			page_start_address = address; // for the first page, start at the given address in case we're dropping in to the middle of a page
		} else {
			page_start_address = i*page_size; // else we start at the beginning of the current page
		}

		digitalWrite(this->CS_pin, LOW);
		SPI.transfer(CMD_RDSR);
		start_time_ms = millis();	

		do{
			scratch = SPI.transfer(0); // dummy to clock out data
		} while ((bitRead(scratch, BIT_WIP) == 1) && (millis() < start_time_ms + WRITE_TIMEOUT_MS));
		digitalWrite(this->CS_pin, HIGH); 

		if(bitRead(scratch, BIT_WIP) == 1){ // if we're still busy writing, something has gone wrong;
			return(1); // timeout
		}

		digitalWrite(this->CS_pin, LOW);
		SPI.transfer(CMD_WREN);
		digitalWrite(this->CS_pin, HIGH); 
		digitalWrite(this->CS_pin, LOW);
		SPI.transfer16((CMD_WRITE<<8) | (0x01 & (page_start_address >> 16))); // write instruction + top bit of 17-bit address
		SPI.transfer16(page_start_address & 0xFFFF); // bottom 16 bits of 17-bit address
		do{
			SPI.transfer(value_array[j-address]);
		} while(page_address(++j)!=0);
		digitalWrite(this->CS_pin, HIGH);
	}

	return(0); // success
}

uint8_t M95M04_t::read_array(uint32_t address, uint8_t value_array[], const uint32_t array_length){

	// returns 0 is the read was successful
	// returns 1 if the read timed out and hence was not successful

	unsigned long start_time_ms = millis();
	uint8_t scratch;
	uint32_t j; // byte address counter

	digitalWrite(this->CS_pin, LOW);
	SPI.transfer(CMD_RDSR);
	start_time_ms = millis();	

	do{
		scratch = SPI.transfer(0); // dummy to clock out data
	} while ((bitRead(scratch, BIT_WIP) == 1) && (millis() < start_time_ms + WRITE_TIMEOUT_MS));
	digitalWrite(this->CS_pin, HIGH); 

	if(bitRead(scratch, BIT_WIP) == 1){ // if we're still busy writing, something has gone wrong;
		return(1); // timeout
	}

	digitalWrite(this->CS_pin, LOW);
	SPI.transfer16((CMD_READ<<8) | (0x01 & (address >> 16))); // write instruction + top bit of 17-bit address
	SPI.transfer16(address & 0xFFFF); // bottom 16 bits of 17-bit address
	for(j=address; j<address+array_length; j++){
		value_array[j-address] = SPI.transfer(0x00); // dummy payload to clock data out
	}
	digitalWrite(this->CS_pin, HIGH);

	return(0);
}

uint32_t M95M04_t::page(uint32_t address){
	if(address < (uint32_t)page_size){
		return(0);
	} else {
		return(address / page_size);
	}
}

uint8_t M95M04_t::page_address(uint32_t address){

	return(address % page_size);

}


M95M04_t M95M04 = M95M04_t();
