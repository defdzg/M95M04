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

/**
 * @brief initializes the EEPROM module:
 * sets the pins, writes CS pin high and starts the SPI transaction
 */
void M95M04_t::begin(uint8_t CS_pin, uint8_t WP_pin, uint8_t HOLD_pin, uint32_t speed_Hz){
	this->CS_pin = CS_pin;
	this->WP_pin = WP_pin;
	this->HOLD_pin = HOLD_pin;
	pinMode(this->CS_pin, OUTPUT);
	pinMode(this->WP_pin, OUTPUT);
	pinMode(this->HOLD_pin, OUTPUT);
	digitalWrite(this->CS_pin, HIGH);
	digitalWrite(this->WP_pin, HIGH);
	digitalWrite(this->HOLD_pin, HIGH);
	SPI.begin();
	// SPI_MODE0 and SPI_MODE3 are both valid - SCK idles LOW with 0, HIGH with 3
	// idling high reduces power consumption if using a pullup on SCK
	SPI.beginTransaction(SPISettings(speed_Hz, MSBFIRST, SPI_MODE0));
}

/**
 * @brief writes an 8-bit value in the byte "address"
 * 
 * returns 0 if the write was successful
 * returns 1 if the write timed out and hence was not successful
 * 
 */
uint8_t M95M04_t::write_byte(uint32_t address, uint8_t value){

	if (check_WIP()==0)
	{
		write_enable();
		uint16_t instruction1, instruction2;
		form_instructions(CMD_READ, address, &instruction1, &instruction2);

		digitalWrite(CS_pin, LOW);
		SPI.transfer16(instruction1);
		SPI.transfer16(instruction2);
		SPI.transfer(value);
		digitalWrite(CS_pin, HIGH);
		return(0); // success
	}
	else
	{
		return(1); //something has gone wrong
	}
	
}

/**
 * @brief reads an 8-bit value from the byte "address"
 * 
 * returns the value of the requested byte if the read was successful
 * returns 0 if the read timed out and was not successful (ambiguous)
 * 
 */
uint8_t M95M04_t::read_byte(uint32_t address){

	if (check_WIP()==0)
	{
		uint16_t instruction1, instruction2;
		form_instructions(CMD_READ, address, &instruction1, &instruction2);

		digitalWrite(CS_pin, LOW);
		SPI.transfer16(instruction1);
		SPI.transfer16(instruction2);
		uint8_t scratch = SPI.transfer(DUMMY8_0); // dummy payload to clock data out
		digitalWrite(CS_pin, HIGH);

		return(scratch); // success
	}
	else
	{
		return(0); //something has gone wrong
	}
}


/**
 * @brief writes an array of 8-bit values starting from the byte "address"
 * switches to the next page automatically if necessary
 * 
 * returns 0 if the write was successful
 * returns 1 if the write timed out and hence was not successful
 * returns 2 if the end of memory array is reached
 * 
 */
uint8_t M95M04_t::write_array(uint32_t address, uint8_t * value_array, const uint32_t array_length){

	uint32_t i; 					// page counter
	uint32_t j = address; 			// byte address counter
	uint32_t page_start_address;

	for(i=page(address); i<=page(address+array_length-1); i++){	

		if (i>=num_pages){
			return(2); //eeprom out of memory
		}

		// for the first page, start at the given address in case we're 
		//    dropping in to the middle of a page
		// else we start at the beginning of the current page
		if(i == page(address)){
			page_start_address = address;
		} else {
			page_start_address = i*page_size_bytes;
		}

		if (check_WIP()==0)
		{

			write_enable();
			uint16_t instruction1, instruction2;
			form_instructions(CMD_WRITE, page_start_address, &instruction1, &instruction2);

			digitalWrite(CS_pin, LOW);
			SPI.transfer16(instruction1);
			SPI.transfer16(instruction2);
			do{
				SPI.transfer(value_array[j-address]);
			} while(page_address(++j)!=0);
			digitalWrite(CS_pin, HIGH);
		}
		else
		{
			return(1);
		}
	}

	return(0); // success
}

/**
 * @brief reads an array of 8-bit values starting from the byte "address"
 * switches to the next page automatically if necessary
 * 
 * returns 0 if reading was successful
 * returns 1 if reading timed out and hence was not successful
 * 
 */
uint8_t M95M04_t::read_array(uint32_t address, uint8_t * value_array, const uint32_t array_length){

	if (check_WIP()==0)
	{
		uint16_t instruction1, instruction2;
		form_instructions(CMD_READ, address, &instruction1, &instruction2);

		digitalWrite(CS_pin, LOW);
		SPI.transfer16(instruction1);
		SPI.transfer16(instruction2);
		for(int j = 0; j < array_length; j++){
			value_array[j] = SPI.transfer(DUMMY8_0); // dummy payload to clock data out
		}
		digitalWrite(CS_pin, HIGH);

		return(0);
	}
	else
	{
		return(1); //something has gone wrong
	}
}


/**
 * @brief writes an 8-bit value in the byte "address" on the ID page
 * 
 * returns 0 if the write was successful
 * returns 1 if the write timed out and hence was not successful
 * 
 */
uint8_t M95M04_t::write_idpage_byte(uint32_t address, uint8_t value){
	if (address > page_size_bytes) {
		return(0); //address outside the ID page
	}
	if (check_WIP()==0)
	{
		write_enable();
		uint16_t instruction1, instruction2;
		form_instructions(CMD_WRID, address, &instruction1, &instruction2);

		digitalWrite(CS_pin, LOW);
		SPI.transfer16(instruction1);
		SPI.transfer16(instruction2);
		SPI.transfer(value);
		digitalWrite(CS_pin, HIGH);
		return(0); // success
	}
	else
	{
		return(1); //something has gone wrong
	}
}

/**
 * @brief reads an 8-bit value from the byte address on the ID page
 * 
 * returns the value of the requested byte if the read was successful
 * returns 0 if the read timed out and was not successful (ambiguous) or 
 * 	 if the address to read was outside of the ID page
 * 
 */
uint8_t M95M04_t::read_idpage_byte(uint32_t address){
	if (address > page_size_bytes) {
		return(0); //address outside the ID page
	}
	if (check_WIP()==0)
	{
		uint16_t instruction1, instruction2;
		form_instructions(CMD_RDID, address, &instruction1, &instruction2);

		digitalWrite(CS_pin, LOW);
		SPI.transfer16(instruction1);
		SPI.transfer16(instruction2);
		uint8_t scratch = SPI.transfer(DUMMY8_0); // dummy payload to clock data out
		digitalWrite(CS_pin, HIGH);

		return(scratch); // success
	}
	else
	{
		return(0); //something has gone wrong
	}
}

/**
 * @brief writes an array of 8-bit values starting from the byte address, to the ID page
 * 
 * returns 0 if the write was successful
 * returns 1 if the write timed out and hence was not successful
 * 
 */
uint8_t M95M04_t::write_idpage_array(uint32_t address, uint8_t * value_array, const uint32_t array_length){
	if (address + array_length > page_size_bytes) {
		return(0); //last address outside the ID page, also insures that adress bit A10 is 0
	}
	if (check_WIP()==0)
	{
		uint16_t instruction1, instruction2;
		form_instructions(CMD_WRID, address, &instruction1, &instruction2);

		digitalWrite(CS_pin, LOW);
		SPI.transfer16(instruction1);
		SPI.transfer16(instruction2);
		for(int j = 0; j < array_length; j++){
			SPI.transfer(value_array[j]);
		}
		digitalWrite(CS_pin, HIGH);

		return(0);
	}
	else
	{
		return(1); //something has gone wrong
	}
}

/**
 * @brief reads an array of 8-bit values starting from the byte address on the ID page
 * 
 * returns 0 if reading was successful
 * returns 1 if reading timed out and hence was not successful or 
 * 	 if the last address to read was outside of the ID page
 * 
 */
uint8_t M95M04_t::read_idpage_array(uint32_t address, uint8_t * value_array, const uint32_t array_length){
	if (address + array_length > page_size_bytes) {
		return(0); //address outside the ID page, also insures that adress bit A10 is 0
	}
	if (check_WIP()==0)
	{
		uint16_t instruction1, instruction2;
		form_instructions(CMD_RDID, address, &instruction1, &instruction2);

		digitalWrite(CS_pin, LOW);
		SPI.transfer16(instruction1);
		SPI.transfer16(instruction2);
		for(int j = 0; j < array_length; j++){
			value_array[j] = SPI.transfer(DUMMY8_0); // dummy payload to clock data out
		}
		digitalWrite(CS_pin, HIGH);

		return(0);
	}
	else
	{
		return(1); //something has gone wrong
	}
}

/**
 * @brief returns the lock status of the id page
 * returns 1 if the page is locked 
 * returns 0 if the page is not locked
 */
uint8_t M95M04_t::read_idpage_lock_status(){
	uint16_t instruction1, instruction2;
	uint32_t address = 0x400; //just the A10 bit is 1
	form_instructions(CMD_RDLS, address, &instruction1, &instruction2);

	digitalWrite(CS_pin, LOW);
	SPI.transfer16(instruction1);
	SPI.transfer16(instruction2);
	uint8_t scratch = SPI.transfer(DUMMY8_0);
	digitalWrite(CS_pin, HIGH);

	return(scratch & 0b00000001); //the LSB of the received data is the lock status
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

/**
 * @brief check the Write In Progress bit of the status register
 * 
 * this bit should never be longer than 5ms in use, and it should
 * be checked before any writing operation, 
 * because if this bit is HIGH, 
 * another writing operation is in progress 
 * and the new one cannot be processed
 * 
 * returns 0 if the write is finished before the timeout
 * returns 1 if the write timed out and hence was not successful
 */
uint8_t M95M04_t::check_WIP(){
	unsigned long start_time_ms;
	uint8_t scratch;

	digitalWrite(CS_pin, LOW);
	SPI.transfer(CMD_RDSR);
	start_time_ms = millis();

	do{
		scratch = SPI.transfer(DUMMY8_0); // dummy to clock out data
	} while ((bitRead(scratch, BIT_WIP) == 1) && (millis() < start_time_ms + WRITE_TIMEOUT_MS));
	digitalWrite(CS_pin, HIGH); 

	if(bitRead(scratch, BIT_WIP) == 1){ // if we're still busy writing, something has gone wrong;
		return(1); // timeout
	}
	else
	{
		return(0);
	}
}

/** 
 * @brief sets values of the BP0 and BP1 bits in status register,
 * puts software-write-protection on the corresponding addresses of memory
 * lock_level = 0: whole memory is free to write
 * lock_level = 1: upper quater is write protected
 * lock_level = 2: upper half is write protected
 * lock_level = 3: whole memory is write protected
 */
uint8_t M95M04_t::software_lock_memory(uint8_t lock_level){
	if (lock_level < 0 || lock_level > 3){
		return(1); //lock_level out of boundary
	}

	write_enable();
	digitalWrite(CS_pin, LOW);
	SPI.transfer(CMD_WRSR);
	SPI.transfer(lock_level<<BIT_BP0);
	digitalWrite(CS_pin, HIGH);	

	return(0);
}

/** 
 * @brief sets values of the SRWD, BP0 and BP1 bits in status register,
 * then drives WP_pin low.
 * Puts EEPROM in hardware-protected mode (HPM)
 * lock_level = 0: whole memory is free to write
 * lock_level = 1: upper quater is write protected
 * lock_level = 2: upper half is write protected
 * lock_level = 3: whole memory is write protected
 */
uint8_t M95M04_t::hardware_lock_memory(uint8_t lock_level){
	if (lock_level < 0 || lock_level > 3){
		return(1); //lock_level out of boundary
	}

	write_enable();
	digitalWrite(CS_pin, LOW);
	SPI.transfer(CMD_WRSR);
	SPI.transfer(1<<BIT_SRWD + lock_level<<BIT_BP0);
	digitalWrite(CS_pin, HIGH);

	digitalWrite(WP_pin, LOW);
	
	return(0);	
}
/** 
 * @brief drives WP_pin high, 
 * then sets values of the SRWD, BP0 and BP1 bits in status register to 0
 * enables writing to memory array and status register
 */
uint8_t M95M04_t::unlock_memory(){
	digitalWrite(WP_pin, HIGH);

	write_enable();
	digitalWrite(CS_pin, LOW);
	SPI.transfer(CMD_WRSR);
	SPI.transfer((uint8_t)0);
	digitalWrite(CS_pin, HIGH);	

	return(0);
}

/** 
 * @brief returns the number of the page of the given address
 * 
 */
uint32_t M95M04_t::page(uint32_t address){
	if(address < (uint32_t)page_size_bytes){
		return(0);
	} else {
		return(address / page_size_bytes);
	}
}

/** 
 * @brief returns the local address on the page from the given address
 * 
 */
uint8_t M95M04_t::page_address(uint32_t address){
	return(address % page_size_bytes);
}

/** 
 * @brief builds 2 16-bit commands to be sent on the MOSI line
 * 
 * @param command uint8_t, the instruction code
 * @param address uint32_t, address
 * @param instr1 * uint16_t, pointer to the first instruction holder
 * @param instr2 * uint16_t, pointer to the second instruction holder
 */
void M95M04_t::form_instructions(
		uint8_t command,
		uint32_t address,
		uint16_t * instr1,
		uint16_t * instr2)
{
	// read instruction code + 3 bits of the upper address byte
	*instr1 =  ((uint16_t)command << 8) | (0x07 & (address >> 16));
	// middle and lower address bytes
	*instr2 = address & 0xFFFF;
}


M95M04_t M95M04 = M95M04_t();
