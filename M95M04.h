/*******************************************************************************

 M95M04 SPI EEPROM library
 -------------------------
 
 M95M04.h - M95M04 class header file
 
 Code written by Stefan Dzisiewski-Smith.
 
 This work is licensed under a MIT license https://opensource.org/licenses/MIT
 
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

*******************************************************************************/

#ifndef M95M04_H
#define M95M04_H

#include <SPI.h>


class M95M04_t
{		
	public:
		M95M04_t();
		void begin(uint8_t CS_pin, uint32_t speed_Hz);
		uint8_t write_byte(uint32_t address, uint8_t value);
		uint8_t read_byte(uint32_t address);
		uint8_t write_array(uint32_t address, uint8_t value_array[], const uint32_t array_length);
		uint8_t read_array(uint32_t address, uint8_t value_array[], const uint32_t array_length);
		static const uint16_t page_size = 512;  // bytes per page
		static const uint16_t num_pages = 1024;
		static const uint32_t num_bytes = (uint32_t)page_size*(uint32_t)num_pages;


	// functions / variables internal to the M95M04_t class - you cannot access these externally

	private:
		// internal helper functions and variables
		// not exposed externally to the user
		uint8_t CS_pin;

		//dummies for SPI transfer
		const uint8_t DUMMY_0 = 0b00000000;
		const uint8_t DUMMY_255 = 0b11111111;

		//Instruction codes
		const uint8_t CMD_WREN  = 0b00000110;  // write enable
		const uint8_t CMD_WRDI  = 0b00000100;  // write disable
		const uint8_t CMD_RDSR  = 0b00000101;  // read status register
		const uint8_t CMD_WRSR  = 0b00000001;  // write status register
		const uint8_t CMD_READ  = 0b00000011;  // read from EEPROM
		const uint8_t CMD_WRITE = 0b00000010;  // write to EEPROM
		const uint8_t CMD_RDID  = 0b10000011;  // read identification page
		const uint8_t CMD_WRID  = 0b10000010;  // write identification page
		const uint8_t CMD_RDLS  = 0b10000011;  // read the identification page lock status
		const uint8_t CMD_LID   = 0b10000010;  // lock the identification page in read only mode
	
		//status register byte addresses
		const uint8_t BIT_WIP   = 0;	 // write in progress
		const uint8_t BIT_WEL   = 1;	 // write enable latch
		const uint8_t BIT_BP0   = 2;	 // block protect 0
		const uint8_t BIT_BP1   = 3;	 // block protect 1
		const uint8_t BIT_SRWD  = 7;	 // status register write disable

		const uint8_t WRITE_TIMEOUT_MS = 10; // a write should only ever take 5 ms max

		//helper functions to compute page / page address in memory
		uint32_t page(uint32_t address);
		uint8_t page_address(uint32_t address);

		//hidden internal functions used to implement the external ones
		void write_enable();
		void write_disable();
		uint8_t read_status_register();


};

extern M95M04_t M95M04;

#endif // M95M04_H