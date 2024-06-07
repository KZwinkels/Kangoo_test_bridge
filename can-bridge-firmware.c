#include "can-bridge-firmware.h"

//example valid CAN frame
//volatile	can_frame_t	static_message		= {.can_id = 0x5BC, .can_dlc = 8, .data = {0,0,0,0,0,0,0,0}};
//volatile	can_frame_t engineering_request = {.can_id = 0x79B, .can_dlc = 8, .data = {0x02, 0x21, 0xFF, 0x50, 0x50, 0x50, 0x50, 0x50}};
//can_frame_t	continue_request	= {.can_id = 0x79B, .can_dlc = 8, .data = {0x30, 0x01, 0x05, 0x50, 0x50, 0x50, 0x50, 0x50}};

//USB variables
volatile	uint8_t		configSuccess		= false;	//tracks whether device successfully enumerated
static		FILE		USBSerialStream;				//fwrite target for CDC
volatile	uint8_t		signature[11];					//signature bytes

//variables for ProcessCDCCommand()
volatile	int16_t		cmd, cmd2;
volatile	uint16_t	i = 0, j = 0, k = 0;
volatile	uint32_t	temp;
volatile	uint16_t	ReportStringLength;
char *		ReportString;

volatile	uint8_t		can_busy			= 0;		//tracks whether the can_handler() subroutine is running
volatile	uint8_t		print_char_limit	= 0;		//serial output buffer size

//timer variables
volatile	uint8_t		ten_sec_timer		= 1;		//increments on every sec_timer underflow
volatile	uint16_t	sec_timer			= 1;		//actually the same as ms_timer but counts down from 1000
volatile	uint8_t		sec_interrupt		= 0;		//sigals main loop to output debug data every second
volatile	uint16_t	ms_timer			= 0;		//increments on every TCC0 overflow (ever ms)
volatile	uint8_t		ten_ms_timer		= 1;		//triggers send_frame
volatile	uint8_t		hun_ms_timer		= 1;		//triggers send_frame
volatile	uint16_t	min_timer			= 0;		//increments every 60.000ms 
			uint16_t	hour_timer			= 0;		//increments every 60 minutes
volatile	uint8_t		charge_flag			= 0;		//signals charge or balancing state is active

//hierzo tantoe grote array cell spanningen
			
//array with all module voltages:
volatile	uint16_t	V_modules[16];

volatile	uint16_t	V_cellen[96];

volatile	uint16_t	V_cell_low			= 0;

volatile	uint16_t	V_cell_high			= 0;

volatile	uint16_t	V_cell_diff			= 0;

volatile	uint32_t	V_total				= 0;

volatile	int32_t		Amperage			= 0;

volatile	uint16_t	SOC					= 0;

volatile	uint8_t		SOCframe425			= 0;

volatile	uint8_t		V_frame425_8b		= 0;

volatile	uint16_t	V_frame425_16b		= 0;

volatile	uint8_t		V_frame445_b4		= 0;

volatile	uint8_t		V_frame445_b5		= 0;

volatile	uint8_t		balancingflag		= 0;

volatile	uint8_t		module_counter		= 0;


//can_frames to be sent to the kangoo
volatile	can_frame_t frame_155  = {.can_id = 0x155, .can_dlc = 8, .data = {0x00,0x87,0xC9,0x54,0x70,0xC0,0x02,0xFF}};
volatile	can_frame_t frame_424  = {.can_id = 0x424, .can_dlc = 8, .data = {0x11,0x40,0x1E,0xAF,0x30,0x48,0xAA,0x31}};
volatile	can_frame_t frame_425  = {.can_id = 0x425, .can_dlc = 8, .data = {0x2A,0x65,0x44,0x9C,0x42,0x58,0x01,0x2B}};
volatile	can_frame_t frame_445  = {.can_id = 0x445, .can_dlc = 7, .data = {0x40,0x55,0x55,0x63,0x2C,0x95,0x00}};
volatile	can_frame_t frame_659  = {.can_id = 0x659, .can_dlc = 4, .data = {0x15,0x00,0x20,0x71}};
	
can_frame_t setup_frame1 = {.can_id = 0x598, .can_dlc = 8, .data = {0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00}};
can_frame_t setup_frame2 = {.can_id = 0x598, .can_dlc = 8, .data = {0x01,0x02,0x01,0x00,0x00,0x00,0x00,0x00}};
can_frame_t setup_frame3 = {.can_id = 0x598, .can_dlc = 8, .data = {0x02,0x01,0x08,0x00,0x00,0x00,0x00,0x00}};
can_frame_t setup_frame4 = {.can_id = 0x598, .can_dlc = 8, .data = {0x02,0x02,0x08,0x00,0x00,0x00,0x00,0x00}};

	
									// frame_155 = {.can_id = 0x155, .can_dlc = 8, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

	
//because the MCP25625 transmit buffers seem to be able to corrupt messages (see errata), we're implementing
//our own buffering. This is an array of frames-to-be-sent, FIFO. Messages are appended to buffer_end++ as they
//come in and handled according to buffer_pos until buffer_pos == buffer_end, at which point both pointers reset
//the buffer size should be well in excess of what this device will ever see
can_frame_t tx0_buffer[TXBUFFER_SIZE];
uint8_t		tx0_buffer_pos		= 0;
uint8_t		tx0_buffer_end		= 0;

can_frame_t tx2_buffer[TXBUFFER_SIZE];
uint8_t		tx2_buffer_pos		= 0;
uint8_t		tx2_buffer_end		= 0;

can_frame_t tx3_buffer[TXBUFFER_SIZE];
uint8_t		tx3_buffer_pos		= 0;
uint8_t		tx3_buffer_end		= 0;

uint32_t	whitelist			= 0xFFFFFFFF;
uint8_t		whitelistar[7]		= {0,0,0,0,0,0,0};


void hw_init(void){
	uint8_t caninit;

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, 48000000);
	
	//turn off everything we don' t use
	PR.PRGEN		= PR_AES_bm | PR_RTC_bm | PR_DMA_bm;
	PR.PRPA			= PR_ADC_bm | PR_AC_bm;
	PR.PRPC			= PR_TWI_bm | PR_USART0_bm | PR_HIRES_bm;
	PR.PRPD			= PR_TWI_bm | PR_USART0_bm | PR_TC0_bm | PR_TC1_bm;
	PR.PRPE			= PR_TWI_bm | PR_USART0_bm;
	
	//blink output
	PORTB.DIRSET	= 3;
	
	//start 16MHz crystal and PLL it up to 48MHz
	OSC.XOSCCTRL	= OSC_FRQRANGE_12TO16_gc |		//16MHz crystal
	OSC_XOSCSEL_XTAL_16KCLK_gc;						//16kclk startup
	OSC.CTRL	   |= OSC_XOSCEN_bm;				//enable crystal
	while(!(OSC.STATUS & OSC_XOSCRDY_bm));			//wait until ready
	OSC.PLLCTRL		= OSC_PLLSRC_XOSC_gc | 2;		//XTAL->PLL, 2x multiplier (32MHz)
	OSC.CTRL	   |= OSC_PLLEN_bm;					//start PLL
	while (!(OSC.STATUS & OSC_PLLRDY_bm));			//wait until ready
	CCP				= CCP_IOREG_gc;					//allow changing CLK.CTRL
	CLK.CTRL		= CLK_SCLKSEL_PLL_gc;			//use PLL output as system clock
	
	//output 16MHz clock to MCP25625 chips (PE0)
	//next iteration: put this on some other port, pin 4 or 7, so we can use the event system
	TCE0.CTRLA		= TC0_CLKSEL_DIV1_gc;						//clkper/1
	TCE0.CTRLB		= TC0_CCAEN_bm | TC0_WGMODE_SINGLESLOPE_bm;	//enable CCA, single-slope PWM
	TCE0.CCA		= 1;										//compare value
	TCE0.PER		= 1;										//period of 1, generates 24MHz output
	
	PORTE.DIRSET	= PIN0_bm;									//set CLKOUT pin to output
	
	//setup CAN pin interrupts
	PORTC.INTCTRL	= PORT_INT0LVL_HI_gc;
	PORTD.INTCTRL	= PORT_INT0LVL_HI_gc | PORT_INT1LVL_HI_gc;
	
	PORTD.INT0MASK	= PIN0_bm;						//PORTD0 has can1 interrupt
	PORTD.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	PORTD.INT1MASK	= PIN5_bm;						//PORTD5 has can2 interrupt
	PORTD.PIN5CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	
	#ifndef DISABLE_CAN3
	PORTC.INT0MASK	= PIN2_bm;						//PORTC2 has can3 interrupt
	PORTC.PIN0CTRL	= PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	#endif
	
	//buffer checking interrupt
	TCC1.CTRLA		= TC0_CLKSEL_DIV1_gc;			//48M/1/4800 ~ 100usec
	TCC1.PER		= 4800;
	TCC1.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//same priority as can interrupts
	
	//we want to optimize performance, so we're going to time stuff
	//48MHz/48=1us timer, which we just freerun and reset whenever we want to start timing something
	//frame time timer
	TCC0.CTRLA		= TC0_CLKSEL_DIV1_gc;			//clock select (prescaler set to 1)
	TCC0.PER		= 48000;						//48MHz/48000=1ms
	TCC0.INTCTRLA	= TC0_OVFINTLVL_HI_gc;			//interrupt on overflow
	
	
	PORTB.OUTCLR	= (1 << 0);
	
	can_system_init:
	
	//Init SPI and CAN interface:
	if(RST.STATUS & RST_WDRF_bm){ //if we come from a watchdog reset, we don't need to setup CAN
		caninit = can_init(MCP_OPMOD_NORMAL, 1); //on second thought, we do
		} else {
		caninit = can_init(MCP_OPMOD_NORMAL, 1);
	}
	
	if(caninit){
		PORTB.OUTSET |= (1 << 0);					//green LED
		} else {
		PORTB.OUTSET |= (1 << 1);					//red LED
		_delay_ms(10);
		goto can_system_init;
	}
	
	//Set and enable interrupts with round-robin
	XMEGACLK_CCP_Write((void * ) &PMIC.CTRL, PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm);//PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm| PMIC_HILVLEN_bm;
	
	USB_Init(USB_OPT_RC32MCLKSRC | USB_OPT_BUSEVENT_PRILOW);
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);
	
	wdt_enable(WDTO_15MS);
	
	sei();
	
	//send settings to BMS
	//make sure to convert back to can 2 and 3 before testing code!
	_delay_ms(1000);
	send_can2(setup_frame1);
	send_can2(setup_frame2);
	send_can2(setup_frame3);
	send_can2(setup_frame4);
	
	send_can3(setup_frame1);
	send_can3(setup_frame2);
	send_can3(setup_frame3);
	send_can3(setup_frame4);
}


int main(void){
	char * str = "           ";
	hw_init();

	while(1){
		if(!output_can_to_serial){
			if(sec_interrupt){
				sec_interrupt = 0;
				
				//sample text output every second
				str = "ms 00000\n";
				int_to_5digit(ms_timer, (char *) (str + 3));
				print(str,11);
				//send_can3(frame_155);
			}
		}
	}
}

/* services commands received over the virtual serial port */
void ProcessCDCCommand(void)
{
	uint8_t it = 0, it2 = 0;
	ReportStringLength = 0;
	cmd = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
	
	if(cmd > -1){
		switch(cmd){
			case 0: //reset when sending 0x00
			case 90: //'Z' - also reset when typing a printable character (fallback for serial terminals that do not support sending non-printable characters)
			_delay_ms(1000);
			CCP				= CCP_IOREG_gc;			//allow changing CLK.CTRL
			RST.CTRL		= RST_SWRST_bm;			//perform software reset
			break;
			case 64: //@ - dump all CAN messages to USB
			output_can_to_serial = 1 - output_can_to_serial;
			break;
			case 255: //send ident
			ReportString = "MUXSAN CAN bridge\n"; ReportStringLength = 18;
			break;
			case 119: //whitelist - w00001F2
			cmd2 = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
			it = 0;
			while(cmd2 > -1){
				if((cmd2 > 47) && (cmd2 < 71)){
					whitelistar[it++] = (uint8_t) cmd2;
				}
				cmd2 = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
			}
			whitelist = 0;
			for(it2 = it; it2 > 0; it2--){ //assuming hex values
				if(whitelistar[it2 - 1] < 58){
					whitelist += ((whitelistar[it2 - 1] - 48) << ((it - it2) * 4));
					} else {
					whitelist += ((whitelistar[it2 - 1] - 55) << ((it - it2) * 4));
				}
			}
			ReportString = "Showing only 0x0000000\n";
			EID_to_str(ReportString + 15,whitelist);
			ReportStringLength = 23;
			break;
			default: //when all else fails
			ReportString = "Unrecognized Command:   \n"; ReportStringLength = 25;
			ReportString[22] = cmd;
			break;
		}
		if(ReportStringLength){
			print(ReportString, ReportStringLength);
		}
	}
}

// Event handler for the LUFA library USB Disconnection event.
void EVENT_USB_Device_Disconnect(void){}

void EVENT_USB_Device_Connect(void){}

// Event handler for the LUFA library USB Configuration Changed event.
void EVENT_USB_Device_ConfigurationChanged(void){ configSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface); }

// Event handler for the LUFA library USB Control Request reception event.
void EVENT_USB_Device_ControlRequest(void){	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface); }

//appends string to ring buffer and initiates transmission
void print(char * str, uint8_t len){
	if((print_char_limit + len) <= 120){
		fwrite(str, len, 1, &USBSerialStream);
		print_char_limit += len;
		} else { //if the buffer is full, show that by sending an X (happens on very busy CAN buses)
		fwrite("X\n",2,1,&USBSerialStream);
	}
}

//fires every 1ms
ISR(TCC0_OVF_vect){
	wdt_reset();
	ms_timer++;
	
	
	//handle print buffer
	if(print_char_limit <= 64) { print_char_limit = 0; }
	else { print_char_limit -= 64; }
	
	ten_ms_timer--;
	sec_timer--;
	
	//fires every 10ms
	if(ten_ms_timer == 0){
		
		V_total = 0;
		for (int i = 1; i < 17; i++) {
			V_total = V_total + V_modules[i];		//adding V_modules so you get V_total
		}
		//niet vergeten aan te passen voor 2 accupakketen parallel 
		
		SOC = (V_total - 60397) + ((Amperage * 2) / 5);	
		frame_155.data[4] = (uint8_t) (SOC >> 8);							//SOC toekennen aan frame_155 [6] en [7]
		frame_155.data[5] = (uint8_t) (SOC & 0xFF);
		
		V_total = V_total * 171 / 35000 ;
		frame_155.data[6] = (uint8_t) (V_total >> 8);						//V_total toekennen aan frame_155 [6] en [7] 
		frame_155.data[7] = (uint8_t) (V_total & 0xFF);
		
		Amperage = ((Amperage * 48) / 1000) + 34768;
		frame_155.data[1] = (uint8_t) (Amperage >> 8);						//Amperage toekennen aan frame_155 [1] en [2]
		frame_155.data[2] = (uint8_t) (Amperage & 0xFF);

		send_can1(frame_155);
		
		ten_ms_timer = 10;
		hun_ms_timer--;
	
		//fires every 100ms
		if(hun_ms_timer == 0){
			send_can1(frame_424);
			//doe iets met die flag!!!
			SOCframe425 = SOC / 286;
			frame_425.data[1] = (uint8_t) (SOCframe425 & 0xFF);
			
			if((V_total - 339) < 256){
			V_frame425_8b = (uint8_t) (V_total - 339);										//V_total is 16b gaat dat goed? 
			}																				//misschien makkelijk een V_total van type int8_t aan te maken
			frame_425.data[5] = (uint8_t) (V_frame425_8b & 0xFF);
			
			V_frame425_16b = (V_total / 2) - 85;
			frame_425.data[6] = (uint8_t) (V_frame425_16b >> 8);						//Amperage toekennen aan frame_155 [1] en [2]
			frame_425.data[7] = (uint8_t) (V_frame425_16b & 0xFF);
			
			send_can1(frame_425);
			
			if(((V_total / 2) - 339) < 256){						
				V_frame445_b4 = (uint8_t)((V_total / 2) - 339);								
			}																				//alles omschrijven want V_total is nu 8 bit ipv 16 
			frame_445.data[4] = (uint8_t) (V_frame445_b4 & 0xFF);
			
			if(((V_total / 4) - 86) < 256){										
				V_frame445_b5 = (uint8_t)((V_total / 4) - 86);											//V_total is 16b gaat dat goed?
			}																				//misschien makkelijk een V_total van type int8_t aan te maken
			frame_445.data[5] = (uint8_t) (V_frame445_b5 & 0xFF);
			
			if(Amperage < -256){													//geen idee of 256 een goeie vergelijk waarde is
				frame_445.data[6] = 0x80;
			}
			else{frame_445.data[6] = 0x00;}
			
			send_can1(frame_445);
			
			hun_ms_timer = 10;					
		}
	}
	
	if(sec_timer == 500){
		frame_424.data[6] = 0x55;			//byte shift from 55 to AA and back every second
		frame_445.data[2] = 0x55;			//byte shift from 55 to AA and back every second
	}
	//fires every second
	if(sec_timer == 0){
		PORTB.OUTCLR = (1 << 1);
		sec_timer = 1000;
		sec_interrupt = 1;
		ten_sec_timer--;
		frame_424.data[6] = 0xAA;
		frame_445.data[2] = 0xAA;
		
		//fires every 3 seconds
		if(ten_sec_timer == 0){
			send_can1(frame_659);
			ten_sec_timer = 3;
		}
	}
	if(ms_timer == 60000){
		ms_timer = 0;
		min_timer++;
		V_cell_high = 0;
		V_cell_low  = 65000;
		/*
		if (balancing = 1){
			for (int i = 0; i < 96; i++) {
				if (V_cellen[i] < V_cell_low){
					V_cell_low = V_cellen[i]
				}
				if (V_cellen[i] > V_cell_high){
					V_cell_high = V_cellen[i]
				}
			}
			V_cell_diff = V_cell_high - V_cell_low
			
			
			
			if (V_cell_diff < 66){
				balancing = 0
			}
		}
		

		if (min_timer == 60){
			min_timer = 0;
			hour_timer++;
		}
	}
	
	//eventueel hierzo aantal waardes op nul zetten???
	*/
	
	if(!can_busy) ProcessCDCCommand();
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
	USB_USBTask();
}



//fires approx. every 100us
ISR(TCC1_OVF_vect){
	check_can1();
	check_can2();
	check_can3();
}

//can1 interrupt
ISR(PORTD_INT0_vect){
	can_busy = 1;
	can_handler(1);
}

//can2 interrupt
ISR(PORTD_INT1_vect){
	can_busy = 1;
	can_handler(2);
}

//can3 receive interrupt
ISR(PORTC_INT0_vect){
	can_busy = 1;
	can_handler(3);
}

//VCM side of the CAN bus (in Muxsan)
void can_handler(uint8_t can_bus){
	can_frame_t frame;
	uint8_t		temp_8;
	uint16_t	temp_16;
	int32_t		temp_32;
	
	#ifdef SHORT_MODE
	char strbuf[] = "                   \n"
	#else
	char strbuf[] = "1|       |                \n";
	if(can_bus == 2){ strbuf[0] = 50; }
	if(can_bus == 3){ strbuf[0] = 51; }
	#endif
	
	uint8_t flag = can_read(MCP_REG_CANINTF, can_bus);
	
	if (flag & (MCP_RX0IF | MCP_RX1IF)){
		
		if(flag & MCP_RX0IF){
			can_read_rx_buf(MCP_RX_0, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX0IF, 0x00, can_bus);
			} else {
			can_read_rx_buf(MCP_RX_1, &frame, can_bus);
			can_bit_modify(MCP_REG_CANINTF, MCP_RX1IF, 0x00, can_bus);
		}
		if(can_bus == 2){							//400V-battery No. 1
			switch(frame.can_id){
				
				case 0x600:
					temp_8 = frame.data[0];   //string number
					temp_16 = frame.data[1];  //block numberf
					
					if(temp_8 == 'tobedetermined' || temp_8 == 'tobedetermined'){		//zet V_module in de array 
						 if (temp_16 >= 0x01 && temp_16 <= 0x08) {
							 V_modules[module_counter] = frame.data[2] << 8) | frame.data[3];
							 module_counter++;
					}				
					if(module_counter == 15){								//bereken V_total als van 16 modules de spanning is ontvangen
						for (int i = 0; i < 16; i++) {					
							V_total += V_modules[i];
							V_total = V_total * 24.4414 / 10000;			//omrekenen naar fysieke waarde 
							print(V_total,27);								//totaalspanning doorsturen over USB
							uint32_to_str									//dit toepassen voor het doorsturen van de spanning over USB
							V_total = 0;
					}
					break;
					
				case 0x602:
					temp_32 = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | frame.data[7];
					Amperage = temp_32;			
					break;
				default:
					break;		
					
			}
		}
		
		
		switch(frame.can_id){
			//sample CAN message modification routine

			default:
			EID_to_str(strbuf + 2, frame.can_id);
			canframe_to_str(strbuf + 10, frame);
			print(strbuf,27);
			break;
		}
		
		
		//if you enable CAN repeating between bus 1 and 2, we end up here
		if(repeat_can){
			//you can blacklist certain messages or message contents like this, blocking them from both being forwarded and being displayed
			uint8_t blacklist = 0;
			switch(frame.can_id){
				//case 0x59E:
				//blacklist = 1;
				//break;
				default:
				blacklist = 0;
				break;
			}
			if(!blacklist){
				if(can_bus == 1){send_can2(frame);} else {send_can1(frame);}
				
				if(output_can_to_serial){
					if(whitelist < 0x1FFFFFFF){
						if(frame.can_id == whitelist){
							EID_to_str(strbuf + 2, frame.can_id);
							canframe_to_str(strbuf + 10, frame);
							print(strbuf,27);
						}
					} 
					/*else {
						#ifdef SHORT_MODE
						EID_to_str(strbuf, frame.can_id);
						canframe_to_str(strbuf + 3, frame);
						print(strbuf,20);
						#else
						EID_to_str(strbuf + 2, frame.can_id);
						canframe_to_str(strbuf + 10, frame);
						print(strbuf,27);
						#endif
					}*/
				}
			}
		}
	}
	
	
	if(flag & 0xA0){
		uint8_t flag2 = can_read(MCP_REG_EFLG, can_bus);
		if(flag2 & 0xC0){
			can_write(MCP_REG_EFLG, 0, can_bus); //reset all errors
			ReportString = "CANX RX OVF\n";
			ReportString[3] = 48 + can_bus;
			print(ReportString,12);
		}
		if(flag2 > 0){ PORTB.OUTSET = (1 << 1); }
		if(flag & 0xE0){ can_bit_modify(MCP_REG_CANINTF, (flag & 0xE0), 0x00, can_bus);	}
	}
	can_busy = 0;
}

void send_can(uint8_t can_bus, can_frame_t frame){
	if(can_bus == 1) send_can1(frame);
	if(can_bus == 2) send_can2(frame);
	if(can_bus == 3) send_can3(frame);
}

void send_can1(can_frame_t frame){
	//put in the buffer
	memcpy(&tx0_buffer[tx0_buffer_end++], &frame, sizeof(frame));
	
	if(tx0_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx0_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can1();
}

void check_can1(void){
	uint8_t reg;
	
	if(tx0_buffer_end != tx0_buffer_pos){
		//check if TXB0 is free use
		reg = can1_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can1_load_txbuff(0, (can_frame_t *) &tx0_buffer[tx0_buffer_pos++]);
			can1_rts(0);
			if(tx0_buffer_pos == tx0_buffer_end){ //end of buffer, reset
				tx0_buffer_end = 0;
				tx0_buffer_pos = 0;
			}
		}
	}
}

void send_can2(can_frame_t frame){
	//put in the buffer
	memcpy(&tx2_buffer[tx2_buffer_end++], &frame, sizeof(frame));
	
	if(tx2_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx2_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can2();
}

void check_can2(void){
	uint8_t reg;
	
	if(tx2_buffer_end != tx2_buffer_pos){
		//check if TXB0 is free use
		reg = can2_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can2_load_txbuff(0, (can_frame_t *) &tx2_buffer[tx2_buffer_pos++]);
			can2_rts(0);
			if(tx2_buffer_pos == tx2_buffer_end){ //end of buffer, reset
				tx2_buffer_end = 0;
				tx2_buffer_pos = 0;
			}
		}
	}
}

void send_can3(can_frame_t frame){
	//put in the buffer
	memcpy(&tx3_buffer[tx3_buffer_end++], &frame, sizeof(frame));
	
	if(tx3_buffer_end >= TXBUFFER_SIZE){ //silently handle buffer overflows
		tx3_buffer_end = TXBUFFER_SIZE - 1;
	}
	
	check_can3();
}

void check_can3(void){
	uint8_t reg;
	
	if(tx3_buffer_end != tx3_buffer_pos){
		//check if TXB0 is free use
		reg = can3_read(MCP_REG_TXB0CTRL);
		
		if(!(reg & MCP_TXREQ_bm)){ //we're free to send
			can3_load_txbuff(0, (can_frame_t *) &tx3_buffer[tx3_buffer_pos++]);
			can3_rts(0);
			if(tx3_buffer_pos == tx3_buffer_end){ //end of buffer, reset
				tx3_buffer_end = 0;
				tx3_buffer_pos = 0;
			}
		}
	}
}

