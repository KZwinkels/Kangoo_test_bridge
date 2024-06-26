/* firmware functionality */

//#define DISABLE_CAN3			//if CAN3 is not connected, enable this
//#define SHORT_MODE				//for very fast can logging, reduce the amount of missed messages

/* variables */
volatile	uint8_t		output_can_to_serial	= 0;	//mirror all received CAN messages on bus 0-2 to serial
volatile	uint8_t		repeat_can				= 0;	//repeat CAN1 to CAN2 and vice versa (transparent bridge mode)