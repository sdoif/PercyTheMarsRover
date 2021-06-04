#include <stdio.h>
#include "system.h"

#include "I2C_core.h"
#include "terasic_includes.h"
#include "mipi_camera_config.h"
#include "mipi_bridge_config.h"

#include "altera_up_avalon_accelerometer_spi.h"
#include "altera_avalon_timer_regs.h"
#include "altera_avalon_timer.h"
#include "altera_avalon_pio_regs.h"

#include "auto_focus.h"

#include "sys/alt_irq.h"
#include <stdlib.h>
#include <stdarg.h>

#include <fcntl.h>
#include <unistd.h>

//EEE_IMGPROC defines
#define EEE_IMGPROC_MSG_START_R ('R'<<16 | 'B'<<8 | 'B')
#define EEE_IMGPROC_MSG_START_G ('G'<<16 | 'B'<<8 | 'B')
#define EEE_IMGPROC_MSG_START_B ('B'<<16 | 'B'<<8 | 'B')
#define EEE_IMGPROC_MSG_START_V ('V'<<16 | 'B'<<8 | 'B')
#define EEE_IMGPROC_MSG_START_Y ('Y'<<16 | 'B'<<8 | 'B')

//offsets
#define EEE_IMGPROC_STATUS 0
#define EEE_IMGPROC_MSG 1
#define EEE_IMGPROC_ID 2
#define EEE_IMGPROC_BBCOL 3

#define EXPOSURE_INIT 0x010000	//0x002000
#define EXPOSURE_STEP 0x100 // increment of exposure - mapped to an input in the code
#define GAIN_INIT 0x250 //0x080
#define GAIN_STEP 0x040 // increment of gain - also mapped to an input
#define DEFAULT_LEVEL 3

#define MIPI_REG_PHYClkCtl		0x0056
#define MIPI_REG_PHYData0Ctl	0x0058
#define MIPI_REG_PHYData1Ctl	0x005A
#define MIPI_REG_PHYData2Ctl	0x005C
#define MIPI_REG_PHYData3Ctl	0x005E
#define MIPI_REG_PHYTimDly		0x0060
#define MIPI_REG_PHYSta			0x0062
#define MIPI_REG_CSIStatus		0x0064
#define MIPI_REG_CSIErrEn		0x0066
#define MIPI_REG_MDLSynErr		0x0068
#define MIPI_REG_FrmErrCnt		0x0080
#define MIPI_REG_MDLErrCnt		0x0090

	// Given functions

void mipi_clear_error(void){
	MipiBridgeRegWrite(MIPI_REG_CSIStatus,0x01FF); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLSynErr,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_FrmErrCnt,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLErrCnt, 0x0000); // clear error

  	MipiBridgeRegWrite(0x0082,0x00);
  	MipiBridgeRegWrite(0x0084,0x00);
  	MipiBridgeRegWrite(0x0086,0x00);
  	MipiBridgeRegWrite(0x0088,0x00);
  	MipiBridgeRegWrite(0x008A,0x00);
  	MipiBridgeRegWrite(0x008C,0x00);
  	MipiBridgeRegWrite(0x008E,0x00);
  	MipiBridgeRegWrite(0x0090,0x00);
}

void mipi_show_error_info(void){

	alt_u16 PHY_status, SCI_status, MDLSynErr, FrmErrCnt, MDLErrCnt;

	PHY_status = MipiBridgeRegRead(MIPI_REG_PHYSta);
	SCI_status = MipiBridgeRegRead(MIPI_REG_CSIStatus);
	MDLSynErr = MipiBridgeRegRead(MIPI_REG_MDLSynErr);
	FrmErrCnt = MipiBridgeRegRead(MIPI_REG_FrmErrCnt);
	MDLErrCnt = MipiBridgeRegRead(MIPI_REG_MDLErrCnt);
	printf("PHY_status=%xh, CSI_status=%xh, MDLSynErr=%xh, FrmErrCnt=%xh, MDLErrCnt=%xh\r\n", PHY_status, SCI_status, MDLSynErr,FrmErrCnt, MDLErrCnt);
}

void mipi_show_error_info_more(void){
    printf("FrmErrCnt = %d\n",MipiBridgeRegRead(0x0080));
    printf("CRCErrCnt = %d\n",MipiBridgeRegRead(0x0082));
    printf("CorErrCnt = %d\n",MipiBridgeRegRead(0x0084));
    printf("HdrErrCnt = %d\n",MipiBridgeRegRead(0x0086));
    printf("EIDErrCnt = %d\n",MipiBridgeRegRead(0x0088));
    printf("CtlErrCnt = %d\n",MipiBridgeRegRead(0x008A));
    printf("SoTErrCnt = %d\n",MipiBridgeRegRead(0x008C));
    printf("SynErrCnt = %d\n",MipiBridgeRegRead(0x008E));
    printf("MDLErrCnt = %d\n",MipiBridgeRegRead(0x0090));
    printf("FIFOSTATUS = %d\n",MipiBridgeRegRead(0x00F8));
    printf("DataType = 0x%04x\n",MipiBridgeRegRead(0x006A));
    printf("CSIPktLen = %d\n",MipiBridgeRegRead(0x006E));
}

bool MIPI_Init(void){
	bool bSuccess;


	bSuccess = oc_i2c_init_ex(I2C_OPENCORES_MIPI_BASE, 50*1000*1000,400*1000); //I2C: 400K
	if (!bSuccess)
		printf("failed to init MIPI- Bridge i2c\r\n");

    usleep(50*1000);
    MipiBridgeInit();

    usleep(500*1000);

    MipiCameraInit();
    MIPI_BIN_LEVEL(DEFAULT_LEVEL);

 	usleep(1000);

	return bSuccess;
}

	// Custom functions/classes
typedef struct{
	int distance, x_coord, y_coord;
	bool seen1, seen2; // if see in the 1st or 2nd scan
} Ball;

bool is_ball(int topleft, int bottomright){
		// x_min
	int top_left_x = topleft && 0xffff0000; // extracting the top 16 bits
		// y_min
	int top_left_y = topleft && 0x0000ffff;
		// x_max
	int bottom_right_x = bottomright && 0xffff0000;
		// y_max
	int bottom_right_y = bottomright && 0x0000ffff;

	int height = top_left_y - bottom_right_y;
	int length = bottom_right_x - top_left_x;

	// If the length is within 10% of the height, we can consider this as a proper box
	if (length < height * 1.1 && length > height * 0.9){ // ADJUST PARAMETERS HERE
		return TRUE;
	}else{
		return FALSE;
	}
}

bool is_in_range(int topleft, int bottomright){
		// x_min
	int top_left_x = topleft && 0xffff0000; // extracting the top 16 bits
		// y_min
	//int top_left_y = topleft && 0x0000ffff;
		// x_max
	int bottom_right_x = bottomright && 0xffff0000;
		// y_max
	//int bottom_right_y = bottomright && 0x0000ffff;

	int middle_x = (bottom_right_x - top_left_x) / 2;
	// middle x-axis pixel = 320
	if (middle_x < 400 && middle_x > 240){
		return TRUE;
	}else{
		return FALSE;
	}
}

bool distance_check_z1(int distance){
	if (distance <= 100 && distance >= 20){
		return TRUE;
	}else{
		return FALSE;
	}
}

// TODO - move processing of x-coordinates into main since both distance and angle use it
//		so you're basically processing the same information again
int distance_calc(int topleft, int bottomright){
	// x_min
	float top_left_x = topleft && 0xffff0000; // extracting the top 16 bits
	// y_min
	//int top_left_y = topleft && 0x0000ffff;
	// x_max
	float bottom_right_x = bottomright && 0xffff0000;
	// y_max
	//int bottom_right_y = bottomright && 0x0000ffff;

		// D = (W*F)/P
	// W = diameter of the ball
	float W = 3.95;
	// F = Focal length
	float F = 700;
	// P = apparent width in pixels
	float P = bottom_right_x - top_left_x;
	// D = Distance from camera
	float D = (W*F)/P;
	int D_int = D;
	return D_int;
}

int angle_calc(int topleft, int bottomright){
	// x_min
	int top_left_x = topleft && 0xffff0000;
	// x_max
	int bottom_right_x = bottomright && 0xffff0000;
	int middle_x = bottom_right_x - top_left_x;

	// resolution width = 640
	// field of view (measured = 50 degrees)
	float ang_per_pix = 50/640;
	return ang_per_pix * (middle_x - 320); // difference in pixels of the object from the centre
}

int main()
{
	printf("\n");
	/* Accelerometer code
//	int x_read;
//	int y_read;
//	int z_read;
//	printf("Attempting to establish connection to accelerometer\n");
//	alt_up_accelerometer_spi_dev * acc_dev;
//	acc_dev = alt_up_accelerometer_spi_open_dev("/dev/accelerometer_spi");
//
////	if (acc_dev == NULL){
////	    printf("Unable to open connection to accelerometer\n");
////	    return 1;
////	}
//
//	while (1) {
//		usleep(1000000);
//	    alt_up_accelerometer_spi_read_x_axis(acc_dev, &x_read);
//	    alt_up_accelerometer_spi_read_y_axis(acc_dev, &y_read);
//	    alt_up_accelerometer_spi_read_z_axis(acc_dev, &z_read);
//	    printf("x : %i, y : %i, z : %i \n", x_read, y_read, z_read);
//	}
	*/

	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

	printf("DE10-LITE D8M VGA Demo\n");
	printf("Imperial College EEE2 Project version\n");
	IOWR(MIPI_PWDN_N_BASE, 0x00, 0x00);
	IOWR(MIPI_RESET_N_BASE, 0x00, 0x00);

	usleep(2000);
	IOWR(MIPI_PWDN_N_BASE, 0x00, 0xFF);
	usleep(2000);
	IOWR(MIPI_RESET_N_BASE, 0x00, 0xFF);

	printf("Image Processor ID: %x\n",IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_ID));
	//printf("Image Processor ID: %x\n",IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_ID)); //Don't know why this doesn't work - definition is in system.h in BSP

	usleep(2000);

	// MIPI Init
	printf("Initialising MIPI bridge");
	if (!MIPI_Init()){
		printf("MIPI_Init Init failed!\r\n");
	}else{
		printf("MIPI_Init Init successfully!\r\n");
	}

	usleep(2000);

 	mipi_clear_error();
	usleep(50*1000);
 	mipi_clear_error();
	usleep(1000*1000);
	mipi_show_error_info();
	printf("\n");

    //////////////////////////////////////////////////////////
    alt_u16 bin_level = DEFAULT_LEVEL;
    alt_u8  manual_focus_step = 10;
    alt_u16  current_focus = 300;
    int boundingBoxColour = 0;
    alt_u32 exposureTime = EXPOSURE_INIT;
    alt_u16 gain = GAIN_INIT;

    OV8865SetExposure(exposureTime);
    OV8865SetGain(gain);
    Focus_Init();

	// 	Opening connection to UART preliminaries
	printf("Opening connection to UART\n");
	FILE* fp;
	char prompt = 0;
	int j = 0;

	/* UART Connection while loop - not working
	while(1){
		fp = fopen("/dev/uart", "r+");
		if(fp){
			printf("Connection made\n");
			fprintf(fp, "Test");
			usleep(1000000);
			fclose(fp);
		}else{
			printf("Unable to connect\n");
		}
	}
	*/

	/* Working UART Connection loop
	while(1){
		j++;
		printf("Try No. %i\n", j);
		fp = fopen("/dev/uart", "r+");
		if(fp){
			//printf("Opened connection to UART\n");
			while(1){
				fprintf(fp, "test");
				printf("Sent test\n");
			}
			fclose(fp);
		}else{
			printf("Unable to connect to UART\n");
		}
		fclose(fp);
	//	usleep(1000000);
	}
	*/

	/* Alternate UART Connection attempts
	//alt_up_rs232_write();
	////while(1){
	//	IOWR(UART_BASE, 0, 0b000000101);
	//	printf("Sent 101\n");
	//	usleep(1000000);
	//	IOWR(UART_BASE, 0, 0b000000110);
	//	printf("Sent 110\n");
	//	usleep(1000000);
	//}
	//fprintf(fp, "Test");
	//printf("Test should have been printed");
	//
	//if(fp){
	//	printf("Opened connection to UART");
	//}
	*/

	/* Receiving test
	while(1){
		int test = IORD(UART_BASE,0);
		printf("Received %i\n", test);
	}
	*/

		// Declarations
	// Measurement related
	int r_topleft, g_topleft, b_topleft, v_topleft, y_topleft;
	int r_bottomright, g_bottomright, b_bottomright, v_bottomright, y_bottomright;
	int distance;

	// Other
	int state; // or stage

  	while(1){
        // touch KEY0 to trigger Auto focus
	    if((IORD(KEY_BASE,0)&0x03) == 0x02){
    		current_focus = Focus_Window(320,240);
        }
	   	// touch KEY1 to ZOOM
		if((IORD(KEY_BASE,0)&0x03) == 0x01){
	      	if(bin_level == 3 )bin_level = 1;
	      	else bin_level ++;
	      	printf("set bin level to %d\n",bin_level);
	      	MIPI_BIN_LEVEL(bin_level);
	      	usleep(500000);
	    }

       	//Read messages from the image processor and print them on the terminal
       	while ((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8) & 0xff) { 	//Find out if there are words to read
           	//Get next word from message buffer
			int word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

			// Decide what state we are in from here - assign value to the variable state

    	   	if (word == EEE_IMGPROC_MSG_START_R){ // If the incoming string == RBB
				// Print on a newline
				printf("\n");
				r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				distance = distance_calc(r_topleft, r_bottomright);
				printf("Red distance : %i cm", distance);
    	   	} else if (word == EEE_IMGPROC_MSG_START_G){ // If the incoming string == GBB
				printf("\n");
				g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				distance = distance_calc(g_topleft, g_bottomright);
				printf("Green distance : %i cm", distance);
    	   	} else if (word == EEE_IMGPROC_MSG_START_B){ // If the incoming string == BBB
				printf("\n");
				b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				distance = distance_calc(b_topleft, b_bottomright);
				printf("Blue distance : %i cm", distance);
    	   	} else if (word == EEE_IMGPROC_MSG_START_V){ // If the incoming string == VBB
				printf("\n");
				v_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				v_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				distance = distance_calc(v_topleft, v_bottomright);
				printf("Violet distance : %i cm", distance);
    	   	} else if (word == EEE_IMGPROC_MSG_START_Y){ // If the incoming string == YBB
				printf("\n");
				y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				distance = distance_calc(y_topleft, y_bottomright);
				printf("Yellow distance : %i cm", distance);
    	   	}
    	   	//printf("%08x ",word);

			/*
			if (word == 5390914) // red box
			if (word == 4670018) // green box
			if (word == 4342338) // blue box
			if (word == 5653058) // violet box
			if (word == 5849666) // yellow box
			*/
    		}

       //Update the bounding box colour
       boundingBoxColour = ((boundingBoxColour + 1) & 0xff);
       IOWR(EEE_IMGPROC_0_BASE, EEE_IMGPROC_BBCOL, (boundingBoxColour << 8) | (0xff - boundingBoxColour));

       //Process input commands
       int in = getchar();
       switch (in) {
       	   case 'e': {
       		   exposureTime += EXPOSURE_STEP;
       		   OV8865SetExposure(exposureTime);
       		   printf("\nExposure = %x ", exposureTime);
       	   	   break;}
       	   case 'd': {
       		   exposureTime -= EXPOSURE_STEP;
       		   OV8865SetExposure(exposureTime);
       		   printf("\nExposure = %x ", exposureTime);
       	   	   break;}
       	   case 't': {
       		   gain += GAIN_STEP;
       		   OV8865SetGain(gain);
       		   printf("\nGain = %x ", gain);
       	   	   break;}
       	   case 'g': {
       		   gain -= GAIN_STEP;
       		   OV8865SetGain(gain);
       		   printf("\nGain = %x ", gain);
       	   	   break;}
       	   case 'r': {
        	   current_focus += manual_focus_step;
        	   if(current_focus >1023) current_focus = 1023;
        	   OV8865_FOCUS_Move_to(current_focus);
        	   printf("\nFocus = %x ",current_focus);
       	   	   break;}
       	   case 'f': {
        	   if(current_focus > manual_focus_step) current_focus -= manual_focus_step;
        	   OV8865_FOCUS_Move_to(current_focus);
        	   printf("\nFocus = %x ",current_focus);
       	   	   break;}
       }


	   //Main loop delay
	   usleep(10000);

   };
  return 0;
}
