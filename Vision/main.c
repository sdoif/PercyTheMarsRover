#include <stdio.h>
#include <string.h>
#include "math.h"
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
	char colour;
	int distance;
	float x_coord, y_coord;
	bool seen, seen1, seen2, seen3; // if seen overall and if seen in the 2nd scan
} Ball;

// Analyses if the object is actually the ball or not
bool is_ball(int left_x, int right_x, int left_y, int right_y){
	int height = right_y - left_y;
	//printf("Height : %i ", height);
	int length = right_x - left_x;
	//printf("Length : %i\n", length);

	// If the length is within 10% of the height, we can consider this as a proper box
	if (length < height * 1.2 && length > height * 0.8 && height > 5 && height < 150 && length > 5 && length < 150 ){ // ADJUST PARAMETERS HERE
		return TRUE;
	}
	return FALSE;
}

// Analyses if the ball is close to the centre of the camera frame
bool is_in_centre_range(int left_x, int right_x){
	int middle_x = (right_x + left_x) / 2;
	// middle x-axis pixel = 320
	if (middle_x < 400 && middle_x > 240){
		return TRUE;
	}else{
		return FALSE;
	}
}

// Check if the ball is within accurate distance measurements
bool distance_check_z1(int distance){
	if (distance >= 30 && distance < 80){
		return TRUE;
	}else{
		return FALSE;
	}
}

bool distance_check_z2(int distance){
	if (distance >= 80 && distance <= 200){
		return TRUE;
	}else{
		return FALSE;
	}
}

// Function that returns the distance of the ball from the camera based on its boundary
//		box coordinates
int distance_calc(int left_x, int right_x, int left_y, int right_y){

		// D = (W*F)/P
	// W = diameter of the ball
	float W = 3.95;
	// F = Focal length
	float F = 700;
	// P = apparent width in pixels
	float width = right_x - left_x;
	float height = left_y - right_y;
	float P;
	if (width > height){
		P = width;
	}else{
		P = height;
	}

	//P = (height + width) / 2;
	// D = Distance from camera
	float D = (W*F)/P;
	return abs(D);
}

// Have a while loop that just reads the boundary box coordinates for the ball we are going to
//		and updates just the distance of that Ball_ptr-> Angle calculations are made here too in this while loop.
//		End of the while loop is caused by the distance to the ball being within a specific range. Function returns true
//		After we've gotten to the ball (and measured/set distance for the final time), we update state to 0 to restart the loop

// Function that corrects the angle as we drive towards the ball, returns TRUE once we've
//		reached the ball
bool go_towards(Ball *ball, FILE* fp){
	int verilog_word;
	int distance;

	if(ball->colour == 'R'){
		printf("Going towards red\n");
		while (1) {
			//printf("Started while loop\n");
			// Update the boundary box co-ordinates
			verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_R){ // If the incoming string == RBB
				//printf("Detected the start of red coordinates\n");
				int r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int r_left_x = (r_topleft)>>16; // extracting the top 16 bits
				int r_right_x = (r_bottomright)>>16;

				int r_left_y = (r_topleft & 0x0000ffff);
				int r_right_y = (r_bottomright & 0x0000ffff);

				// Measure angle and correct if needed to
                int middle_pix = (r_right_x + r_left_x) / 2;
                printf("middle pixel : %i\n", middle_pix);
                if (middle_pix < 280){
					fprintf(fp, "v0l!\n");
                }else if(middle_pix > 360){
					fprintf(fp, "v0r!\n");
				}else{
					fprintf(fp, "v0g!\n");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(r_left_x, r_right_x, r_left_y, r_right_y) && is_in_centre_range(r_left_x, r_right_x)){
					distance = distance_calc(r_left_x, r_right_x, r_left_y, r_right_y);
					if (distance < 80 && distance > 30){
						// Update members
						fprintf(fp, "v0s!\n");
						ball->distance = distance;
						ball->seen = TRUE;
						fprintf(fp, "c/r/%i/!\n", distance);
						return TRUE;
					}
				}
    	   	}
		}
	}else if(ball->colour == 'G'){
		printf("Going towards green\n");
		while (1) {
			// Update the boundary box co-ordinates
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_G){ // If the incoming string == RBB
				int g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int g_left_x = (g_topleft)>>16; // extracting the top 16 bits
				int g_right_x = (g_bottomright)>>16;

				int g_left_y = (g_topleft & 0x0000ffff);
				int g_right_y = (g_bottomright & 0x0000ffff);

				// Measure angle and correct if needed to
                int middle_pix = (g_right_x + g_left_x) / 2;
                if (middle_pix < 280){
					fprintf(fp, "v0l!\n");
                }else if(middle_pix > 360){
					fprintf(fp, "v0r!\n");
				}else{
					fprintf(fp, "v0g!\n");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(g_left_x, g_right_x, g_left_y, g_right_y) && is_in_centre_range(g_left_x, g_right_x)){
					distance = distance_calc(g_left_x, g_right_x, g_left_y, g_right_y);
					if (distance < 60 && distance > 30){
						// Update members
						fprintf(fp, "v0s!\n");
						ball->distance = distance;
						ball->seen = TRUE;
						fprintf(fp, "c/g/%i/!\n", distance);
						return TRUE;
					}
				}
    	   	}
		}
	}else if(ball->colour == 'B'){
		printf("Going towards blue\n");
		while (1) {
			// Update the boundary box co-ordinates
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_B){ // If the incoming string == RBB
				int b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int b_left_x = (b_topleft)>>16; // extracting the top 16 bits
				int b_right_x = (b_bottomright)>>16;

				int b_left_y = (b_topleft & 0x0000ffff);
				int b_right_y = (b_bottomright & 0x0000ffff);

				// Measure angle and correct if needed to
                int middle_pix = (b_right_x + b_left_x) / 2;
                if (middle_pix < 280){
					fprintf(fp, "v0l!\n");
                }else if(middle_pix > 360){
					fprintf(fp, "v0r!\n");
				}else{
					fprintf(fp, "v0g!\n");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(b_left_x, b_right_x, b_left_y, b_right_y) && is_in_centre_range(b_left_x, b_right_x)){
					distance = distance_calc(b_left_x, b_right_x, b_left_y, b_right_y);
					if (distance < 60 && distance > 30){
						fprintf(fp, "v0s!\n");
						ball->distance = distance;
						ball->seen = TRUE;
						fprintf(fp, "c/r/%i/!\n", distance);
						return TRUE;
					}
				}
    	   	}
		}
	}/*else if(ball->colour == 'V'){
		int v_d[5];
		while ((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8) & 0xff) {

			// Update the boundary box co-ordinates
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_V){ // If the incoming string == RBB
				// Print on a newline
				printf("\n");
				int v_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int v_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int v_left_x = (v_topleft)>>16; // extracting the top 16 bits
				int v_right_x = (v_bottomright)>>16;

				int v_left_y = (v_topleft & 0x0000ffff);
				int v_right_y = (v_bottomright & 0x0000ffff);

				// Measure angle and correct if needed to
                int middle_pix = (v_right_x + v_left_x) / 2;
                if (middle_pix < 240){
					fprintf(fp, "0s");
					fprintf(fp, "0l");
                }else if(middle_pix > 400){
					fprintf(fp, "0s");
					fprintf(fp, "0r");
				}else{
					fprintf(fp, "0g");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(v_left_x, v_right_x, v_left_y, v_right_y) && is_in_centre_range(v_left_x, v_right_x)){
					distance = distance_calc(v_left_x, v_right_x, v_left_y, v_right_y);
					if (distance < 50 && distance > 30){
						// Update members
						ball->distance = distance;
						ball->seen = TRUE;
						return TRUE;
					}
				}

    	   	}
		}
	}*/else if(ball->colour == 'Y'){
		printf("Going towards yellow\n");
		while (1) {
			// Update the boundary box co-ordinates
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_Y){ // If the incoming string == RBB
				int y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int y_left_x = (y_topleft)>>16; // extracting the top 16 bits
				int y_right_x = (y_bottomright)>>16;

				int y_left_y = (y_topleft & 0x0000ffff);
				int y_right_y = (y_bottomright & 0x0000ffff);

				// Measure angle and correct if needed to
                int middle_pix = (y_right_x + y_left_x) / 2;
                if (middle_pix < 280){
					fprintf(fp, "v0l!\n");
                }else if(middle_pix > 360){
					fprintf(fp, "v0r!\n");
				}else{
					fprintf(fp, "v0g!\n");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && is_in_centre_range(y_left_x, y_right_x)){
					distance = distance_calc(y_left_x, y_right_x, y_left_y, y_right_y);
					if (distance < 60 && distance > 30){
						fprintf(fp, "v0s!\n");
						// Update members
						ball->distance = distance;
						ball->seen = TRUE;
						fprintf(fp, "c/y/%i/!\n", distance);

						return TRUE;
					}
				}
    	   	}
			// Need to read the next few messages to restart the while loop
			IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
		}
	}
	printf("Reaching return 0\n");
	return 0;
}

int main()
{
	printf("\n");
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

		//	Setting up connection to UART
	// 	Opening connection to UART preliminaries
	printf("Opening connection to UART\n");
	// File pointer point to the UART connection
	FILE* fp;

	// Loop until we have connected to the UART
	while(1){
		fp = fopen("/dev/uart", "r+");
		if(fp){
			printf("Opened connection to UART\n");
			break;
		}else{
			printf("Unable to connect to UART, trying again\n");
		}
	}

		// Declarations
	// Struct/class declarations
	Ball redBall, greenBall, blueBall, yellowBall;
	Ball *redBall_ptr = &redBall;
	Ball *greenBall_ptr = &greenBall;
	Ball *blueBall_ptr = &blueBall;
	//Ball *violetBall_ptr = &violetBall;
	Ball *yellowBall_ptr = &yellowBall;

	redBall_ptr->colour = 'R';
	redBall_ptr->seen = FALSE;
	redBall_ptr->seen1 = FALSE;
	redBall_ptr->seen2 = FALSE;
	redBall_ptr->seen3 = FALSE;

	greenBall_ptr->colour = 'G';
	greenBall_ptr->seen = FALSE;
	greenBall_ptr->seen1 = FALSE;
	greenBall_ptr->seen2 = FALSE;
	greenBall_ptr->seen3 = FALSE;

	blueBall_ptr->colour = 'B';
	blueBall_ptr->seen = FALSE;
	blueBall_ptr->seen1 = FALSE;
	blueBall_ptr->seen2 = FALSE;
	blueBall_ptr->seen3 = FALSE;

//	violetBall_ptr->colour = 'V';
//	violetBall_ptr->seen = FALSE;
//	violetBall_ptr->seen1 = FALSE;
//	violetBall_ptr->seen2 = FALSE;
//	violetBall_ptr->seen3 = FALSE;

	yellowBall_ptr->colour = 'Y';
	yellowBall_ptr->seen = FALSE;
	yellowBall_ptr->seen1 = FALSE;
	yellowBall_ptr->seen2 = FALSE;
	yellowBall_ptr->seen3 = FALSE;

	int balls_detected = 0;

	// Measurement related (Base)
	int r_topleft, g_topleft, b_topleft, v_topleft, y_topleft;
	int r_bottomright, g_bottomright, b_bottomright, v_bottomright, y_bottomright;

	// Measurement related (Derived)
	int r_left_x, g_left_x, b_left_x, v_left_x, y_left_x;
	int r_right_x, g_right_x, b_right_x, v_right_x, y_right_x;
	int r_left_y, g_left_y, b_left_y, v_left_y, y_left_y;
	int r_right_y, g_right_y, b_right_y, v_right_y, y_right_y;

	// Single variable to handle all of the distances
	int distance;

	// 2nd scan specific variables
	int s2_balls_detected = 0;
	Ball nothing;
	Ball *closestBall;
	closestBall = &nothing;
	closestBall->distance = 500;

	// Other
	int state = 0; // or stage
	int state1_array[2];
	int foundbit = 0;

	fprintf(fp, "v0g\n");
	printf("Sent v0g\n");

	// In this loop, we look at what state we are in and perform
	//		the desired actions
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
		int count = 0;
       	//Read messages from the image processor and print them on the terminal
       	while ((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8) & 0xff) {	//Find out if there are words to read
    		count++;
       		//printf("Starting while loop\n");
       		//Get next word from message buffer (Verilog)
			int word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

			// ---------- Analyse incoming string of characters from Control -------------
			// Control input decoding variables
			char theta[10] = "0";
			char x_coordinate[10] = "0";
			char y_coordinate[10] = "0";
			int incomingChar;

			int state1 = 0;
			int state2 = 0;
			float int_theta = 75;
			float int_x = 0;
			float int_y = 0;

			//if((count % 3) == 0){
			//incomingChar = fgetc(fp);
			//printf("Received : %c\n", incomingChar);

			/*
			if (incomingChar == 'v'){ // detected beginning of my info
			   	incomingChar = fgetc(fp); // grab a '/'
				//printf("detected a v\n");
				state1_array[0] = state1_array[1];
			   	state1 = fgetc(fp);
			   	if (state1 == '1'){
			   		printf("State1 set to 1\n");
			   	}
				state1_array[1] = state1;
	        	incomingChar = fgetc(fp); // Grabs a '/'
			   	printf("state1 : %c\n", state1);
	        	state2 = fgetc(fp);
	        	printf("state2 : %c\n", state2);
			//}
	        	incomingChar = fgetc(fp); // Grabs a '/'
	        	//printf("should be a +: %c\n", incomingChar);
			    incomingChar = fgetc(fp); // first theta char
	        	while(incomingChar != '/'){ // read chars and append to string until we reach the +
	        		//printf("incoming theta digit: %c\n", incomingChar);
	        		if((incomingChar < 58 && incomingChar > 47) || incomingChar == 46){
	        			strncat(theta, &incomingChar, 1);
	        		}
	        		incomingChar = fgetc(fp);
			    }
			    //printf("theta: %s\n", theta);
			    int_theta = atof(theta);
			    printf("int theta : %f\n", int_theta);

			    incomingChar = fgetc(fp); // read the first digit
			    while(incomingChar != '/'){ // read chars and append to string until we reach the end of the line
	        		//printf("incoming radius digit: %c\n", incomingChar);
			        if((incomingChar < 58 && incomingChar > 47) || incomingChar == 46){
			        	strncat(x_coordinate, &incomingChar, 1);
			        }
			        incomingChar = fgetc(fp);
	        	}
			    int_x = atof(x_coordinate);
			    printf("int x : %f\n", int_x);


			    incomingChar = fgetc(fp); // read the first digit
			    while(incomingChar != '/'){ // read chars and append to string until we reach the end of the line
			    	//printf("incoming radius digit: %c\n", incomingChar);
			       if((incomingChar < 58 && incomingChar > 47) || incomingChar == 46){
			          	strncat(y_coordinate, &incomingChar, 1);
			       }
			       incomingChar = fgetc(fp);
			    }
			    int_y = atof(y_coordinate);
			    //printf("int y : %f\n", int_y);

			}
			*/
			//}
			//printf("Finished reading info\n");

			// ---------- End of analysing incoming chars from Control ---------

			if ((balls_detected == 4) && state1 == 1){
				// send signal which indicates that all 5 balls have been detected
				foundbit = 1;
				printf("All balls detected\n");
				fprintf(fp, "v%ig\n", foundbit);
			}

			if (state1_array[0] == 0 && state1_array[1] == 1){
				printf("Finished a scan\n");

				//if (redBall_ptr->seen == TRUE)

				fprintf(fp, "v%ig\n", foundbit);
			}

			// Check incoming chars for state changes
			if (state1_array[0] == 1 && state1_array[1] == 0 && state != 2){ // going from a 1 to a 0 indicates that we've gone to state 0/1
				printf("Increment state\n");
				state++;
			}else if(state1_array[0] == 1 && state1_array[1] == 0 && state == 2){
				printf("Reset state to 1st scan\n");
				state = 0;
			}

			//printf("Current state : %i\n", state);
			// Analyse incoming BB information and verilog and make the proper variable assignments
    	   	if (word == EEE_IMGPROC_MSG_START_R){ // If the incoming string == RBB
				r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				r_left_x = (r_topleft)>>16; // extracting the top 16 bits
				r_right_x = (r_bottomright)>>16;
				r_left_y = (r_topleft & 0x0000ffff);
				r_right_y = (r_bottomright & 0x0000ffff);
    	   	} else if (word == EEE_IMGPROC_MSG_START_G){ // If the incoming string == GBB
				g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				g_left_x = (g_topleft)>>16; // extracting the top 16 bits
				g_right_x = (g_bottomright)>>16;
				g_left_y = (g_topleft & 0x0000ffff);
				g_right_y = (g_bottomright & 0x0000ffff);
    	   	} else if (word == EEE_IMGPROC_MSG_START_B){ // If the incoming string == BBB
				b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				b_left_x = (b_topleft)>>16; // extracting the top 16 bits
				b_right_x = (b_bottomright)>>16;
				b_left_y = (b_topleft & 0x0000ffff);
				b_right_y = (b_bottomright & 0x0000ffff);
    	   	} else if (word == EEE_IMGPROC_MSG_START_V){ // If the incoming string == VBB
				v_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				v_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				v_left_x = (v_topleft)>>16; // extracting the top 16 bits
				v_right_x = (v_bottomright)>>16;
				v_left_y = (v_topleft & 0x0000ffff);
				v_right_y = (v_bottomright & 0x0000ffff);
    	   	} else if (word == EEE_IMGPROC_MSG_START_Y){ // If the incoming string == YBB
				y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				y_left_x = (y_topleft)>>16; // extracting the top 16 bits
				y_right_x = (y_bottomright)>>16;
				y_left_y = (y_topleft & 0x0000ffff);
				y_right_y = (y_bottomright & 0x0000ffff);
    	   	}
			// Variables from incoming verilog info have been assigned, data is ready

//    	   	printf("Iterating\n");
//			if(is_ball(b_left_x, b_right_x, b_left_y, b_right_y)){
//				printf("Blue is a ball\n");
//			}

//			 if(is_ball(y_left_x, y_right_x, y_left_y, y_right_y)){
//			 	printf("Yellow is a ball\n");
//			 }

//			if(is_in_centre_range(b_left_x, b_right_x)){
//				printf("Blue is in the centre\n");
//			}

//			 if(is_in_centre_range(y_left_x, y_right_x)){
//			 	printf("Yellow is in the centre\n");
//			 }

//    	   	if(yellowBall_ptr->seen1 == TRUE){
//    	   		printf("Yellow ball has been spotted\n");
//    	   	}else if (yellowBall_ptr->seen1 == FALSE){
//    	   		printf("Not yet seen\n");
//    	   		//yellowBall_ptr->seen = TRUE;
//    	   	}

			// Check for state, perform operations based on state
			if(state == 0){ // 1st scan - accurate distances
				//printf("Entered state 0\n");
				if((redBall_ptr->seen == FALSE) && (redBall_ptr->seen1 == FALSE) && is_ball(r_left_x, r_right_x, r_left_y, r_right_y) && is_in_centre_range(r_left_x, r_right_x)){
					printf("Red ball detected\n");
					fprintf(fp, "v%is!\n", foundbit); // tell the rover to stop
					printf("Sent s\n");

					distance = distance_calc(r_left_x, r_right_x, r_left_y, r_right_y);
					printf("Distance : %i\n", distance);
					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						printf("Distance in accurate distance range\n");
						fprintf(fp, "c/r/%d/!\n", distance);
						redBall_ptr->distance = distance;
						redBall_ptr->seen = TRUE;
						balls_detected++;

//						redBall_ptr->x_coord = int_x + distance*cos(int_theta);
//						redBall_ptr->y_coord = int_y + distance*sin(int_theta);
					}else{
						redBall_ptr->seen = FALSE;
					}
					redBall_ptr->seen1 = TRUE;

					fprintf(fp, "v%ig!\n", foundbit);
					printf("Sent g\n");

					// Need to read the next few messages to restart the while loop
					IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				} else if ((greenBall_ptr->seen == FALSE) && (greenBall_ptr->seen1 == FALSE) && is_ball(g_left_x, g_right_x, g_left_y, g_right_y) && is_in_centre_range(g_left_x, g_right_x)){
					printf("Green ball detected\n");
					fprintf(fp, "v%is!\n", foundbit);
					printf("Sent s\n");
					distance = distance_calc(g_left_x, g_right_x, g_left_y, g_right_y);
					printf("Distance : %i\n", distance);
					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						printf("Distance in accurate distance range\n");
						fprintf(fp, "c/g/%d/!\n", distance);
						greenBall_ptr->distance = distance;
						greenBall_ptr->seen = TRUE;
						balls_detected++;
					}else{
						greenBall_ptr->seen = FALSE;
					}
					greenBall_ptr->seen1 = TRUE;
					fprintf(fp, "v%ig!\n", foundbit);
				} else if ((blueBall_ptr->seen == FALSE) && (blueBall_ptr->seen1 == FALSE) && is_ball(b_left_x, b_right_x, b_left_y, b_right_y) && is_in_centre_range(b_left_x, b_right_x)){
					printf("Blue ball detected\n");
					fprintf(fp, "v%is!\n", foundbit);
					printf("Sent s\n");

					distance = distance_calc(b_left_x, b_right_x, b_left_y, b_right_y);
					printf("Distance : %i\n", distance);
					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						printf("Distance in accurate distance range\n");
						fprintf(fp, "c/b/%d/!\n", distance);
						blueBall_ptr->distance = distance;
						blueBall_ptr->seen = TRUE;
						balls_detected++;
					}else{
						blueBall_ptr->seen = FALSE;
					}
					blueBall_ptr->seen1 = TRUE;
					fprintf(fp, "v%ig!\n", foundbit);
					printf("Sent g\n");
					IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				/*} else if (is_ball(v_left_x, v_right_x, v_left_y, v_right_y) && is_in_centre_range(v_left_x, v_right_x)){
					printf("Violet ball detected\n");
					//fprintf(fp, "v%is\n", foundbit);

					distance = distance_calc(v_left_x, v_right_x, v_left_y, v_right_y);

					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						violetBall_ptr->distance = distance;
						violetBall_ptr->seen = TRUE;
						balls_detected++;

						// int_x;
						// int_y
						// int_theta
						// int x;
						// int y;
						// int theta;
					}else{
						violetBall_ptr->seen = FALSE;
					}
					//fprintf(fp, "v%ig\n", foundbit);
			}*/	}else if ((yellowBall_ptr->seen == FALSE) && (yellowBall_ptr->seen1 == FALSE) && is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && is_in_centre_range(y_left_x, y_right_x)){
					printf("Yellow ball detected\n");
					fprintf(fp, "v%is!\n", foundbit);
					printf("Sent s\n");

					distance = distance_calc(y_left_x, y_right_x, y_left_y, y_right_y);
					printf("Distance : %i\n", distance);

					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						printf("Distance in accurate distance range\n");
						fprintf(fp, "c/y/%d/!\n", distance);
						yellowBall_ptr->distance = distance;
						yellowBall_ptr->seen = TRUE;
						balls_detected++;
					}else{
						yellowBall_ptr->seen = FALSE;
					}
					yellowBall_ptr->seen1 = TRUE;

					fprintf(fp, "v%ig!\n", foundbit);
					printf("Sent g\n");

					IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				}
			}else if(state == 1){ // 2nd scan/vague distance
				//printf("Entered 2nd scan\n");
				// If we've finished the full 360
				if(s2_balls_detected == 4){//state1 == 1){ // fill with char that we expect once the full 360 for stage 2 is done
					if (s2_balls_detected == 0){ // no vague distances
						state = 2; // advance to stage 2
					}else{ // go to the closest ball
						//s2_balls_detected = 0; // reset counter for future 2nd scans

						if (closestBall->colour == 'R'){
							printf("Closest ball is red\n");
							if ((redBall_ptr->seen == FALSE) && is_ball(r_left_x, r_right_x, r_left_y, r_right_y) && is_in_centre_range(r_left_x, r_right_x)){
								fprintf(fp, "v%is!\n", foundbit);
								printf("Sent s\n");
								// Rover then starts to go towards the ball; set off function that corrects the angle
								if (go_towards(closestBall, fp)){ // returns true if we've reached the ball and measured the distance
									balls_detected++;
									state = 0;
									s2_balls_detected = 0;
									closestBall = NULL;
									printf("Reset the closest ball\n");
								}
							}
						}else if(closestBall->colour == 'G'){
							printf("Closest ball is green\n");
							if ((greenBall_ptr->seen == FALSE) && is_ball(g_left_x, g_right_x, g_left_y, g_right_y) && is_in_centre_range(g_left_x, g_right_x)){
								fprintf(fp, "v%is!\n", foundbit);
								printf("Sent s\n");
								if(go_towards(closestBall, fp)){
									balls_detected++;
									state = 0;
									s2_balls_detected = 0;
									closestBall = NULL;
									printf("Reset the closest ball\n");
								}
							}
						}else if(closestBall->colour == 'B'){
							printf("Closest ball is blue\n");
							if ((blueBall_ptr->seen == FALSE) && is_ball(b_left_x, b_right_x, b_left_y, b_right_y) && is_in_centre_range(b_left_x, b_right_x)){
								fprintf(fp, "v%is!\n", foundbit);
								printf("Sent s\n");
								if(go_towards(closestBall, fp)){
									balls_detected++;
									state = 0;
									s2_balls_detected = 0;
									closestBall = NULL;
									printf("Reset the closest ball\n");
								}
							}
						}/*else if(closestBall->colour == 'V'){
							if ((violetBall_ptr->seen == FALSE) && is_ball(v_left_x, v_right_x, v_left_y, v_right_y) && is_in_centre_range(v_left_x, v_right_x)){
								fprintf(fp, "v%is\n", foundbit);
								if (go_towards(closestBall, fp)){
									balls_detected++;
									state = 0;
								}
							}
						}*/else if(closestBall->colour == 'Y'){
							printf("Closest ball is yellow\n");
							if ((yellowBall_ptr->seen == FALSE) && is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && is_in_centre_range(y_left_x, y_right_x)){
								fprintf(fp, "v%is!\n", foundbit);
								printf("Sent s\n");
								if(go_towards(closestBall, fp)){
									balls_detected++;
									state = 0;
									s2_balls_detected = 0;
									closestBall = NULL;
									printf("Reset the closest ball\n");
								}
							}
						}
					}
				}

				// While doing the full 360
				if((redBall_ptr->seen == FALSE) && (redBall_ptr->seen2 == FALSE) && is_ball(r_left_x, r_right_x, r_left_y, r_right_y) && is_in_centre_range(r_left_x, r_right_x)){
					printf("2nd scan : Red ball detected\n");
					fprintf(fp, "v%is!\n", foundbit);
					printf("Sent s\n");

					distance = distance_calc(r_left_x, r_right_x, r_left_y, r_right_y);
					printf("Distance : %i\n", distance);
					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						printf("Red ball is in zone 2\n");
						redBall_ptr->distance = distance;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							printf("Red ball is the new closest distance\n");
							closestBall = &redBall;
							//closestBall_ptr->distance = distance;
						}
					}
					redBall_ptr->seen2 = TRUE;
					fprintf(fp, "v%ig!\n", foundbit);
					printf("Sent g\n");
				}else if((greenBall_ptr->seen == FALSE) && (greenBall_ptr->seen2 == FALSE) && is_ball(g_left_x, g_right_x, g_left_y, g_right_y) && is_in_centre_range(g_left_x, g_right_x)){
					printf("2nd scan : Green ball detected\n");
					fprintf(fp, "v%is!\n", foundbit);
					printf("Sent s\n");

					distance = distance_calc(g_left_x, g_right_x, g_left_y, g_right_y);
					printf("Distance : %i\n", distance);
					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						printf("Green ball is in zone 2\n");
						greenBall_ptr->distance = distance;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							printf("Green ball is the new closest ball\n");
							closestBall = &greenBall;
							//closestBall_ptr->distance = distance;
						}
					}

					greenBall_ptr->seen2 = TRUE;
					fprintf(fp, "v%ig!\n", foundbit);
					printf("Sent g\n");
				}else if((blueBall_ptr->seen == FALSE) && (blueBall_ptr->seen2 == FALSE) && is_ball(b_left_x, b_right_x, b_left_y, b_right_y) && is_in_centre_range(b_left_x, b_right_x)){
					printf("2nd scan : Blue ball detected\n");
					fprintf(fp, "v%is!\n", foundbit);
					printf("Sent s\n");

					distance = distance_calc(b_left_x, b_right_x, b_left_y, b_right_y);
					printf("Distance : %i\n", distance);

					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						printf("Blue ball is in zone 2\n");
						blueBall_ptr->distance = distance;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							printf("Blue ball is the new closest ball\n");
							closestBall = &blueBall;
							//closestBall_ptr->distance = distance;
						}
					}
					blueBall_ptr->seen2 = TRUE;
					fprintf(fp, "v%ig!\n", foundbit);
					printf("Sent g\n");
				}/*else if((violetBall_ptr->seen == FALSE) && (violetBall_ptr->seen2 == FALSE) && is_ball(v_left_x, v_right_x, v_left_y, v_right_y) && is_in_centre_range(v_left_x, v_right_x)){
					fprintf(fp, "v%is\n", foundbit);

					distance = distance_calc(v_left_x, v_right_x, v_left_y, v_right_y);

					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						violetBall_ptr->distance = distance;
						violetBall_ptr->seen = TRUE;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							closestBall = &violetBall;
							//closestBall_ptr->distance = distance;
						}
					}else{
						violetBall_ptr->seen = FALSE;
					}

					violetBall_ptr->seen = TRUE;

					fprintf(fp, "v%ig\n", foundbit);
				} */else if ((yellowBall_ptr->seen == FALSE) && (yellowBall_ptr->seen2 == FALSE) && is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && is_in_centre_range(y_left_x, y_right_x)){
					printf("2nd scan : Yellow ball detected\n");
					fprintf(fp, "v%is!\n", foundbit);
					printf("Sent s\n");

					distance = distance_calc(y_left_x, y_right_x, y_left_y, y_right_y);
					printf("Distance : %i\n", distance);

					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						printf("Yellow ball is in zone 2\n");
						yellowBall_ptr->distance = distance;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							printf("Yellow ball is the new closest ball\n");
							closestBall = &yellowBall;
							//closestBall_ptr->distance = distance;
						}
					}

					yellowBall_ptr->seen2 = TRUE;
					fprintf(fp, "v%ig!\n", foundbit);
					printf("Sent g\n");
				}
			}else if(state == 2){ // Scan until we see the first ball
				if ((redBall_ptr->seen == FALSE) && is_ball(r_left_x, r_right_x, r_left_y, r_right_y) && is_in_centre_range(r_left_x, r_right_x)){
					printf("s2 : first ball seen is red\n");
					fprintf(fp, "v%is!\n", foundbit);
					fprintf(fp, "v%ig!\n", foundbit);

					if (go_towards(&redBall, fp) == TRUE){
						printf("Reached red\n");
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}

				} else if ((greenBall_ptr->seen == FALSE) && is_ball(g_left_x, g_right_x, g_left_y, g_right_y) && is_in_centre_range(g_left_x, g_right_x)){
					printf("s2 : first ball seen is green\n");
					fprintf(fp, "v%is!\n", foundbit);
					fprintf(fp, "v%ig!\n", foundbit);

					if (go_towards(&greenBall, fp) == TRUE){
						printf("Reached green\n");
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}

				} else if ((blueBall_ptr->seen == FALSE) && is_ball(b_left_x, b_right_x, b_left_y, b_right_y) && is_in_centre_range(b_left_x, b_right_x)){
					fprintf(fp, "v%is!\n", foundbit);
					fprintf(fp, "v%ig!\n", foundbit);

					if (go_towards(&blueBall, fp) == TRUE){
						printf("Reached blue\n");
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}
				} /*else if ((violetBall_ptr->seen == FALSE) && is_ball(v_left_x, v_right_x, v_left_y, v_right_y) && is_in_centre_range(v_left_x, v_right_x)){
					fprintf(fp, "v%is\n", foundbit);
					usleep(2000000);
					fprintf(fp, "v%ig\n", foundbit);

					if (go_towards(&violetBall, fp) == TRUE){
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}
				} */else if ((yellowBall_ptr->seen == FALSE) && is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && is_in_centre_range(y_left_x, y_right_x)){
					fprintf(fp, "v%is!\n", foundbit);
					fprintf(fp, "v%ig!\n", foundbit);

					if (go_towards(&yellowBall, fp) == TRUE){
						printf("Reached yellow ball\n");
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}
				}
			}
    	}

       	/*
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

		*/
	   //Main loop delay
	   usleep(10000);

   	};
	fclose(fp);
  	return 0;
}
