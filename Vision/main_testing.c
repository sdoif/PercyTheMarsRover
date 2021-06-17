#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
	int distance, x_coord, y_coord;
	bool seen, seen2; // if seen overall and if seen in the 2nd scan
} Ball;

// Analyses if the object is actually the ball or not
bool is_ball(int topleft, int bottomright, int left_x, int right_x){
	int left_y = (topleft & 0x0000ffff);
	int right_y = (bottomright && 0x0000ffff);

	int height = left_y - right_y;
	int length = right_x - left_x;

	// If the length is within 10% of the height, we can consider this as a proper box
	if (length < height * 1.1 && length > height * 0.9){ // ADJUST PARAMETERS HERE
		return TRUE;
	}else{
		return FALSE;
	}
}

// Analyses if the ball is close to the centre of the camera frame
bool is_in_centre_range(int left_x, int right_x){
	int middle_x = (right_x - left_x) / 2;
	// middle x-axis pixel = 320
	if (middle_x < 400 && middle_x > 240){
		return TRUE;
	}else{
		return FALSE;
	}
}

// Check if the ball is within accurate distance measurements
bool distance_check_z1(int distance){
	if (distance >= 20 && distance <= 80){
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
int distance_calc(int left_x, int right_x){

		// D = (W*F)/P
	// W = diameter of the ball
	int W = 3.95;
	// F = Focal length
	int F = 700;
	// P = apparent width in pixels
	int P = right_x - left_x;
	// D = Distance from camera
	int D = (W*F)/P;
	int posD;

	if(D >= 0){
		posD = D;
	}else{
		posD = 0;
	}
	return posD;
	/* Moving average filter has been moved to int main
	//		Moving Average Filter
	// int *array[] stores the previous 10 results
	// index - handled in int main - global index that iterates from main
	//		which allows us to manage array index placement of all arrays
	//		using a single variable and without having to move values
	array[index] = posD;

	if (array[0] == 0){ // if we have not obtained enough distance values, dont average
		return posD;
	}

	// Finding the average
	int sum = 0;
	for (int i = 0; i < 10; i++){
		sum += posD;
	}

	return (sum / 10);
	*/
}

int angle_calc(int left_x, int right_x){
	int middle_x = right_x - left_x;

	// resolution width = 640
	// field of view (measured = 50 degrees)
	float ang_per_pix = 50/640;
	return ang_per_pix * (middle_x - 320); // difference in pixels of the object from the centre
}

// Have a while loop that just reads the boundary box coordinates for the ball we are going to
//		and updates just the distance of that ball. Angle calculations are made here too in this while loop.
//		End of the while loop is caused by the distance to the ball being within a specific range. Function returns true
//		After we've gotten to the ball (and measured/set distance for the final time), we update state to 0 to restart the loop

// Function that corrects the angle as we drive towards the ball, returns TRUE once we've
//		reached the ball
bool go_towards(Ball *ball, FILE* fp){
	int verilog_word;
	int distance, avg_distance;
	int sum = 0;

	if(ball->colour == 'R'){
		int r_d[5];
		while ((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8) & 0xff) {

			// Update the boundary box co-ordinates
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_R){ // If the incoming string == RBB
				// Print on a newline
				printf("\n");
				int r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int r_left_x = (r_topleft)>>16; // extracting the top 16 bits
				int r_right_x = (r_bottomright)>>16;

				int left_y = (r_topleft & 0x0000ffff);
				int right_y = (r_bottomright && 0x0000ffff);

				// Measure angle and correct if needed to
				int angle = angle_calc(r_left_x, r_right_x);
				if(angle > 10){
					fprintf(fp, "S");
					fprintf(fp, "L");
				}else if(angle < -10){
					fprintf(fp, "S");
					fprintf(fp, "R");
				}else{
					fprintf(fp, "F");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(r_topleft, r_bottomright, r_left_x, r_right_x) && is_in_centre_range(r_left_x, r_right_x)){
					distance = distance_calc(r_left_x, r_right_x);
				}

				// Initialise the array with the first distance value if empty
				if (r_d[4] == 0){
					for (int i = 0; i < 5; i++){
						r_d[i] = distance;
					}
				}else{ // Else shift the values and update the last term if the distance and calculate the average
					for(int i = 0; i < 4; i++){
						r_d[i] = r_d[i+1];
						sum+= r_d[i];
					}
					r_d[4] = distance;
					avg_distance = sum/5;
				}

				if (avg_distance < 50 && avg_distance > 30){
					// Update members
					ball->distance = avg_distance;
					ball->seen = TRUE;

					return TRUE;
				}
    	   	}
		}
	}else if(ball->colour == 'G'){
		int g_d[5];
		while ((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8) & 0xff) {

			// Update the boundary box co-ordinates
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_G){ // If the incoming string == RBB
				// Print on a newline
				printf("\n");
				int g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int g_left_x = (g_topleft)>>16; // extracting the top 16 bits
				int g_right_x = (g_bottomright)>>16;

				int left_y = (g_topleft & 0x0000ffff);
				int right_y = (g_bottomright && 0x0000ffff);

				// Measure angle and correct if needed to
				int angle = angle_calc(g_left_x, g_right_x);
				if(angle > 10){
					fprintf(fp, "S");
					fprintf(fp, "L");
				}else if(angle < -10){
					fprintf(fp, "S");
					fprintf(fp, "R");
				}else{
					fprintf(fp, "F");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(g_topleft, g_bottomright, g_left_x, g_right_x) && is_in_centre_range(g_left_x, g_right_x)){
					distance = distance_calc(g_left_x, g_right_x);
				}

				// Initialise the array with the first distance value if empty
				if (g_d[4] == 0){
					for (int i = 0; i < 5; i++){
						g_d[i] = distance;
					}
				}else{ // Else shift the values and update the last term if the distance and calculate the average
					for(int i = 0; i < 4; i++){
						g_d[i] = g_d[i+1];
						sum+= g_d[i];
					}
					g_d[4] = distance;
					avg_distance = sum/5;
				}
				// Send distance measurement here

				if (avg_distance < 50 && avg_distance > 30){
					// Update members
					ball->distance = avg_distance;
					ball->seen = TRUE;

					return TRUE; // Exit function here
				}
    	   	}
		}
	}else if(ball->colour == 'B'){
		int b_d[5];
		while ((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8) & 0xff) {

			// Update the boundary box co-ordinates
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_B){ // If the incoming string == RBB
				// Print on a newline
				printf("\n");
				int b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int b_left_x = (b_topleft)>>16; // extracting the top 16 bits
				int b_right_x = (b_bottomright)>>16;

				int left_y = (b_topleft & 0x0000ffff);
				int right_y = (b_bottomright && 0x0000ffff);

				// Measure angle and correct if needed to
				int angle = angle_calc(b_left_x, b_right_x);
				if(angle > 10){
					fprintf(fp, "S");
					fprintf(fp, "L");
				}else if(angle < -10){
					fprintf(fp, "S");
					fprintf(fp, "R");
				}else{
					fprintf(fp, "F");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(b_topleft, b_bottomright, b_left_x, b_right_x) && is_in_centre_range(b_left_x, b_right_x)){
					distance = distance_calc(b_left_x, b_right_x);
				}

				// Initialise the array with the first distance value if empty
				if (b_d[4] == 0){
					for (int i = 0; i < 5; i++){
						b_d[i] = distance;
					}
				}else{ // Else shift the values and update the last term if the distance and calculate the average
					for(int i = 0; i < 4; i++){
						b_d[i] = b_d[i+1];
						sum+= b_d[i];
					}
					b_d[4] = distance;
					avg_distance = sum/5;
				}

				if (avg_distance < 50 && avg_distance > 30){
					// Update members
					ball->distance = avg_distance;
					ball->seen = TRUE;

					return TRUE;
				}
    	   	}
		}
	}else if(ball->colour == 'V'){
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

				int left_y = (v_topleft & 0x0000ffff);
				int right_y = (v_bottomright && 0x0000ffff);

				// Measure angle and correct if needed to
				int angle = angle_calc(v_left_x, v_right_x);
				if(angle > 10){
					fprintf(fp, "S");
					fprintf(fp, "L");
				}else if(angle < -10){
					fprintf(fp, "S");
					fprintf(fp, "R");
				}else{
					fprintf(fp, "F");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(v_topleft, v_bottomright, v_left_x, v_right_x) && is_in_centre_range(v_left_x, v_right_x)){
					distance = distance_calc(v_left_x, v_right_x);
				}

				// Initialise the array with the first distance value if empty
				if (v_d[4] == 0){
					for (int i = 0; i < 5; i++){
						v_d[i] = distance;
					}
				}else{ // Else shift the values and update the last term if the distance and calculate the average
					for(int i = 0; i < 4; i++){
						v_d[i] = v_d[i+1];
						sum+= v_d[i];
					}
					v_d[4] = distance;
					avg_distance = sum/5;
				}

				if (avg_distance < 50 && avg_distance > 30){
					// Update members
					ball->distance = avg_distance;
					ball->seen = TRUE;

					return TRUE;
				}
    	   	}
		}
	}else if(ball->colour == 'Y'){
		int y_d[5];
		while ((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8) & 0xff) {

			// Update the boundary box co-ordinates
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_Y){ // If the incoming string == RBB
				// Print on a newline
				printf("\n");
				int y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int y_left_x = (y_topleft)>>16; // extracting the top 16 bits
				int y_right_x = (y_bottomright)>>16;

				int left_y = (y_topleft & 0x0000ffff);
				int right_y = (y_bottomright && 0x0000ffff);

				// Measure angle and correct if needed to
				int angle = angle_calc(y_left_x, y_right_x);
				if(angle > 10){
					fprintf(fp, "S");
					fprintf(fp, "L");
				}else if(angle < -10){
					fprintf(fp, "S");
					fprintf(fp, "R");
				}else{
					fprintf(fp, "F");
				}

				// If we can make a valid decision measurement, make it
				if(is_ball(y_topleft, y_bottomright, y_left_x, y_right_x) && is_in_centre_range(y_left_x, y_right_x)){
					distance = distance_calc(y_left_x, y_right_x);
				}

				// Initialise the array with the first distance value if empty
				if (y_d[4] == 0){
					for (int i = 0; i < 5; i++){
						y_d[i] = distance;
					}
				}else{ // Else shift the values and update the last term if the distance and calculate the average
					for(int i = 0; i < 4; i++){
						y_d[i] = y_d[i+1];
						sum+= y_d[i];
					}
					y_d[4] = distance;
					avg_distance = sum/5;
				}

				if (avg_distance < 50 && avg_distance > 30){
					// Update members
					ball->distance = avg_distance;
					ball->seen = TRUE;

					return TRUE;
				}
    	   	}
		}
	}
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

    FILE *fp;
    int j = 0;
    char incomingString[32];

    char state[2];
    char theta[6];
    char radius[6];
    int incomingChar;

    // v00+0057.4+0132.0


	// Working UART Connection loop
	//while(1){
		j++;
		printf("Try No. %i\n", j);
		fp = fopen("/dev/uart", "r+");
		if(fp){
			//printf("Opened connection to UART\n");
			//while(1){
				fprintf(fp, "test\n");
				printf("Sent test\n");

				//char incomingChar = fgetc(fp);
				while(1){
				    char state[2];
				    char incomingChar;

					//fgets(incomingString, 32, fp);
					printf("\n");
					//printf("Incoming string : %s\n", incomingString);
					//usleep(1000000);
					//printf("0th index : %c, 17th index : %c\n", incomingString[0], incomingString[17]);

					incomingChar = fgetc(fp);

			        if (incomingChar == 'v'){ // detected beginning of my info
			        	printf("detected a v\n");
			        	int state1 = fgetc(fp);
			        	printf("state1 : %c\n", state1);
//			        	usleep(1000000);
			        	int state2 = fgetc(fp);
			        	printf("state2 : %c\n", state2);
//			        	usleep(1000000);
			        	incomingChar = fgetc(fp); // to skip the first +
			        	printf("should be a +: %c\n", incomingChar);
			        	incomingChar = fgetc(fp);
					    char theta[6] = "0";
			        	while(incomingChar != '+'){ // read chars and append to string until we reach the +
//			        		usleep(1000000);
			        		printf("incoming theta digit: %c\n", incomingChar);
			        		if((incomingChar < 58 && incomingChar > 47) || incomingChar == 46){
			        			strncat(theta, &incomingChar, 1);
			        		}
			        		//theta[i] = incomingChar;
			        		//strcat(theta, incomingChar);
			        		incomingChar = fgetc(fp);
			        	}
			        	printf("theta: %s\n", theta);
			        	float int_theta = 0;
			        	int_theta = atof(theta);
			        	printf("int theta : %f\n", int_theta);

					    char radius[6] = "0";
			        	incomingChar = fgetc(fp); // read the first digit
			        	while(incomingChar != '-'){ // read chars and append to string until we reach the end of the line
			        		// usleep(1000000);
			        		printf("incoming radius digit: %c\n", incomingChar);
			        		if((incomingChar < 58 && incomingChar > 47) || incomingChar == 46){
			        			strncat(radius, &incomingChar, 1);
			        		}
			        		incomingChar = fgetc(fp);

			        	}
			        	printf("radius: %s\n", radius);
			        	float int_radius = 0;
			        	int_radius = atof(radius);
			        	printf("int radius : %f\n", int_radius);
			        	usleep(2000000);
			        }

			        /*
					for (int i = 0; i < 18; i++){
						incomingChar = fgetc(fp);
						printf("incoming car : %c\n", incomingChar);
					}

					//strncat(state, &incomingString[0], 2);
					//printf("state chars : %s\n", state);

					if (incomingString[0] == 'v' && incomingString[16] == '.'){
						//for (int i = 1; i < 3; i++){ // skip the v, start at i = 1
							//strncat(state, &incomingString[i], 2);
						//}
						//printf("state chars : %s\n", state);

	//					thetaSign = incomingString[3];
	//					printf("thetaSign : %c", thetaSign);

						for (int i = 3; i < 10; i++){
							strncat(theta, &incomingString[i], 1);
						}
						printf("theta : %s\n", theta);

						for (int i = 11; i < 17; i++){
							strncat(radius, &incomingString[i], 1);
						}
						printf("radius : %s\n", radius);
					}*/
				}
			fclose(fp);
		}else{
			printf("Unable to connect to UART\n");
		}
		fclose(fp);
	//	usleep(1000000);
	//}


	/* Receiving test
	while(1){
		int test = IORD(UART_BASE,0);
		printf("Received %i\n", test);
	}
	*/

    //FILE *fp;

    // v00+0057.4+0132.0
	/*
    while(1){

		// Using fopen
		while(1){
			fp = fopen("dev/uart", "r+");
				if(fp){
					//printf("Opened connection to UART\n");
					while(1){
						fprintf(fp, "test\n");
						printf("Sent test\n");
					}
					fclose(fp);
				}else{
					printf("Unable to connect to UART\n");
				}
			}
		}
	*/

		/* Using IORD
		// should be word from UART
		usleep(1000000);
		int control_word = IORD(UART_BASE, 0);
        printf("received: %c\n", control_word);
        */
        /*
        if (control_word == 118){ // detected beginning of my info
        	printf("detected a v\n");
        	int state1 = IORD(UART_BASE, 0);
        	usleep(1000000);
        	int state2 = IORD(UART_BASE, 0);
        	usleep(1000000);
        	thetaSign = IORD(UART_BASE, 0); // capture the sign
        	while(IORD(UART_BASE, 0) != 43){ // read chars and append to string until we reach the +
        		usleep(1000000);
        		incomingChar = IORD(UART_BASE, 0);
        		strncat(theta, &incomingChar, 1);
        	}
        	while(IORD(UART_BASE, 0) != 13 && IORD(UART_BASE, 0) != 10){ // read chars and append to string until we reach the +
        		usleep(1000000);
        		incomingChar = IORD(UART_BASE, 0);
        		strncat(radius, &incomingChar, 1);
        	}
        	printf("theta: %s\n", theta);
        	printf("radius: %s\n", radius);
        }
        */
      //}

	while(1){
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