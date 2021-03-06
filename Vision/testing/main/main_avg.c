#include <stdio.h>
#include <string.h>
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
	int middle_x = (right_x + left_x) / 2;
	printf("Middle axis pixel is %i\n", middle_x);
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

int avg_distance(int distance, int *array){
    // Fill array if we have no previous distances so far
    if (array[0] == 0){
        printf("Avg distance array is empty\n");
        for (int i = 0; i < 5; i++){
            array[i] = distance;
        }
        return distance;
    }else{ // If there are already terms in the array
        int sum;
        for (int i = 0; i < 4; i++){
            array[i] = array[i+1];
            sum+= array[i];
        }
        array[4] = distance;
        sum += distance;
        return abs(sum / 5);
    }
}

//int avg_leftx()

// Function that returns the distance of the ball from the camera based on its boundary
//		box coordinates
alt_u8 distance_calc(int left_x, int right_x){

		// D = (W*F)/P
	// W = diameter of the ball
	double W = 3.95;
	// F = Focal length
	double F = 700;
	// P = apparent width in pixels
	double P = right_x - left_x;
	// D = Distance from camera
	alt_u8 D = (W*F)/P;

	return abs(D);
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

	/* Receiving test
	while(1){
		int test = IORD(UART_BASE,0);
		printf("Received %i\n", test);
	}
	*/

	// while loop that means computation only begins once we've read
	//		the character telling us to begin
	while(1){
		// should be word from UART
		int word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
		if (word == 0){ // TODO - fill with char that tells us we've started
			break;
		}
	}

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
	Ball redBall, greenBall, blueBall, violetBall, yellowBall;
	redBall.colour = 'R';
	redBall.seen = FALSE;

	greenBall.colour = 'G';
	greenBall.seen = FALSE;

	blueBall.colour = 'B';
	blueBall.seen = FALSE;

	violetBall.colour = 'V';
	violetBall.seen = FALSE;

	yellowBall.colour = 'Y';
	yellowBall.seen = FALSE;

	int balls_detected = 0;

	// Measurement related (Base)
	alt_u32 r_topleft, g_topleft, b_topleft, v_topleft, y_topleft;
	alt_u32 r_bottomright, g_bottomright, b_bottomright, v_bottomright, y_bottomright;

	// Measurement related (Derived)
	alt_u32 r_left_x, g_left_x, b_left_x, v_left_x, y_left_x;
	alt_u32 r_right_x, g_right_x, b_right_x, v_right_x, y_right_x;
	alt_u32 r_left_y, g_left_y, b_left_y, v_left_y, y_left_y;
	alt_u32 r_right_y, g_right_y, b_right_y, v_right_y, y_right_y;

	alt_u32 r_left_x_final, g_left_x_final, b_left_x_final, v_left_x_final, y_left_x_final;
	alt_u32 r_right_x_final, g_right_x_final, b_right_x_final, v_right_x_final, y_right_x_final;
	alt_u32 r_left_y_final, g_left_y_final, b_left_y_final, v_left_y_final, y_left_y_final;
	alt_u32 r_right_y_final, g_right_y_final, b_right_y_final, v_right_y_final, y_right_y_final;

	// Single variable to handle all of the distances
	alt_u8 distance;

	// 2nd scan specific variables
	int s2_balls_detected = 0;
	Ball nothing;
	Ball *closestBall;
	closestBall = &nothing;
	closestBall->distance = 500;

	// 3rd scan specific variables
	Ball *firstBall;
	firstBall = &nothing;

	// Moving average filters - arrays, their points and the index
	double r_d[5], g_d[5], b_d[5], v_d[5], y_d[5];
	double *r_d_ptr = r_d;
	double *g_d_ptr = g_d;
	double *b_d_ptr = b_d;
	double *v_d_ptr = v_d;
	double *y_d_ptr = y_d;
	int index; // to handle all moving average filters in sync

	// Other
	int state = 0; // or stage

	alt_u32 r_lxsum, g_lxsum, b_lxsum, v_lxsum, y_lxsum;
	alt_u32 r_rxsum, g_rxsum, b_rxsum, v_rxsum, y_rxsum;
	alt_u32 r_lysum, g_lysum, b_lysum, v_lysum, y_lysum;
	alt_u32 r_rysum, g_rysum, b_rysum, v_rysum, y_rysum;

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

       	//Read messages from the image processor and print them on the terminal
       	while ((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8) & 0xff) { 	//Find out if there are words to read
			//int control_word = IORD(UART_BASE, 0);

			/*
			if (balls_detected == 5 && (control_word == 1) && (control_word == 2)){ // TODO - Replace 1 and 2 with expected end of scan identifiers
				// send signal which indicates that all 5 balls have been detected
				fprintf(fp, "");
			}
			*/

			// Analyse incoming BB information and verilog and make the proper variable assignments.
			// 		Make 5 data measurements and take averages from them

       		for (int i = 0; i < 25; i++){ // TODO - Have all data here be remembered in this scope and the final value be the average of the 5 iterations
				//Get next word from message buffer (Verilog)
				int word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				if (word == EEE_IMGPROC_MSG_START_R){ // If the incoming string == RBB
					// Print on a newline
					printf("\n");
					r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
					r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

					r_left_x = (r_topleft)>>16; // extracting the top 16 bits
					r_right_x = (r_bottomright)>>16;
					r_left_y = (r_topleft & 0x0000ffff);
					r_right_y = (r_bottomright & 0x0000ffff);

					r_lxsum += r_left_x;
					r_rxsum += r_right_x;
					r_lysum += r_left_y;
					r_rysum += r_right_y;

					// distance = distance_calc(r_left_x, r_right_x, r_d_ptr, index);
					//("Red distance : %i cm", distance);
				} else if (word == EEE_IMGPROC_MSG_START_G){ // If the incoming string == GBB
					printf("\n");
					g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

					g_left_x = (g_topleft)>>16; // extracting the top 16 bits
					g_right_x = (g_bottomright)>>16;
					g_left_y = (g_topleft & 0x0000ffff);
					g_right_y = (g_bottomright & 0x0000ffff);

					g_lxsum += g_left_x;
					g_rxsum += g_right_x;
					g_lysum += g_left_y;
					g_rysum += g_right_y;

					// distance = distance_calc(g_left_x, g_right_x, g_d_ptr, index);
					//printf("Green distance : %i cm", distance);
				} else if (word == EEE_IMGPROC_MSG_START_B){ // If the incoming string == BBB
					printf("\n");
					b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

					b_left_x = (b_topleft)>>16; // extracting the top 16 bits
					b_right_x = (b_bottomright)>>16;
					b_left_y = (b_topleft & 0x0000ffff);
					b_right_y = (b_bottomright & 0x0000ffff);

					b_lxsum += b_left_x;
					b_rxsum += b_right_x;
					b_lysum += b_left_y;
					b_rysum += b_right_y;

					// distance = distance_calc(b_left_x, b_right_x, b_d_ptr, index);
					// printf("Blue distance : %i cm\n", distance);
				} else if (word == EEE_IMGPROC_MSG_START_V){ // If the incoming string == VBB
					printf("\n");
					v_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					v_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

					v_left_x = (v_topleft)>>16; // extracting the top 16 bits
					v_right_x = (v_bottomright)>>16;
					v_left_y = (v_topleft & 0x0000ffff);
					v_right_y = (v_bottomright & 0x0000ffff);

					v_lxsum += v_left_x;
					v_rxsum += v_right_x;
					v_lysum += v_left_y;
					v_rysum += v_right_y;

					// distance = distance_calc(v_left_x, v_right_x, v_d_ptr, index);
					//printf("Violet distance : %i cm", distance);
				} else if (word == EEE_IMGPROC_MSG_START_Y){ // If the incoming string == YBB
					printf("\n");
					y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

					y_left_x = (y_topleft)>>16; // extracting the top 16 bits
					printf("left x noavg : %i\n", y_left_x);
					y_right_x = (y_bottomright)>>16;
					printf("right x noavg : %i\n", y_right_x);
					y_left_y = (y_topleft & 0x0000ffff);
					y_right_y = (y_bottomright & 0x0000ffff);

					y_lxsum += y_left_x;
					printf("lxsum : %i\n", y_lxsum);
					y_rxsum += y_right_x;
					y_lysum += y_left_y;
					y_rysum += y_right_y;

					// distance = distance_calc(y_left_x, y_right_x, y_d_ptr, index);
					//printf("Yellow distance : %i cm", distance);
				}
       		}

       		r_left_x = r_lxsum / 5;
       		r_right_x = r_rxsum / 5;
       		r_left_y = r_lysum / 5;
       		r_right_y = r_rysum / 5;

       		g_left_x = g_lxsum / 5;
       		g_right_x = g_rxsum / 5;
       		g_left_y = g_lysum / 5;
       		g_right_y = g_rysum / 5;

       		b_left_x = b_lxsum / 5;
       		b_right_x = b_rxsum / 5;
       		b_left_y = b_lysum / 5;
       		b_right_y = b_rysum / 5;

       		v_left_x = v_lxsum / 5;
       		v_right_x = v_rxsum / 5;
       		v_left_y = v_lysum / 5;
       		v_right_y = v_rysum / 5;

       		y_left_x_final = y_lxsum / 5;
       		y_right_x_final = y_rxsum / 5;
       		y_left_y_final = y_lysum / 5;
       		y_right_y_final = y_rysum / 5;

       		r_lxsum = 0;
       		r_rxsum = 0;
       		r_lysum = 0;
       		r_rysum = 0;

       		g_lxsum = 0;
       		g_rxsum = 0;
       		g_lysum = 0;
       		g_rysum = 0;

       		b_lxsum = 0;
       		b_rxsum = 0;
       		b_lysum = 0;
       		b_rysum = 0;

       		v_lxsum = 0;
       		v_rxsum = 0;
       		v_lysum = 0;
       		v_rysum = 0;

       		y_lxsum = 0;
       		y_rxsum = 0;
       		y_lysum = 0;
       		y_rysum = 0;
			// Variables from incoming verilog info have been assigned, data is ready

            // Testing functions
    	   	/*
            if(is_ball(y_topleft, y_bottomright, y_left_x, r_right_x)){
                printf("Yellow ball is being considered as a ball\n");
            }

            if (is_in_centre_range(y_left_x, y_right_x)){
                printf("Yellow ball is in the centre range\n");
            }
            int angle = angle_calc(y_left_x, y_right_x);
            printf("Angle from centre is: %i\n", angle);
            */

            // Find average distance of the previous 5 results
       		printf("Left x: %d, Right x : %d\n", y_left_x_final, y_right_x_final);
       		alt_u8 distance = distance_calc(y_left_x, y_right_x);
			//distance = avg_distance(distance, y_d_ptr);
            printf("Distance : %i\n", distance);
/*
			// Check incoming chars for state changes
			if (control_word == 0){ // TODO - Update and complete once we've finalised the character selections
				state = 1;
			}//else if


			// Check for state, perform operations based on state
			if(state == 0){ // 1st scan - accurate distances
				if(is_ball(r_topleft, r_bottomright, r_left_x, r_right_x) && is_in_centre_range(r_left_x, r_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(r_left_x, r_right_x);
						r_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						redBall.distance = distance;
						fprintf(fp, "D%i\n", distance);
						redBall.seen = TRUE;
						balls_detected++;
					}else{
						redBall.seen = FALSE;
					}
					fprintf(fp, "G");
				} else if (is_ball(g_topleft, g_bottomright, g_left_x, g_right_x) && is_in_centre_range(g_left_x, g_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(g_left_x, g_right_x);
						g_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						greenBall.distance = distance;
						fprintf(fp, "D%i\n", distance);
						greenBall.seen = TRUE;
						balls_detected++;
					}else{
						greenBall.seen = FALSE;
					}
					fprintf(fp, "G");
				} else if (is_ball(b_topleft, b_bottomright, b_left_x, b_right_x) && is_in_centre_range(b_left_x, b_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(b_left_x, b_right_x);
						b_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						blueBall.distance = distance;
						fprintf(fp, "D%i\n", distance);
						blueBall.seen = TRUE;
						balls_detected++;
					}else{
						blueBall.seen = FALSE;
					}
					fprintf(fp, "G");
				} else if (is_ball(v_topleft, v_bottomright, v_left_x, v_right_x) && is_in_centre_range(v_left_x, v_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(v_left_x, v_right_x);
						v_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						violetBall.distance = distance;
						fprintf(fp, "D%i\n", distance);
						violetBall.seen = TRUE;
						balls_detected++;
					}else{
						violetBall.seen = FALSE;
					}
					fprintf(fp, "G");
				} else if (is_ball(y_topleft, y_bottomright, y_left_x, y_right_x) && is_in_centre_range(y_left_x, y_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(y_left_x, y_right_x);
						y_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z1(distance) == 1){
						yellowBall.distance = distance;
						fprintf(fp, "D%i\n", distance);
						yellowBall.seen = TRUE;
						balls_detected++;
					}else{
						yellowBall.seen = FALSE;
					}
					fprintf(fp, "G");
				}
			}else if(state == 1){ // 2nd scan/vague distance

				// If we've finished the full 360
				if(control_word == 00){ // fill with char that we expect once the full 360 for stage 2 is done
					if (s2_balls_detected == 0){ // no vague distances
						state = 2; // advance to stage 2
					}else{ // go to the closest ball
						s2_balls_detected = 0; // reset counter for future 2nd scans
						fprintf(fp, "i"); // trigger 2nd function causing us to go towards the first ball we stop at

						if (closestBall->colour == 'R'){
							if (is_ball(r_topleft, r_bottomright, r_left_x, r_right_x) && is_in_centre_range(r_left_x, r_right_x)){
								fprintf(fp, "S");
								// Rover then starts to go towards the ball; set off function that corrects the angle
								if (go_towards(closestBall, fp)){ // returns true if we've reached the ball and measured the distance
									balls_detected++;
								}
							}
						}else if(closestBall->colour == 'G'){
							if (is_ball(g_topleft, g_bottomright, g_left_x, g_right_x) && is_in_centre_range(g_left_x, g_right_x)){
								fprintf(fp, "S");
								if(go_towards(closestBall, fp)){
									balls_detected++;
								}
							}
						}else if(closestBall->colour == 'B'){
							if (is_ball(b_topleft, b_bottomright, b_left_x, b_right_x) && is_in_centre_range(b_left_x, b_right_x)){
								fprintf(fp, "S");
								if(go_towards(closestBall, fp)){
									balls_detected++;
								}
							}
						}else if(closestBall->colour == 'V'){
							if (is_ball(v_topleft, v_bottomright, v_left_x, v_right_x) && is_in_centre_range(v_left_x, v_right_x)){
								fprintf(fp, "S");
								if (go_towards(closestBall, fp)){
									balls_detected++;
								}
							}
						}else if(closestBall->colour == 'Y'){
							if (is_ball(y_topleft, y_bottomright, y_left_x, y_right_x) && is_in_centre_range(y_left_x, y_right_x)){
								fprintf(fp, "S");
								if(go_towards(closestBall, fp)){
									balls_detected++;
								}
							}
						}
					}
				}

				// While doing the full 360
				if(is_ball(r_topleft, r_bottomright, r_left_x, r_right_x) && is_in_centre_range(r_left_x, r_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(r_left_x, r_right_x);
						r_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						redBall.distance = distance;
						redBall.seen2 = TRUE;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							closestBall = &redBall;
							//closestBall.distance = distance;
						}
					}else{
						redBall.seen2 = FALSE;
					}
					fprintf(fp, "G");
				}else if(is_ball(g_topleft, g_bottomright, g_left_x, g_right_x) && is_in_centre_range(g_left_x, g_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(g_left_x, g_right_x);
						g_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						greenBall.distance = distance;
						greenBall.seen2 = TRUE;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							closestBall = &greenBall;
							//closestBall.distance = distance;
						}
					}else{
						greenBall.seen2 = FALSE;
					}
					fprintf(fp, "G");
				}else if(is_ball(b_topleft, b_bottomright, b_left_x, b_right_x) && is_in_centre_range(b_left_x, b_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(b_left_x, b_right_x);
						b_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						blueBall.distance = distance;
						blueBall.seen2 = TRUE;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							closestBall = &blueBall;
							//closestBall.distance = distance;
						}
					}else{
						blueBall.seen2 = FALSE;
					}
					fprintf(fp, "G");
				}else if(is_ball(v_topleft, v_bottomright, v_left_x, v_right_x) && is_in_centre_range(v_left_x, v_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(v_left_x, v_right_x);
						v_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						violetBall.distance = distance;
						violetBall.seen2 = TRUE;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							closestBall = &violetBall;
							//closestBall.distance = distance;
						}
					}else{
						violetBall.seen2 = FALSE;
					}

					fprintf(fp, "G");
				} else if (is_ball(y_topleft, y_bottomright, y_left_x, y_right_x) && is_in_centre_range(y_left_x, y_right_x)){
					fprintf(fp, "S"); // tell the rover to stop

					// Find average distance of the previous 5 results
					int sum = 0;
					int temp_d;
					for (int i = 0; i < 5; i++){
						temp_d = distance_calc(y_left_x, y_right_x);
						y_d[i] = temp_d;
						sum+= temp_d;
					}
					distance = temp_d / 5;

					// Check what zone that distance corresponds to
					if (distance_check_z2(distance) == 1){
						yellowBall.distance = distance;
						yellowBall.seen2 = TRUE;
						s2_balls_detected++;

						if (distance < closestBall->distance){
							closestBall = &yellowBall;
							//closestBall.distance = distance;
						}
					}else{
						yellowBall.seen2 = FALSE;
					}
					fprintf(fp, "G");
				}
			}else if(state == 2){ // Scan until we see the first ball
				fprintf(fp, "i");
				if (is_ball(r_topleft, r_bottomright, r_left_x, r_right_x) && is_in_centre_range(r_left_x, r_right_x)){
					fprintf(fp, "S"); // tell the rover to stop
					fprintf(fp, "G");

					if (go_towards('R', fp) == TRUE){
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}

				} else if (is_ball(g_topleft, g_bottomright, g_left_x, g_right_x) && is_in_centre_range(g_left_x, g_right_x)){
					fprintf(fp, "S"); // tell the rover to stop
					fprintf(fp, "G");

					if (go_towards('G', fp) == TRUE){
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}

				} else if (is_ball(b_topleft, b_bottomright, b_left_x, b_right_x) && is_in_centre_range(b_left_x, b_right_x)){
					fprintf(fp, "S"); // tell the rover to stop
					fprintf(fp, "G");

					if (go_towards('B', fp) == TRUE){
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}
				} else if (is_ball(v_topleft, v_bottomright, v_left_x, v_right_x) && is_in_centre_range(v_left_x, v_right_x)){
					fprintf(fp, "S"); // tell the rover to stop
					fprintf(fp, "G");

					if (go_towards('V', fp) == TRUE){
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}
				} else if (is_ball(y_topleft, y_bottomright, y_left_x, y_right_x) && is_in_centre_range(y_left_x, y_right_x)){
					fprintf(fp, "S"); // tell the rover to stop
					fprintf(fp, "G");

					if (go_towards('Y', fp) == TRUE){
						balls_detected++;
						state = 0; // to repeat the 1st scan and loop back
					}
			}
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