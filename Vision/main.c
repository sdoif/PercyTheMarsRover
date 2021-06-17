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

//EEE_IMGPROC defines - MSG ID tags
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
	char colour; // single character indicating the colour of the ball
	int distance; // member used to compare distance between different balls
	bool seen, seen1, seen2, seen3; // members describing if the ball has been spotted
} Ball;

// Measures relationship between height and length of the boundary box 
//		and determines if this is around a ball
bool is_ball(int left_x, int right_x, int left_y, int right_y){
	int height = right_y - left_y;
	//printf("Height : %i ", height);
	int length = right_x - left_x;
	//printf("Length : %i\n", length);

	// If the length is within 30% of the height, we can consider this as a proper box.
	// Other constraints are for stopping the analysis of boundary boxes that are too large or too small
	if (length < height * 1.3 && length > height * 0.7 && height > 5 && height < 175 && length > 5 && length < 175 ){
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

// Scan 1 - Accurate distance scan parameters
bool distance_check_z1(int distance){
	if (distance >= 25 && distance < 80){
		return TRUE;
	}else{
		return FALSE;
	}
}

// Scan 2 - Vague distance scan parameters
bool distance_check_z2(int distance){
	if (distance >= 80 && distance <= 180){
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

	// D = Distance from camera
	float D = (W*F)/P;
	return abs(D);
}

// Function that corrects the angle as we drive towards the ball, returns TRUE once we've
//		reached the ball input by the argument ball
bool go_towards(Ball *ball, FILE* fp){
	int verilog_word; // information received from verilog
	int distance; // distance of the ball we are going towards

	// variables used to stop repeatedly re-entering the same while loop
	int s1, s2, s3;
	s1 = 0;
	s2 = 0;
	s3 = 0;

	if(ball->colour == 'R'){ // going towards red
		printf("Going towards red\n");
		while (1) {
			fflush(fp); // get rid of past commands that may have accumulated

			// Update the boundary box co-ordinates
			verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_R){ // If the incoming string == RBB
				int r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				// Processing the coordinates
				int r_left_x = (r_topleft)>>16; 
				int r_right_x = (r_bottomright)>>16;
				int r_left_y = (r_topleft & 0x0000ffff);
				int r_right_y = (r_bottomright & 0x0000ffff);

				if(is_ball(r_left_x, r_right_x, r_left_y, r_right_y)){
					// printf("is ball\n");
					// Measure angle and correct if needed to
					int middle_pix = (r_right_x + r_left_x) / 2;
					if (middle_pix < 240 && s1 == 0){ // turn left
						fprintf(fp, "v0l!\n");
						printf("r : turn left\n");
						s1 = 1;
						s2 = 0;
						s3 = 0;
					}else if(middle_pix > 400 && s2 == 0){ // turn right
						fprintf(fp, "v0r!\n");
						printf("r : turn right\n");
						s1 = 0;
						s2 = 1;
						s3 = 0;
					}else if (middle_pix <= 400 && middle_pix >= 240 && s3 == 0){ // stay on course
						fprintf(fp, "v0g!\n");
						printf("r : go forward\n");
						s1 = 0;
						s2 = 0;
						s3 = 1;
					}

					// If we can make a valid decision measurement, make it
					if(is_ball(r_left_x, r_right_x, r_left_y, r_right_y) && middle_pix <= 440 && middle_pix >= 200){
						//printf("Is ball and is centred\n");
						distance = distance_calc(r_left_x, r_right_x, r_left_y, r_right_y);
						printf("Distance : %i\n", distance);
						if (distance < 80 && distance > 25){
							printf("Reached red\n");

							// tell Drive to stop
							fprintf(fp, "v0s!\n");

							// tell Command the colour detected and the distance from the camera
							fprintf(fp, "c/r/%i/!\n", distance);

							// update members
							ball->distance = distance;
							ball->seen = TRUE;
							return TRUE;
						}
					}
				}
			// 		Obstacle detection
			// Other if scopes are for actions if we detect a ball that's not the one we are supposed to be going towards.
    	   	}else if (verilog_word == EEE_IMGPROC_MSG_START_G){
				int g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

				int g_left_x = (g_topleft)>>16; 
				int g_right_x = (g_bottomright)>>16;
				int g_left_y = (g_topleft & 0x0000ffff);
				int g_right_y = (g_bottomright & 0x0000ffff);

				if(is_ball(g_left_x, g_right_x, g_left_y, g_right_y) && is_in_centre_range(g_left_x, g_right_x)){
					distance = distance_calc(g_left_x, g_right_x, g_left_y, g_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle green\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}else if (verilog_word == EEE_IMGPROC_MSG_START_B){
				int b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
				int b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

				int b_left_x = (b_topleft)>>16; // extracting the top 16 bits
				int b_right_x = (b_bottomright)>>16;
				int b_left_y = (b_topleft & 0x0000ffff);
				int b_right_y = (b_bottomright & 0x0000ffff);

				if(is_ball(b_left_x, b_right_x, b_left_y, b_right_y) && is_in_centre_range(b_left_x, b_right_x)){
					distance = distance_calc(b_left_x, b_right_x, b_left_y, b_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle blue\n");
						// Update members
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}else if (verilog_word == EEE_IMGPROC_MSG_START_Y){
				int y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

				int y_left_x = (y_topleft)>>16; 
				int y_right_x = (y_bottomright)>>16;
				int y_left_y = (y_topleft & 0x0000ffff);
				int y_right_y = (y_bottomright & 0x0000ffff);

				if(is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && is_in_centre_range(y_left_x, y_right_x)){
					distance = distance_calc(y_left_x, y_right_x, y_left_y, y_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle yellow\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}
    		//}
		}
	}else if(ball->colour == 'G'){
		printf("Going towards green\n");
		while (1) {
            fflush(fp);
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_R){
				int r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

				int r_left_x = (r_topleft)>>16; 
				int r_right_x = (r_bottomright)>>16;
				int r_left_y = (r_topleft & 0x0000ffff);
				int r_right_y = (r_bottomright & 0x0000ffff);

				if(is_ball(r_left_x, r_right_x, r_left_y, r_right_y) && is_in_centre_range(r_left_x, r_right_x)){
					distance = distance_calc(r_left_x, r_right_x, r_left_y, r_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle red\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}else if (verilog_word == EEE_IMGPROC_MSG_START_G){ 
				int g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

				int g_left_x = (g_topleft)>>16; 
				int g_right_x = (g_bottomright)>>16;
				int g_left_y = (g_topleft & 0x0000ffff);
				int g_right_y = (g_bottomright & 0x0000ffff);

                if(is_ball(g_left_x, g_right_x, g_left_y, g_right_y)){
                    int middle_pix = (g_right_x + g_left_x) / 2;
                    if (middle_pix < 240 && s1 == 0){
                        fprintf(fp, "v0l!\n");
                        printf("g : turn left\n");
                        s1 = 1;
						s2 = 0;
						s3 = 0;
                    }else if(middle_pix > 400 && s2 == 0){
                        fprintf(fp, "v0r!\n");
                        printf("g : turn right\n");
                        s1 = 0;
						s2 = 1;
						s3 = 0;
                    }else if (middle_pix <= 400 && middle_pix >= 240 && s3 == 0){
                        fprintf(fp, "v0g!\n");
                        printf("g : go forward\n");
                        s1 = 0;
						s2 = 0;
						s3 = 1;
					}

                    if(is_ball(g_left_x, g_right_x, g_left_y, g_right_y) && middle_pix <= 440 && middle_pix >= 200){
						printf("Is ball and is centred\n", distance);
                        distance = distance_calc(g_left_x, g_right_x, g_left_y, g_right_y);
						printf("Distance : %i\n", distance);
                        if (distance < 80 && distance > 25){
                            printf("Reached green\n");

                            fprintf(fp, "v0s!\n");
							fprintf(fp, "c/g/%i/!\n", distance);

                            ball->distance = distance;
                            ball->seen = TRUE;
                            
                            return TRUE;
                        }
                    }
                }
    	   	} else if (verilog_word == EEE_IMGPROC_MSG_START_B){
				int b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

				int b_left_x = (b_topleft)>>16; 
				int b_right_x = (b_bottomright)>>16;
				int b_left_y = (b_topleft & 0x0000ffff);
				int b_right_y = (b_bottomright & 0x0000ffff);

				if(is_ball(b_left_x, b_right_x, b_left_y, b_right_y) && is_in_centre_range(b_left_x, b_right_x)){
					distance = distance_calc(b_left_x, b_right_x, b_left_y, b_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle blue\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}else if (verilog_word == EEE_IMGPROC_MSG_START_Y){
				int y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
				int y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				int y_left_x = (y_topleft)>>16; 
				int y_right_x = (y_bottomright)>>16;
				int y_left_y = (y_topleft & 0x0000ffff);
				int y_right_y = (y_bottomright & 0x0000ffff);

				if(is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && is_in_centre_range(y_left_x, y_right_x)){
					distance = distance_calc(y_left_x, y_right_x, y_left_y, y_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle yellow\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}
		}
	}else if(ball->colour == 'B'){
		printf("Going towards blue\n");
		while (1) {
            fflush(fp);
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_R){
				int r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

				int r_left_x = (r_topleft)>>16; 
				int r_right_x = (r_bottomright)>>16;
				int r_left_y = (r_topleft & 0x0000ffff);
				int r_right_y = (r_bottomright & 0x0000ffff);

				if(is_ball(r_left_x, r_right_x, r_left_y, r_right_y) && is_in_centre_range(r_left_x, r_right_x)){
					distance = distance_calc(r_left_x, r_right_x, r_left_y, r_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle red\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}else if (verilog_word == EEE_IMGPROC_MSG_START_G){
				int g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				int g_left_x = (g_topleft)>>16; 
				int g_right_x = (g_bottomright)>>16;
				int g_left_y = (g_topleft & 0x0000ffff);
				int g_right_y = (g_bottomright & 0x0000ffff);

				if(is_ball(g_left_x, g_right_x, g_left_y, g_right_y) && is_in_centre_range(g_left_x, g_right_x)){
					distance = distance_calc(g_left_x, g_right_x, g_left_y, g_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle green\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}else if (verilog_word == EEE_IMGPROC_MSG_START_B){ 
				int b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

                int b_left_x = (b_topleft)>>16; 
				int b_right_x = (b_bottomright)>>16;
				int b_left_y = (b_topleft & 0x0000ffff);
				int b_right_y = (b_bottomright & 0x0000ffff);

                if(is_ball(b_left_x, b_right_x, b_left_y, b_right_y)){
                    int middle_pix = (b_right_x + b_left_x) / 2;
					if (middle_pix < 240 && s1 == 0){
						fprintf(fp, "v0l!\n");
						printf("b : turn left\n");
						s1 = 1;
						s2 = 0;
						s3 = 0;
					}else if(middle_pix > 400 && s2 == 0){
						fprintf(fp, "v0r!\n");
						printf("b : turn right\n");
						s1 = 0;
						s2 = 1;
						s3 = 0;
					}else if (middle_pix <= 400 && middle_pix >= 240 && s3 == 0){
						fprintf(fp, "v0g!\n");
						printf("b : go forward\n");
						s1 = 0;
						s2 = 0;
						s3 = 1;
					}

                    if(is_ball(b_left_x, b_right_x, b_left_y, b_right_y) && middle_pix <= 440 && middle_pix >= 200){
						printf("Is ball and is centred\n", distance);
                        distance = distance_calc(b_left_x, b_right_x, b_left_y, b_right_y);
						printf("Distance : %i\n", distance);
                        if (distance < 80 && distance > 25){
                            printf("Reached blue\n");

                            fprintf(fp, "v0s!\n");
							fprintf(fp, "c/b/%i/!\n", distance);

                            ball->distance = distance;
                            ball->seen = TRUE;

                            return TRUE;
                        }
                    }
                }
    	   	}else if (verilog_word == EEE_IMGPROC_MSG_START_Y){
				int y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

				int y_left_x = (y_topleft)>>16; 
				int y_right_x = (y_bottomright)>>16;
				int y_left_y = (y_topleft & 0x0000ffff);
				int y_right_y = (y_bottomright & 0x0000ffff);

				if(is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && is_in_centre_range(y_left_x, y_right_x)){
					distance = distance_calc(y_left_x, y_right_x, y_left_y, y_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle yellow\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}
		}
	}else if(ball->colour == 'Y'){
		printf("Going towards yellow\n");
		while (1) {
            fflush(fp);
			int verilog_word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
			if (verilog_word == EEE_IMGPROC_MSG_START_R){
				int r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

				int r_left_x = (r_topleft)>>16; 
				int r_right_x = (r_bottomright)>>16;
				int r_left_y = (r_topleft & 0x0000ffff);
				int r_right_y = (r_bottomright & 0x0000ffff);

				if(is_ball(r_left_x, r_right_x, r_left_y, r_right_y) && is_in_centre_range(r_left_x, r_right_x)){
					distance = distance_calc(r_left_x, r_right_x, r_left_y, r_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle red\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}else if (verilog_word == EEE_IMGPROC_MSG_START_G){
				int g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

				int g_left_x = (g_topleft)>>16;
				int g_right_x = (g_bottomright)>>16;
				int g_left_y = (g_topleft & 0x0000ffff);
				int g_right_y = (g_bottomright & 0x0000ffff);

				if(is_ball(g_left_x, g_right_x, g_left_y, g_right_y) && is_in_centre_range(g_left_x, g_right_x)){
					distance = distance_calc(g_left_x, g_right_x, g_left_y, g_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle green\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			} else if (verilog_word == EEE_IMGPROC_MSG_START_B){
				int b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 

				int b_left_x = (b_topleft)>>16; 
				int b_right_x = (b_bottomright)>>16;
				int b_left_y = (b_topleft & 0x0000ffff);
				int b_right_y = (b_bottomright & 0x0000ffff);

				if(is_ball(b_left_x, b_right_x, b_left_y, b_right_y) && is_in_centre_range(b_left_x, b_right_x)){
					distance = distance_calc(b_left_x, b_right_x, b_left_y, b_right_y);
					if (distance < 40 && distance > 20){
						printf("Facing obstacle blue\n");
						fprintf(fp, "v0s!\n");
						return TRUE;
					}
				}
			}else if (verilog_word == EEE_IMGPROC_MSG_START_Y){ 
				int y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); 
				int y_left_x = (y_topleft)>>16; 
				int y_right_x = (y_bottomright)>>16;
				int y_left_y = (y_topleft & 0x0000ffff);
				int y_right_y = (y_bottomright & 0x0000ffff);

                if(is_ball(y_left_x, y_right_x, y_left_y, y_right_y)){
                    int middle_pix = (y_right_x + y_left_x) / 2;
					if (middle_pix < 240 && s1 == 0){
						fprintf(fp, "v0l!\n");
						printf("y : turn left\n");
						s1 = 1;
						s2 = 0;
						s3 = 0;
					}else if(middle_pix > 400 && s2 == 0){
						fprintf(fp, "v0r!\n");
						printf("y : turn right\n");
						s1 = 0;
						s2 = 1;
						s3 = 0;
					}else if (middle_pix <= 400 && middle_pix >= 240 && s3 == 0){
						fprintf(fp, "v0g!\n");
						printf("y : go forward\n");
						s1 = 0;
						s2 = 0;
						s3 = 1;
					}

                    if(is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && middle_pix <= 440 && middle_pix >= 200){
						printf("Is ball and is centred\n", distance);
                        distance = distance_calc(y_left_x, y_right_x, y_left_y, y_right_y);
						printf("Distance : %i\n", distance);
                        if (distance < 80 && distance > 25){
                            printf("Reached yellow\n");
                            fprintf(fp, "v0s!\n");
							fprintf(fp, "c/y/%i/!\n", distance);

                            ball->distance = distance;
                            ball->seen = TRUE;

                            return TRUE;
                        }
                    }
                }
    	   	}
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

	// Initialising the member variables
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

	// Single variable to capture distances
	int distance;

	// 2nd scan specific variables
	int s2_balls_detected = 0;
	Ball nothing;
	Ball *closestBall;
	closestBall = &nothing;
	closestBall->distance = 500;

	// Other
	int state = 0; // or stage
	int second_scan_done = 0;
	int foundbit = 0;

	int incomingChar1;

	// Initial message to verify that information is transferred
	fprintf(fp, "v0g!\n");
	printf("Sent v0g\n");

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

       	//Read messages from the image processor and ESP32
       	while (1){
			// flushing past outgoing messages to avoid oversending information
       		fflush(fp);
       		//printf("Starting while loop\n");

			if (balls_detected == 5){
				// send signal which indicates that all 5 balls have been detected
				foundbit = 1;
				printf("All balls detected\n");
				fprintf(fp, "v%ig!\n", foundbit);
			}

			// Cannot read boundary box coordinate information in every loop of while(1) since information is not always
			//		available so when it isn't available, we just read the incoming character from the ESP32

    		if((IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_STATUS)>>8) & 0xff){ // if verilog information is available

				// ---------- Start of analysing incoming information from EEE_IMGPROC ---------
    			int word = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Read the incoming message from EEE_IMGPROC

				// Analyse incoming BB information and verilog and make the proper variable assignments
				if (word == EEE_IMGPROC_MSG_START_R){ // If the incoming string == RBB
					r_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (top left coordinate)
					r_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG); // Grab the next word (bottom right coordinate)

					r_left_x = (r_topleft)>>16; 
					r_right_x = (r_bottomright)>>16;
					r_left_y = (r_topleft & 0x0000ffff);
					r_right_y = (r_bottomright & 0x0000ffff);
				} else if (word == EEE_IMGPROC_MSG_START_G){ // If the incoming string == GBB
					g_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					g_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

					g_left_x = (g_topleft)>>16; 
					g_right_x = (g_bottomright)>>16;
					g_left_y = (g_topleft & 0x0000ffff);
					g_right_y = (g_bottomright & 0x0000ffff);
				} else if (word == EEE_IMGPROC_MSG_START_B){ // If the incoming string == BBB
					b_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					b_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

					b_left_x = (b_topleft)>>16; 
					b_right_x = (b_bottomright)>>16;
					b_left_y = (b_topleft & 0x0000ffff);
					b_right_y = (b_bottomright & 0x0000ffff);
				} else if (word == EEE_IMGPROC_MSG_START_V){ // If the incoming string == VBB
					v_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					v_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

					v_left_x = (v_topleft)>>16;
					v_right_x = (v_bottomright)>>16;
					v_left_y = (v_topleft & 0x0000ffff);
					v_right_y = (v_bottomright & 0x0000ffff);
				} else if (word == EEE_IMGPROC_MSG_START_Y){ // If the incoming string == YBB
					y_topleft = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);
					y_bottomright = IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_MSG);

					y_left_x = (y_topleft)>>16;
					y_right_x = (y_bottomright)>>16;
					y_left_y = (y_topleft & 0x0000ffff);
					y_right_y = (y_bottomright & 0x0000ffff);
				}
				// Variables from incoming verilog info have been assigned, data is ready
    		}else{ // if verilog information is not available, read the current state from Drive

    	   		// ---------- Analyse incoming string of characters from Control -------------
				incomingChar1 = getc(fp); // read the character sent by the ESP32
    	   		//printf("Received : %c\n", incomingChar1);
    			if(incomingChar1 == '0'){ // In the 1st scan
    				printf("s0 - 1st scan\n");
    				state = 0;
    			}else if (incomingChar1 == '1'){ // End of first 360 scan, now in the 2nd scan
    	   			printf("s1 - 1st scan done\n");
    	   			state = 1;
    	   		}else if (incomingChar1 == '2'){ // End of second 360 scan
    	   			// we instantly jump from 2 to 3 so any processing in 2 occurs in 3
    	   		}else if (incomingChar1 == '3'){ // Starting 3rd scan - look for the closest ball
				   	// triggered if no balls were spotted in the 2nd scan
    	   			printf("s2/3 - end of 2nd scan, starting 3rd scan\n");
    	   			second_scan_done = 1;
    	   		}else if(incomingChar1 == '4'){ // 4 indicates that we are now facing the ball
    	   			printf("s4 - now facing the closest or first ball\n");
    	   		}else if(incomingChar1 == '5'){
    	   			printf("s5\n");
					
					// reset any seen members so that balls are picked up in repeat scans if they have not yet been seen
    	   			if (redBall_ptr->seen == FALSE){
    	   				redBall_ptr->seen1 = FALSE;
    	   				redBall_ptr->seen2 = FALSE;
    	   				redBall_ptr->seen3 = FALSE;
    	   			}
    	   			if (greenBall_ptr->seen == FALSE){
    	   				greenBall_ptr->seen1 = FALSE;
    	   				greenBall_ptr->seen2 = FALSE;
    	   				greenBall_ptr->seen3 = FALSE;
    	   			}
    	   			if (blueBall_ptr->seen == FALSE){
    	   				blueBall_ptr->seen1 = FALSE;
    	   				blueBall_ptr->seen2 = FALSE;
    	   				blueBall_ptr->seen3 = FALSE;
    	   			}
    	   			if (yellowBall_ptr->seen == FALSE){
    	   				yellowBall_ptr->seen1 = FALSE;
    	   				yellowBall_ptr->seen2 = FALSE;
    	   				yellowBall_ptr->seen3 = FALSE;
    	   			}
    	   		}else if (incomingChar1 == '6'){ // 6 indicates that we've gotten to the ball
    	   			printf("s6 - reached the ball, repeat the 1st scan");
    	   			state = 0; // return to the 1st scan
    	   		}
				
				// ---------- End of analysing incoming chars from Control ---------

    		}
			
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
					printf("Sent g\n");
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
				}else if ((yellowBall_ptr->seen == FALSE) && (yellowBall_ptr->seen1 == FALSE) && is_ball(y_left_x, y_right_x, y_left_y, y_right_y) && is_in_centre_range(y_left_x, y_right_x)){
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
				if(second_scan_done == 1){ //|| s2_balls_detected == 3){//state1 == 1){ // fill with char that we expect once the full 360 for stage 2 is done
					//printf("balls detected in s2 : %i\n", s2_balls_detected);
					if (s2_balls_detected == 0){ // no vague distances
						state = 2; // advance to stage 2 - first ball we see
						second_scan_done = 0;
					}else{ // go to the closest ball
						printf("Searching for closest ball\n");
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

									// resetting variables
									s2_balls_detected = 0;
									closestBall = NULL;
									printf("Reset the closest ball\n");
									second_scan_done = 0;
									fflush(fp);
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

									// resetting variables
									s2_balls_detected = 0;
									closestBall = NULL;
									printf("Reset the closest ball\n");
									second_scan_done = 0;
									fflush(fp);
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

									// resetting variables
									s2_balls_detected = 0;
									closestBall = NULL;
									printf("Reset the closest ball\n");
									second_scan_done = 0;
									fflush(fp);
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

									// resetting variables
									s2_balls_detected = 0;
									closestBall = NULL;
									printf("Reset the closest ball\n");
									second_scan_done = 0;
									fflush(fp);
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

	   //Main loop delay
	   usleep(10000);

   	};
	fclose(fp);
  	return 0;
}