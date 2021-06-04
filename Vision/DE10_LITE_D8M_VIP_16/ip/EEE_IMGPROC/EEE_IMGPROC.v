// Creating the module block for the image processor
module EEE_IMGPROC(
	//		Labelling all signals that this block uses - inputs and outputs; we determine which they are after

	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// conduit
	mode
	
);

// 		Establishing which are inputs and outputs along with how many bits per signal

// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//

//		Setting parameters - values permanently tied to a variable
//	Width of the frames captured by the camera in pixels
parameter IMAGE_W = 11'd640;
//	Height of the frames
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 6;
parameter BB_COL_DEFAULT = 24'h00ff00;
parameter R_BB_COL_DEFAULT = 24'hff7575; // light red 
parameter G_BB_COL_DEFAULT = 24'h92ff96;
parameter B_BB_COL_DEFAULT = 24'h92eeff;
parameter V_BB_COL_DEFAULT = 24'hff92ea;
parameter Y_BB_COL_DEFAULT = 24'hf9ff92;

//	Wires are intermediate signals we use to simplify logic; recall that we
//		can set a wire to only look at a specific amount of bits from a signal
//		or wires can be used to hold temporary values

// Red, green, blue and grey are 8 bit values since the camera produces 8 bits
//		of colour depth per channel. 
//	I'm assuming we access these colour channels through these variables so these 
//		variables store the RGB values of the pixel we are looking at detected from the camera
wire [7:0] red, green, blue;
// Grey is the variable storing the colour for the grey we will be replacing the 
//		non-colour pixels with
// wire [7:0] grey;
//	The output colour channels which the camera outputs through the VGA cable
wire [7:0]   red_out, green_out, blue_out;

// sop - start of packet
// eop - end of packet
// in_valid - enable signal controlled by the previous register in the stream pipeline
wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////

// ----------------------- Mapping RGB to HSV -----------------------
// https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html
//Implementation with just ternary operators
wire [7:0] h, s, v;
wire [7:0] min;
wire [7:0] s_temp, v_temp, h_temp;

// not divided by 255
assign v_temp = (red >= green && red >= blue) ? red
				: (green >= red && green >= blue) ? green
				: (blue >= red && blue >= green) ? blue
				: 0;
	
assign v = v_temp; // *100/255; // scale to percent

// not divided by 255
assign min = red <= green && red <= blue ? red
				: green <= red && green <= blue ? green
				: blue <= red && blue <= green ? blue
				: 0;

// scaled up by 255
assign s_temp = v_temp != 0 ? 255*(v_temp - min)/v_temp
					: 0;
						
assign s = s_temp; // *100/255; // scaling to percentage
						
// v_temp and the colours are not scaled down
assign h_temp = (red == green && green == blue) ? 0
			: v_temp == red ? //60*(green - blue)/(v_temp - min) // if V == R
					green >= blue ? 60*(green - blue)/(v_temp - min) // if g-b is positive, dont add 360
					: (360*(v_temp-min) + 60*(green - blue))/(v_temp - min) // if negative, add 360
			: v_temp == green ? (120*(v_temp - min) + 60*(blue - red))/(v_temp - min)
			: v_temp == blue ?	(240*(v_temp - min) + 60*(red - green))/(v_temp - min)
			: 0;
				
assign h = h_temp *1/2; // H correction to fit colourmap
// --------------------- Colour detection (HSV) ---------------------

wire red_detect, green_detect, blue_detect, violet_detect, yellow_detect, border_detect;

// 	Original HSV ranges - based on images
// red_ = (h > 12 && h < 48 && s > 42 && v > 40) ? 1 : 0;
// green = (h > 145 && h < 175 && s > 45 && v < 85 && v > 20) ? 1 : 0;
// blue = (h > 190 && h < 240 && s > 70 && v > 25 && v < 75) ? 1 : 0;
// violet =  ((h > 330 || h < 30) && s > 10 && s < 60 && v < 80 && v > 20) ? 1 : 0;
// yellow = (h > 55 && h < 85 && s > 25 && s < 75 && v > 40) ? 1 : 0;

// New HSV ranges based on colourmap : https://stackoverflow.com/questions/47483951/how-to-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-image
//might be better to scale up the values on the colourmap
//	All have default limits stopping low s and high v values to stop picking up pure white - do the same for black
//assign red_det = ((h > 165 && s > 10 && s < 150 && v > 10 && v < 245) || (h < 15 && s > 160 && v > 102 && v < 230)) ? 1 : 0; // good - acquire a better range, taking away the lower hue range to avoid overlap with purple

// x, y - registers storing the current pixel coordinate
//		Can set if parameters on these to trigger new conditions
//	Width in pixels = 640; have the 210 pixels on the left and right as the special cases

assign red_detect = (x >= 210 && x <= 430) && ((h > 170 || h < 25) && s > 149 && s < 232 && v > 125 && v <= 200) ? 1 // changed max v from 245 to 200
						: ( (x < 210 || x > 430) && (h > 170 || h < 15) && s > 75 && s < 161 && v > 100 && v < 200) ? 1 // 235 -> 161
						: 0;
						
assign blue_detect = (h > 90 && h < 120 && s > 76 && s < 195 && v > 25 && v < 128) ? 1 : 0; // good - check previous commits for recent values
assign green_detect = (h > 60 && h < 80 && s > 75 && s < 200 && v > 50 && v < 128) ? 1 : 0; // good (h > 115 && h < 135 && s > 200 && v > 115 && v < 135)

assign violet_detect =  (h < 15 && s > 80 && s < 150 && v > 75 && v <= 125) ? 1 : 0;
								// (x >= 210 && x <= 430 && h < 15 && s > 80 && s < 150 && v > 75 && v <= 125) ? 1 // central third
								//: ( (x < 210 || x > 430) && h > 9 && h < 17 && s > 160 && s < 235 && v > 88 && v < 200) ? 1 // left and right
								// : (x < 210 || x > 430) ? 1 // set everything in the right and left side to violet
								//: ( x > 430 && ) ? 1// right third
								//: 0;

// (h < 25 && s > 80 && s < 160 && v > 50 && v <= 125) ? 1 : 0;// ((h < 20  && s > 89 && s < 170 && v > 64 && v < 128) || (h > 160 && s > 10 && s < 115 && v > 154 && v < 245)) ? 1 : 0; // bad
//assign violet_detect = 0;
assign yellow_detect = (h > 25 && h < 45 && s > 100 && s < 153 && v > 102 && v <= 245) ? 1 : 0; // good - needs adjusting?

assign border_detect = (x <= 20 || x >= 620) || (y <= 20 || y >= 460); 

// works well for blue : ((h > 165 || h < 15) && s > 65) ? 1 : 0; for non-scaled h

// red = ((h > 165 && s > 10 && s < 150 && v > 10 && v < 245) || (h < 15 && s > 160 && v > 102 && v < 230)) ? 1 : 0; // good - acquire a better range, taking away the lower hue range to avoid overlap with purple
// violet =  (h < 20 && s > 76 && s < 160 && v < 192 && v > 50) ? 1 : 0; // bad


// ------------------ Highlight detected areas -------------------
//	24 bit value which stores the concatenated 8-bit RGB values together for each colour 
wire [23:0] red_high, green_high, blue_high, violet_high, yellow_high, black;

// assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4
assign black = 24'h0;
wire [23:0] background; // variable storing what we want to fill the background colour with (colours not detected)

assign background = black;
// {grey, grey, grey}

assign red_high  =  {8'hff, 8'h0, 8'h01};
assign green_high  =  {8'h0, 8'hff, 8'h0}; 
assign blue_high  =  {8'h0, 8'h0, 8'hff}; 
assign violet_high  =  {8'h8e, 8'h15, 8'h96}; 
assign yellow_high  =  {8'hed, 8'hff, 8'h6f};

// 		Show bounding box
//	new_image - Output pixel variable that is either the pure output colour pixel or the box colour
wire [23:0] new_image;

assign new_image = border_detect ? background
						: red_detect ? red_high
						: green_detect ? green_high
						: blue_detect ? blue_high
						: violet_detect ? violet_high
						: yellow_detect ? yellow_high
						: background;

// -------------------------- Filtering --------------------------

// new_image stores the current output pixel (post colour detection) where the RGB values are 
//		concatenated into a single 24-bit value.
//	To filter, we must store the previous x new_image values and process them accordingly unless there is
//		a method of accessing the pixels seperately.

//	Storing the previous 3 pixel values

reg [23:0] pixels [0:2]; // array of 3 pixels, each of size 24 bits
reg [15:0] pixels_hue [0:2];
integer i, j;

reg [15:0] smallest_hue, largest_hue;
reg smallest_hue_index, largest_hue_index, median_hue_index;

reg [15:0] temp;
reg [23:0] filtered_image; // variable to store the filtered pixel

// update each term in the array at each clock edge
always@(posedge clk) begin
	// move values down the filter array to make space for the current pixel
	for (i = 0; i < 2; i = i+1) begin	
		pixels[i] = pixels[i+1]; // blocking assignment, combinational logic
	end
	
	// append the current pixel to the array
	pixels[2] = new_image;
	
/*
		Pixel values
	red_high  =  {8'hff, 8'h0, 8'h01}; h = 360, s = 100, v = 100
	green_high  =  {8'h0, 8'hff, 8'h0}; h = 120, s = 100, v = 100
	blue_high  =  {8'h0, 8'h0, 8'hff}; h = 240, s = 100, v = 100
	violet_high  =  {8'h8e, 8'h15, 8'h96}; h = 296, s = 86, v = 58.8
	yellow_high  =  {8'hed, 8'hff, 8'h6f}; h = 68, 56.5, v = 100
	black = 24'h0;
*/

	// 		Calculate the median of the pixels - based on hue
	// 	Finding the hue of each pixel
	for (i = 0; i < 3; i = i+1) begin
		if (pixels[i] == red_high) begin
			pixels_hue[i] = 360;
		end else if (pixels[i] == green_high) begin
			pixels_hue[i] = 120;
		end else if (pixels[i] == blue_high) begin
			pixels_hue[i] = 240;
		end else if (pixels[i] == violet_high) begin
			pixels_hue[i] = 296;
		end else if (pixels[i] == yellow_high) begin
			pixels_hue[i] = 68;
		end else if (pixels[i] == background) begin
			pixels_hue[i] = 0;
		end else begin
			pixels_hue[i] = 0;
		end
	end
	
/*
	// 	Forming the sorted array based on the hue using bubble sort - very inefficient, was just for functionality
	//	Can improve the sorting algorithm used in the future
	for (i = 0; i < 5; i = i+1) begin
		// might be able to change j < 3 to j < 3 - i, check OneNote
		for (j = 0; j < 4; j = j+1) begin
			if (pixels_hue[j] > pixels_hue[j+1]) begin
				// swap them in the array
				temp = pixels_hue[j];
				pixels_hue[j] = pixels_hue[j+1];
				pixels_hue[j+1] = temp;
				// larger value now swapped so that it is at the larger index
			end
		end
	end
*/

	//		Forming the sorted array just by finding the smallest and largest values - much more efficient - O(n)

smallest_hue <= pixels_hue[0];
largest_hue <= pixels_hue[0];
smallest_hue_index <= 0;
largest_hue_index <= 0;

	for (i = 1; i < 3; i = i+1) begin
		if (pixels_hue[i] < smallest_hue) begin
			smallest_hue <= pixels_hue[i];
			smallest_hue_index <= i;
		end else if (pixels_hue[i] > largest_hue) begin
			largest_hue <= pixels_hue[i];
			largest_hue_index <= i;
		end else begin
		end
	end
	
	if ((smallest_hue_index == 0 && largest_hue_index == 1) || (smallest_hue_index == 1 && largest_hue_index == 0)) begin
		median_hue_index = 2;
	end else if ((smallest_hue_index == 0 && largest_hue_index == 2) || (smallest_hue_index == 2 && largest_hue_index == 0)) begin
		median_hue_index = 1;
	end else if ((smallest_hue_index == 1 && largest_hue_index == 2) || (smallest_hue_index == 2 && largest_hue_index == 1)) begin
		median_hue_index = 0;
	end else begin
		median_hue_index = 1;
	end

	//		Selecting the median value as the pixel and converting back to RGB
	//	Replace median_hue_index with 1 if using bubble sort function
	//median_hue_index = 1;
	if (pixels_hue[median_hue_index] == 360) begin // select red
		filtered_image = red_high;
	end else if (pixels_hue[median_hue_index] == 120) begin // select green
		filtered_image = green_high;
	end else if (pixels_hue[median_hue_index] == 240) begin // select blue
		filtered_image = blue_high;
	end else if (pixels_hue[median_hue_index] == 296) begin // select violet
		filtered_image = violet_high;
	end else if (pixels_hue[median_hue_index] == 68) begin // select yellow
		filtered_image = yellow_high;
	end else if (pixels_hue[median_hue_index] == 0) begin // select black
		filtered_image = background;
	end else begin
		filtered_image = background;
	end
end

//	---------- End of Filtering ----------

//	---------- Applying bounding boxes to each colour ----------
// Make assignments based on the state of filtered_image (if filtered_image == colour_high)
reg [10:0] r_xmin, r_ymin, r_xmax, r_ymax; // red
reg [10:0] g_xmin, g_ymin, g_xmax, g_ymax; // green
reg [10:0] b_xmin, b_ymin, b_xmax, b_ymax; // blue
reg [10:0] v_xmin, v_ymin, v_xmax, v_ymax; // violet
reg [10:0] y_xmin, y_ymin, y_xmax, y_ymax; // yellow

wire r_bb_active, g_bb_active, b_bb_active, v_bb_active, y_bb_active;
assign r_bb_active = (x == r_left) | (x == r_right) | (y == r_top) | (y == r_bottom);
assign g_bb_active = (x == g_left) | (x == g_right) | (y == g_top) | (y == g_bottom);
assign b_bb_active = (x == b_left) | (x == b_right) | (y == b_top) | (y == b_bottom);
assign v_bb_active = (x == v_left) | (x == v_right) | (y == v_top) | (y == v_bottom);
assign y_bb_active = (x == y_left) | (x == y_right) | (y == y_top) | (y == y_bottom);

wire [23:0] bounded_image;
assign bounded_image = r_bb_active ? r_bb_col
							: g_bb_active ? g_bb_col
							: b_bb_active ? b_bb_col
							: v_bb_active ? v_bb_col
							: y_bb_active ? y_bb_col
							: filtered_image;
// ---------- End of applying bounding boxes ----------
						
//		Adjusting the pixels output to the screen
// Switch output pixels depending on mode switch; determined by the signal 'mode'
// Don't modify the start-of-packet word - it's a packet descriptor; determined by the signal 'sop'
// Don't modify data in non-video packets; determined by only making the assignment if packet_video == 1
//		packet_video tells us that the current packet is part of the video

// If mode == 1 and sop == 0 and packet_video == 1, assign the output to the new image - otherwise keep it the same
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? bounded_image : {red,green,blue};

//		Count valid pixels to get the image coordinates. Reset and detect packet type on Start of Packet.

//	x and y are register types which store the current pixel coordinates we are looking at.
//		These coordinates are 11-bit values
reg [10:0] x, y;
//	packet_video determines if the current packet we are looking at is a video packet or not
//	1 = packet is a video type
reg packet_video;

// This block updates what pixel we are looking at with each clock edge
always@(posedge clk) begin
	if (sop) begin
		// If it is the start of the packet (representing the frame), set the x and y values to just 0s
		x <= 11'h0;
		y <= 11'h0;
		// If the 4 LSBs of blue match the 3-bit hex value of 0 (?), update the packet_video variable
		packet_video <= (blue[3:0] == 3'h0);
	end
	// If the enable signal of the previous register in the pipeline is high - means the input at this clock edge is valid
	else if (in_valid) begin
		// if the x-coordinate is at the right edge of the frame
		if (x == IMAGE_W-1) begin
			//	reset the x-coordinate to go back to the beginning (leftmost bit)
			x <= 11'h0;
			//	increase the height of the y-coordinate so that you are on the next row of pixels
			y <= y + 11'h1;
		end
		// else if we aren't at the end of the row, just continue incrementing along the row
		else begin
			x <= x + 11'h1;
		end
	end
end

//Find first and last pixels of each colour
always@(posedge clk) begin
		// update bounding boxes for each box post filtering
	if (filtered_image == red_high & in_valid) begin
		if (x < r_xmin) r_xmin <= x; // continually update x_min, x_max, y_min and y_max if the current red pixel
		if (x > r_xmax) r_xmax <= x;	// 	is beyond the bounds of the current frame
		if (y < r_ymin) r_ymin <= y;
		r_ymax <= y;
	end else if(filtered_image == green_high & in_valid) begin
		if (x < g_xmin) g_xmin <= x; 
		if (x > g_xmax) g_xmax <= x;
		if (y < g_ymin) g_ymin <= y;
		g_ymax <= y;
	end else if(filtered_image == blue_high & in_valid) begin
		if (x < b_xmin) b_xmin <= x; 
		if (x > b_xmax) b_xmax <= x;
		if (y < b_ymin) b_ymin <= y;
		b_ymax <= y;
	end else if(filtered_image == violet_high & in_valid) begin
		if (x < v_xmin) v_xmin <= x; 
		if (x > v_xmax) v_xmax <= x;
		if (y < v_ymin) v_ymin <= y;
		v_ymax <= y;
	end else if(filtered_image == yellow_high & in_valid) begin
		if (x < y_xmin) y_xmin <= x; 
		if (x > y_xmax) y_xmax <= x;
		if (y < y_ymin) y_ymin <= y;
		y_ymax <= y;
	end
	
	// The sections above and below need to be in the same always@(posedge clk)
	//		block otherwise it causes errors due to making assignments of the same variable
	//		in different always@(*) blocks
	
	//Reset bounds on start of packet
	if (sop & in_valid) begin
		r_xmin <= IMAGE_W-11'h1;
		g_xmin <= IMAGE_W-11'h1;
		b_xmin <= IMAGE_W-11'h1;
		v_xmin <= IMAGE_W-11'h1;
		y_xmin <= IMAGE_W-11'h1;
		
		r_xmax <= 0;
		g_xmax <= 0;
		b_xmax <= 0;
		v_xmax <= 0;
		y_xmax <= 0;
		
		r_ymin <= IMAGE_H-11'h1;
		g_ymin <= IMAGE_H-11'h1;
		b_ymin <= IMAGE_H-11'h1;
		v_ymin <= IMAGE_H-11'h1;
		y_ymin <= IMAGE_H-11'h1;		
		
		r_ymax <= 0;
		g_ymax <= 0;
		b_ymax <= 0;
		v_ymax <= 0;
		y_ymax <= 0;

	end
end

//		Process bounding box at the end of the frame.
//	This block is completed after we have gone through each pixel and determined
//		what the actual minimum and maximum values

//	Variable storing what message type we are sending to the processor
reg [3:0] msg_state; // need 16 states overall for all 5 balls, should be 4 bits
// storing the coordinates of the left, right, top and bottom positions of the boundary box in registers
reg [10:0] r_left, r_right, r_top, r_bottom;
reg [10:0] g_left, g_right, g_top, g_bottom;
reg [10:0] b_left, b_right, b_top, b_bottom;
reg [10:0] v_left, v_right, v_top, v_bottom;
reg [10:0] y_left, y_right, y_top, y_bottom;

reg [7:0] frame_count;
always@(posedge clk) begin
	if (eop & in_valid & packet_video) begin  //Ignore non-video packets
		
		//Latch edges for display overlay on next frame
		r_left <= r_xmin;
		r_right <= r_xmax;
		r_top <= r_ymin;
		r_bottom <= r_ymax;
		
		g_left <= g_xmin;
		g_right <= g_xmax;
		g_top <= g_ymin;
		g_bottom <= g_ymax;
		
		b_left <= b_xmin;
		b_right <= b_xmax;
		b_top <= b_ymin;
		b_bottom <= b_ymax;
		
		v_left <= v_xmin;
		v_right <= v_xmax;
		v_top <= v_ymin;
		v_bottom <= v_ymax;
		
		y_left <= y_xmin;
		y_right <= y_xmax;
		y_top <= y_ymin;
		y_bottom <= y_ymax;
		
		
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		frame_count <= frame_count - 1;
		
		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 2'b0001;
			frame_count <= MSG_INTERVAL-1;
		end
	end
	
	//Cycle through message writer states once started
	if (msg_state != 2'b0000) msg_state <= msg_state + 2'b0001;

end
	
	// ------------- Communicating with the CPU -------------
//Generate output messages for CPU - how we communicate with the FPGA/eclipse
reg [31:0] msg_buf_in; // what we write to the processor (accessed in Eclipse)
wire [31:0] msg_buf_out; // wire connected to the output of the FIFO register - information first passes through the FIFO register before getting to the CPU
reg msg_buf_wr; // write request; being high means we write to FIFO and so information can be transferred to Eclipse
wire msg_buf_rd, msg_buf_flush; 
wire [7:0] msg_buf_size;
wire msg_buf_empty;

// Assigning the string RBB to the variable RED_BOX_MSG_ID
`define RED_BOX_MSG_ID "RBB"
`define GREEN_BOX_MSG_ID "GBB"
`define BLUE_BOX_MSG_ID "BBB"
`define VIOLET_BOX_MSG_ID "VBB"
`define YELLOW_BOX_MSG_ID "YBB"

always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		2'b0000: begin
			msg_buf_in = 32'b0;
			msg_buf_wr = 1'b0;
		end
		2'b0001: begin
			msg_buf_in = `RED_BOX_MSG_ID;	//Message ID
			msg_buf_wr = 1'b1;
		end
		2'b0010: begin
			msg_buf_in = {5'b0, r_xmin, 5'b0, r_ymin};	//Top left coordinate
			msg_buf_wr = 1'b1;
		end
		2'b0011: begin
			msg_buf_in = {5'b0, r_xmax, 5'b0, r_ymax}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
		2'b0100: begin
			msg_buf_in = `GREEN_BOX_MSG_ID;	//Message ID
			msg_buf_wr = 1'b1;
		end
		2'b0101: begin
			msg_buf_in = {5'b0, g_xmin, 5'b0, g_ymin};	//Top left coordinate
			msg_buf_wr = 1'b1;
		end
		2'b0110: begin
			msg_buf_in = {5'b0, g_xmax, 5'b0, g_ymax}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
		2'b0111: begin
			msg_buf_in = `BLUE_BOX_MSG_ID;	//Message ID
			msg_buf_wr = 1'b1;
		end
		2'b1000: begin
			msg_buf_in = {5'b0, b_xmin, 5'b0, b_ymin};	//Top left coordinate
			msg_buf_wr = 1'b1;
		end
		2'b1001: begin
			msg_buf_in = {5'b0, b_xmax, 5'b0, b_ymax}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
		2'b1010: begin
			msg_buf_in = `VIOLET_BOX_MSG_ID;	//Message ID
			msg_buf_wr = 1'b1;
		end
		2'b1011: begin
			msg_buf_in = {5'b0, v_xmin, 5'b0, v_ymin};	//Top left coordinate
			msg_buf_wr = 1'b1;
		end
		2'b1100: begin
			msg_buf_in = {5'b0, v_xmax, 5'b0, v_ymax}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
		2'b1101: begin
			msg_buf_in = `YELLOW_BOX_MSG_ID;	//Message ID
			msg_buf_wr = 1'b1;
		end
		2'b1110: begin
			msg_buf_in = {5'b0, y_xmin, 5'b0, y_ymin};	//Top left coordinate
			msg_buf_wr = 1'b1;
		end
		2'b1111: begin
			msg_buf_in = {5'b0, y_xmax, 5'b0, y_ymax}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
	endcase
end

// ----------- Beyond here, it's just the connection setup that allows us to communicate with Eclipse -----------
//	We process the data on Eclipse/the processor and send it to the control module from Eclipse

//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);

// 		Pipeling stages
//	These two instances of the submodule STREAM_REG are pipeline stages placed between components of
//	 the system to allow for pipelining capability

//		Signals and what they do	
//	in_valid - enable signal controlled by the previous register in the stream pipeline
//	out_valid - enable signal generated for the next stage
//	ready_in - backpressure input from the next stage; tells us that the output is
//		ready to receive data
//	ready_out - backpressure output to the previous stage; tells the previous stage
//		that we are ready to receive the data

//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}), // analogous to output Q (of a FF)
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop}) // analogous to input D (of a FF)
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}), // analogous to output Q (of a FF)
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop}) // analogous to input D (of a FF)
);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col; // boundary box colour
reg	[23:0] r_bb_col, g_bb_col, b_bb_col, v_bb_col, y_bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		bb_col <= BB_COL_DEFAULT;
		r_bb_col <= R_BB_COL_DEFAULT;
		g_bb_col <= G_BB_COL_DEFAULT;
		b_bb_col <= B_BB_COL_DEFAULT;
		v_bb_col <= V_BB_COL_DEFAULT;
		y_bb_col <= Y_BB_COL_DEFAULT;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule

