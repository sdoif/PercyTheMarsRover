module rgb_to_hsv(
	red,
	green,
	blue,
	
	h,
	s,
	v,
	
	min
);

input [7:0] red, green, blue;
output [7:0] h, s, v, min;

wire [7:0] min;
wire [7:0] s_temp, v_temp;

// values cannot be stored as decimals otherwise quartus is unable to represent them.
//	Leave them all scaled

// not divided by 255
assign v_temp = (red >= green && red >= blue) ? red
				: (green >= red && green >= blue) ? green
				: (blue >= red && blue >= green) ? blue
				: 0;
	
assign v = v_temp*100/255; // scale to percent

// not divided by 255
assign min = red <= green && red <= blue ? red
				: green <= red && green <= blue ? green
				: blue <= red && blue <= green ? blue
				: 0;

// scaled up by 255
assign s_temp = v_temp != 0 ? 255*(v_temp - min)/v_temp
					: 0;
						
assign s = s_temp*100/255; // scaling to percentage
						
// v_temp and the colours are not scaled down
assign h = (red == green && green == blue) ? 0
			: v_temp == red ? //60*(green - blue)/(v_temp - min) // if V == R
					green >= blue ? 60*(green - blue)/(v_temp - min) // if g-b is positive, dont add 360
					: (360*(v_temp-min) + 60*(green - blue))/(v_temp - min) // if negative, add 360
			: v_temp == green ? (120*(v_temp - min) + 60*(blue - red))/(v_temp - min)
			: v_temp == blue ?	(240*(v_temp - min) + 60*(red - green))/(v_temp - min)
			: 0;
				
endmodule