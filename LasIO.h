#include <string.h>
#include <vector>

enum las_File
{
	las_x,
	las_y,
	las_z,
	las_r,
	las_g,
	las_b,
	las_Intensity,
	las_Return_Number,
	las_Number_of_Returns,
	las_Classification,
	las_Scan_Direction_Flag,
	las_Edge_of_Flight_Line,
	las_Scan_Angle_Rank,
	las_Point_Source_ID,
	las_GPS_Time
};

const char las_File_Name[][28] = { "X",
									"Y",
									"Z",
									"Intensity",
									"Return Number",
									"Number of Returns",
									"Scan Direction",
									"Flightline Edge",
									"Classification",
									"Scan Angle Rank",
									"User Data",
									"Point Source ID",
									"Red",
									"Green",
									"Blue",
									"Time",
									"[Classif] Value",
									"[Classif] Synthetic flag",
									"[Classif] Key-point flag",
									"[Classif] Withheld flag",
};