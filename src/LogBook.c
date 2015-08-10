/*
 ============================================================================
 Name        : LogBook.c
 Author      : Wil Selby
 Version     : v0.1
 Copyright   : Your copyright notice
 Description : Processes ArduPilot telemetry log .csv files and creates a flight logbook
 ============================================================================
 */

//Includes
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Defines
#define LOCAL_PI 3.1415926535897932385
#define MAX_LOGS 10

//Variables
char *input_array[MAX_LOGS];
int num_input_files = 0;
int i;
int j;
int k;
FILE *fileOut;
FILE *stream;
char line[1024];
char* line_in;
char* tmp;
char* dtg;
char* t_start;
char* t_end;
char * time_out;
int num_to;
double dist;
double time_sec = 0;
int log_processed = 0;
char* date;
int datetime_flag = 0;
int type_flag = 0;
int autopilot_flag = 0;
char* MAV_type;
char* MAV_autopilot;
int counter = 0;

struct GPS_dist{

	double lat_old;
	double lon_old;

	double lat_new;
	double lon_new;

	double total;
	double dist_calc;
};

struct GPS_dist distance;

struct MAV_totals{

	double mav_dist_total[19];  //Distance
	double mav_time_total[19];  //Time
	int mav_to_total[19];	//Flights
	int file_pos;
};

struct MAV_totals mav_totals;


//Heartbeat Message (For future reference)
typedef struct __mavlink_heartbeat_t
{
	uint32_t custom_mode; ///< A bitfield for use for autopilot-specific flags.
	uint8_t type; ///< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
	uint8_t autopilot; ///< Autopilot type / class. defined in MAV_AUTOPILOT ENUM
	uint8_t base_mode; ///< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
	uint8_t system_status; ///< System status flag, see MAV_STATE ENUM
	uint8_t mavlink_version; ///< MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
} mavlink_heartbeat_t;

//uint8_t type
typedef enum MAV_TYPE
{
	MAV_TYPE_GENERIC=0, /* Generic micro air vehicle. | */
	MAV_TYPE_FIXED_WING=1, /* Fixed wing aircraft. | */
	MAV_TYPE_QUADROTOR=2, /* Quadrotor | */
	MAV_TYPE_COAXIAL=3, /* Coaxial helicopter | */
	MAV_TYPE_HELICOPTER=4, /* Normal helicopter with tail rotor. | */
	MAV_TYPE_ANTENNA_TRACKER=5, /* Ground installation | */
	MAV_TYPE_GCS=6, /* Operator control unit / ground control station | */
	MAV_TYPE_AIRSHIP=7, /* Airship, controlled | */
	MAV_TYPE_FREE_BALLOON=8, /* Free balloon, uncontrolled | */
	MAV_TYPE_ROCKET=9, /* Rocket | */
	MAV_TYPE_GROUND_ROVER=10, /* Ground rover | */
	MAV_TYPE_SURFACE_BOAT=11, /* Surface vessel, boat, ship | */
	MAV_TYPE_SUBMARINE=12, /* Submarine | */
	MAV_TYPE_HEXAROTOR=13, /* Hexarotor | */
	MAV_TYPE_OCTOROTOR=14, /* Octorotor | */
	MAV_TYPE_TRICOPTER=15, /* Octorotor | */
	MAV_TYPE_FLAPPING_WING=16, /* Flapping wing | */
	MAV_TYPE_KITE=17, /* Flapping wing | */
	MAV_TYPE_ONBOARD_CONTROLLER=18, /* Onboard companion controller | */
	MAV_TYPE_ENUM_END=19, /*  | */
} MAV_TYPE;

//uint8_t autopilot
typedef enum MAV_AUTOPILOT
{
	MAV_AUTOPILOT_GENERIC=0, /* Generic autopilot, full support for everything | */
	MAV_AUTOPILOT_PIXHAWK=1, /* PIXHAWK autopilot, http://pixhawk.ethz.ch | */
	MAV_AUTOPILOT_SLUGS=2, /* SLUGS autopilot, http://slugsuav.soe.ucsc.edu | */
	MAV_AUTOPILOT_ARDUPILOTMEGA=3, /* ArduPilotMega / ArduCopter, http://diydrones.com | */
	MAV_AUTOPILOT_OPENPILOT=4, /* OpenPilot, http://openpilot.org | */
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY=5, /* Generic autopilot only supporting simple waypoints | */
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6, /* Generic autopilot supporting waypoints and other simple navigation commands | */
	MAV_AUTOPILOT_GENERIC_MISSION_FULL=7, /* Generic autopilot supporting the full mission command set | */
	MAV_AUTOPILOT_INVALID=8, /* No valid autopilot, e.g. a GCS or other MAVLink component | */
	MAV_AUTOPILOT_PPZ=9, /* PPZ UAV - http://nongnu.org/paparazzi | */
	MAV_AUTOPILOT_UDB=10, /* UAV Dev Board | */
	MAV_AUTOPILOT_FP=11, /* FlexiPilot | */
	MAV_AUTOPILOT_PX4=12, /* PX4 Autopilot - http://pixhawk.ethz.ch/px4/ | */
	MAV_AUTOPILOT_SMACCMPILOT=13, /* SMACCMPilot - http://smaccmpilot.org | */
	MAV_AUTOPILOT_AUTOQUAD=14, /* AutoQuad -- http://autoquad.org | */
	MAV_AUTOPILOT_ARMAZILA=15, /* Armazila -- http://armazila.com | */
	MAV_AUTOPILOT_AEROB=16, /* Aerob -- http://aerob.ru | */
	MAV_AUTOPILOT_ENUM_END=17, /*  | */
} MAV_AUTOPILOT;

//uint8_t base_mode
typedef enum MAV_MODE_FLAG
{
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use. | */
	MAV_MODE_FLAG_TEST_ENABLED=2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	MAV_MODE_FLAG_AUTO_ENABLED=4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	MAV_MODE_FLAG_GUIDED_ENABLED=8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	MAV_MODE_FLAG_STABILIZE_ENABLED=16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	MAV_MODE_FLAG_HIL_ENABLED=32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED=64, /* 0b01000000 remote control input is enabled. | */
	MAV_MODE_FLAG_SAFETY_ARMED=128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
	MAV_MODE_FLAG_ENUM_END=129, /*  | */
} MAV_MODE_FLAG;

//uint8_t system_status
typedef enum MAV_STATE
{
	MAV_STATE_UNINIT=0, /* Uninitialized system, state is unknown. | */
	MAV_STATE_BOOT=1, /* System is booting up. | */
	MAV_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
	MAV_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
	MAV_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
	MAV_STATE_CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
	MAV_STATE_EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
	MAV_STATE_POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
	MAV_STATE_ENUM_END=8, /*  | */
} MAV_STATE;


//Functions
char* mavType(int input)
{
	switch (input)
	{
	case MAV_TYPE_GENERIC: return "Generic sUAS";
	case MAV_TYPE_FIXED_WING: return "Fixed Wing";
	case MAV_TYPE_QUADROTOR: return "Quadrotor";
	case MAV_TYPE_COAXIAL: return "Coaxial Helicopter";
	case MAV_TYPE_HELICOPTER: return "Helicopter";
	case MAV_TYPE_ANTENNA_TRACKER: return "Antenna Tracker";
	case MAV_TYPE_GCS: return "GCS";
	case MAV_TYPE_AIRSHIP: return "Airship";
	case MAV_TYPE_FREE_BALLOON: return "Free Balloon";
	case MAV_TYPE_ROCKET: return "Rocket";
	case MAV_TYPE_GROUND_ROVER: return "Ground Rover";
	case MAV_TYPE_SURFACE_BOAT: return "Surface Boat";
	case MAV_TYPE_SUBMARINE: return "Submarine";
	case MAV_TYPE_HEXAROTOR: return "Hexarotor";
	case MAV_TYPE_OCTOROTOR: return "Octotor";
	case MAV_TYPE_TRICOPTER: return "Tricopter";
	case MAV_TYPE_FLAPPING_WING: return "Flapping Wing";
	case MAV_TYPE_KITE: return "Kite";
	case MAV_TYPE_ONBOARD_CONTROLLER: return "Onboard Companion Controller";
	default : return "Unknown";
	}
}

char* apType(int input)
{
	switch (input)
	{
	case MAV_AUTOPILOT_GENERIC: return "Generic Autopilot";
	case MAV_AUTOPILOT_PIXHAWK: return "PIXHAWK";
	case MAV_AUTOPILOT_SLUGS: return "SLUGS";
	case MAV_AUTOPILOT_ARDUPILOTMEGA: return "ArduPilotMega";
	case MAV_AUTOPILOT_OPENPILOT: return "OpenPilot";
	case MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY: return "Generic (waypoints)";
	case MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY: return "Generic (waypoints+)";
	case MAV_AUTOPILOT_GENERIC_MISSION_FULL: return "Generic (full mission)";
	case MAV_AUTOPILOT_INVALID: return "Invalid";
	case MAV_AUTOPILOT_PPZ: return "PPZ";
	case MAV_AUTOPILOT_UDB: return "UAV Dev Board";
	case MAV_AUTOPILOT_FP: return "FlexiPilot";
	case MAV_AUTOPILOT_PX4: return "PX4";
	case MAV_AUTOPILOT_SMACCMPILOT: return "SMACCMPILOT";
	case MAV_AUTOPILOT_AUTOQUAD: return "AutoQuad";
	case MAV_AUTOPILOT_ARMAZILA: return "Armazila";
	case MAV_AUTOPILOT_AEROB: return "Aerob";
	default : return "Unknown";
	}
}

char* getfield(char* line, int num)
{
	char* tok;
	int x;
	for ( x = 1; x <= num; x++ )
	{
		if ( x == 1){

			tok = strtok(line, ",");
			//printf( " %s\n", tok);
		}
		else{

			tok = strtok(NULL, ",");
			//printf( " %s\n", tok);
		}

	}

	return tok;
}

char* get_datetime(char* line_input){

	char* date_time;

	date_time = getfield(line_input,1);  //Get first row, first column which is the date
	//dtg = strtok(date_time, "T");  //Separate date from time
	//printf( "Date: %s\n", dtg );

	return date_time;
}

char* get_autopilot_type(char* line_input){

	char* autopilotType;
	char* msg_name;
	char* MAV_ap_num;

	msg_name = getfield(line_input,13);

	//printf( "name pulled %s\n", msg_name );

	if (strcmp(msg_name,"autopilot") == 0){
		//printf("Entered strings are equal.\n");
		//printf( "is %s\n", msg_name );
		autopilot_flag = 1;
	}
	else{
		//printf("Entered strings are not equal.\n");
		//printf( "is not %s\n", msg_name );
	}

	if(autopilot_flag){
		MAV_ap_num = getfield(NULL,1);
		//printf( "is num  %s\n", MAV_type_num );
		int num = atoi( MAV_ap_num );
		autopilotType = apType(num);
		printf( "Autopilot: %s\n", autopilotType );
	}

	return autopilotType;
}

double ToRadians(double degrees)
{
	double radians = degrees * LOCAL_PI / 180;
	return radians;
}

double DirectDistance(double lat1, double lng1, double lat2, double lng2)
{
	double earthRadius = 3958.75;
	double dLat = ToRadians(lat2-lat1);
	double dLng = ToRadians(lng2-lng1);
	double a = sin(dLat/2) * sin(dLat/2) +
			cos(ToRadians(lat1)) * cos(ToRadians(lat2)) *
			sin(dLng/2) * sin(dLng/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	double dist = earthRadius * c;
	double meterConversion = 1609.34;
	return dist * meterConversion;
}

struct GPS_dist calc_distance(char* line_input, struct GPS_dist dist_input){

	char* lat;
	char* lon;
	char* msg_name;
	char* tmp;

	tmp = strdup(line_input);
	msg_name = getfield(tmp,8);

	//printf( "name pulled %s\n", msg_name );

	if (strcmp(msg_name,"mavlink_global_position_int_t") == 0){
		//printf("Entered strings are equal.\n");
		//printf( "Message: %s\n", msg_name );

		tmp = strdup(line_input);
		lat = getfield(tmp,12);
		dist_input.lat_new = atof(lat);
		dist_input.lat_new = dist_input.lat_new / 1E7;

		tmp = strdup(line_input);
		lon = getfield(tmp,14);
		dist_input.lon_new = atof(lon);
		dist_input.lon_new = dist_input.lon_new / 1E7;

		//Latitude/Longitude, expressed as degrees * 1E7
		//printf( "string lat: %s lon: %s\n", lat, lon );
		//printf( "old lat: %f lon: %f \n", dist_input.lat_old, dist_input.lon_old );
		//printf( "new lat: %f lon: %f \n\n", dist_input.lat_new, dist_input.lon_new );

		//If first time through
		if(dist_input.lat_old == 0  && dist_input.lon_old == 0){
			dist_input.lat_old = dist_input.lat_new;
			dist_input.lon_old = dist_input.lon_new;
		}

		//Calculate distance using Haversine formula (meters)
		dist_input.dist_calc = DirectDistance(dist_input.lat_old, dist_input.lon_old, dist_input.lat_new, dist_input.lon_new);

		//Update variables
		dist_input.lat_old = dist_input.lat_new;
		dist_input.lon_old = dist_input.lon_new;

	}
	else{
		//printf("Entered strings are not equal.\n");
		//printf( "is not %s\n", msg_name );

		dist_input.dist_calc = 0;
	}

	return dist_input;
}

char* sec_to_hms(double time){

	char out[100];
	char* string_out;
	int hf, mf, sf;

	hf = (int)(time/3600);
	mf = (int)((time-(3600*hf))/60);
	sf = (int)(time-3600*hf-60*mf);

	//printf("%f %d %d %d \n",time, hf, mf, sf);

	sprintf(out,"%d:%d:%d", hf, mf, sf);

	string_out = &out[0];
	//printf("%s\n",string_out);

	return string_out;
}

double calc_time(char* start, char* finish){

	char* tmp;
	char* tok;
	double h1, h2, m1, m2, s1, s2;
	double seconds1, seconds2;
	double diff;

	//Convert Start Time to seconds
	tmp = strdup(start);
	tok = strtok(tmp, ":");
	h1 = atof(tok);
	tok = strtok(NULL, ":");
	m1 = atof(tok);
	tok = strtok(NULL, ":");
	s1 = atof(tok);

	//printf("%f %f %f \n", h1, m1, s1);

	//Convert Start Time to seconds
	tmp = strdup(finish);
	tok = strtok(tmp, ":");
	h2 = atof(tok);
	tok = strtok(NULL, ":");
	m2 = atof(tok);
	tok = strtok(NULL, ":");
	s2 = atof(tok);

	//printf("%f %f %f \n", h2, m2, s2);

	seconds1 = 3600*h1 + 60*m1 +s1;
	seconds2 = 3600*h2 + 60*m2 +s2;

	diff = seconds2-seconds1;

	return diff;
}

char* get_MAV_type(char* line_input){

	char* MAVtype;
	char* msg_name;
	char* MAV_type_num;

	msg_name = getfield(line_input,8);

	//printf( "name pulled %s\n", msg_name );


	if (strcmp(msg_name,"mavlink_heartbeat_t") == 0){
		//printf("Entered strings are equal.\n");
		//printf( "is %s\n", msg_name );
		type_flag = 1;
	}
	else{
		//printf("Entered strings are not equal.\n");
		//printf( "is not %s\n", msg_name );
	}

	if(type_flag){
		MAV_type_num = getfield(NULL,4);
		//printf( "is num  %s\n", MAV_type_num );
		int num = atoi( MAV_type_num );
		MAVtype = mavType(num);
		printf( "MAV Type: %s\n", MAVtype );
	}

	return MAVtype;
}

struct MAV_totals calc_MAV_dist_total(struct MAV_totals mav_total_input, char * mavtype, int to, double dist_input, double time_total_sec )
{

	if (strcmp(mavtype,"Generic sUAS") == 0) //0
	{
		mav_total_input.mav_dist_total[0] = mav_total_input.mav_dist_total[0] + dist_input;
		mav_total_input.mav_time_total[0] = mav_total_input.mav_time_total[0] + time_total_sec;
		mav_total_input.mav_to_total[0] = mav_total_input.mav_to_total[0] + to;
	}
	if (strcmp(mavtype,"Fixed Wing") == 0) //1
	{
		mav_total_input.mav_dist_total[1] = mav_total_input.mav_dist_total[1] + dist_input;
		mav_total_input.mav_time_total[1] = mav_total_input.mav_time_total[1] + time_total_sec;
		mav_total_input.mav_to_total[1] = mav_total_input.mav_to_total[1] + to;
	}
	if (strcmp(mavtype,"Quadrotor") == 0) //2
	{
		mav_total_input.mav_dist_total[2] = mav_total_input.mav_dist_total[2] + dist_input;
		mav_total_input.mav_time_total[2] = mav_total_input.mav_time_total[2] + time_total_sec;
		mav_total_input.mav_to_total[2] = mav_total_input.mav_to_total[2] + to;
	}
	if (strcmp(mavtype,"Coaxial Helicopter") == 0) //3
	{
		mav_total_input.mav_dist_total[3] = mav_total_input.mav_dist_total[3] + dist_input;
		mav_total_input.mav_time_total[3] = mav_total_input.mav_time_total[3] + time_total_sec;
		mav_total_input.mav_to_total[3] = mav_total_input.mav_to_total[3] + to;
	}
	if (strcmp(mavtype,"Helicopter") == 0) //4
	{
		mav_total_input.mav_dist_total[4] = mav_total_input.mav_dist_total[4] + dist_input;
		mav_total_input.mav_time_total[4] = mav_total_input.mav_time_total[4] + time_total_sec;
		mav_total_input.mav_to_total[4] = mav_total_input.mav_to_total[4] + to;
	}
	if (strcmp(mavtype,"Airship") == 0) //7
	{
		mav_total_input.mav_dist_total[7] = mav_total_input.mav_dist_total[7] + dist_input;
		mav_total_input.mav_time_total[7] = mav_total_input.mav_time_total[7] + time_total_sec;
		mav_total_input.mav_to_total[7] = mav_total_input.mav_to_total[7] + to;
	}
	if (strcmp(mavtype,"Surface Boat") == 0) //11
	{
		mav_total_input.mav_dist_total[11] = mav_total_input.mav_dist_total[11] + dist_input;
		mav_total_input.mav_time_total[11] = mav_total_input.mav_time_total[11] + time_total_sec;
		mav_total_input.mav_to_total[11] = mav_total_input.mav_to_total[11] + to;
	}
	if (strcmp(mavtype,"Hexarotor") == 0) //13
	{
		mav_total_input.mav_dist_total[13] = mav_total_input.mav_dist_total[13] + dist_input;
		mav_total_input.mav_time_total[13] = mav_total_input.mav_time_total[13] + time_total_sec;
		mav_total_input.mav_to_total[13] = mav_total_input.mav_to_total[13] + to;
	}
	if (strcmp(mavtype,"Octotor") == 0) //14
	{
		mav_total_input.mav_dist_total[14] = mav_total_input.mav_dist_total[14] + dist_input;
		mav_total_input.mav_time_total[14] = mav_total_input.mav_time_total[14] + time_total_sec;
		mav_total_input.mav_to_total[14] = mav_total_input.mav_to_total[14] + to;
	}
	if (strcmp(mavtype,"Tricopter") == 0) //15
	{
		mav_total_input.mav_dist_total[15] = mav_total_input.mav_dist_total[15] + dist_input;
		mav_total_input.mav_time_total[15] = mav_total_input.mav_time_total[15] + time_total_sec;
		mav_total_input.mav_to_total[15] = mav_total_input.mav_to_total[15] + to;
	}
	if (strcmp(mavtype,"Flapping Wing") == 0) //16
	{
		mav_total_input.mav_dist_total[16] = mav_total_input.mav_dist_total[16] + dist_input;
		mav_total_input.mav_time_total[16] = mav_total_input.mav_time_total[16] + time_total_sec;
		mav_total_input.mav_to_total[16] = mav_total_input.mav_to_total[16] + to;
	}

	return mav_total_input;
}

void print_totals( FILE * log, struct MAV_totals mav_total_input, int write){

	char* time_hms;
	char* MAV_type_print;
	int l;
	int first_line_flag = 0;

	if(write){
		fprintf(log,"\n\n");	//Make space from the row entries
	}

	//Iterate through all MAV Types
	for(l=0; l<=(MAV_TYPE_ENUM_END-1); l++){

		//If there are non-zero values to print
		if((mav_totals.mav_dist_total[l] != 0) && (mav_totals.mav_time_total[l] != 0)){

			time_hms = sec_to_hms(mav_totals.mav_time_total[l]);	//Convert time from seconds to H:M:S string
			MAV_type_print = mavType(l);
			printf("%s: Flights- %d Distance- %.2f (km) Time- %s (H:M:S) \n", MAV_type_print, mav_totals.mav_to_total[l] , mav_totals.mav_dist_total[l]/1000, time_hms);
			if(write && (first_line_flag == 0) ){
				fprintf(log, "Totals:,%s, , ,%d,%s,%.2f \n", MAV_type_print,mav_totals.mav_to_total[l], time_hms, mav_totals.mav_dist_total[l]/1000);
			}
			if(write && (first_line_flag == 1) ){
				fprintf(log, " ,%s, , , %d,%s,%.2f \n", MAV_type_print, mav_totals.mav_to_total[l], time_hms, mav_totals.mav_dist_total[l]/1000);
			}
			first_line_flag = 1;	//So we don't writhe the word "Total" on every line
		}
	}
}

void print_entry(FILE * log, char * date, char * mavtype, int to, char * aptype, double distance, char * time){

	fprintf(log,"%s, %s, %s, LOS/FPV, %d , %s, %.2f\n", date, mavtype, aptype, to, time, distance/1000);
}

struct MAV_totals extract_totals(FILE * old_log, struct MAV_totals mav_total_input){

	char* tmp2;
	char* totals;
	char buffer[1024];
	char* MAV_type_old_tot;
	char* old_dist_tot;
	char* old_time_tot;
	double old_dist;
	double old_time;
	char* tmp3;
	char* tok;
	double h1, m1, s1;
	int totals_flag = 0;
	char* log_entry;
	int num_logs = 0;
	int n;
	int count;
	int num_to = 0;
	char* to;

	fseek(old_log, 0, SEEK_SET);	//If using a+, rewind to beginning of the file

	while(fgets(buffer,sizeof(buffer),old_log)!=NULL){

		tmp2 = strdup(buffer);

		//Count the number of old log entries so we can move the write pointer
		log_entry = getfield(tmp2,1);
		tmp2 = strdup(buffer);
		tok = strtok(tmp2, "-"); //If the first entry is deliminated by '-'
		n = atoi(tok);
		count = 0;  //reset each time
		while(n!=0)
		  {
		      n/=10;
		      count=count+1;
		  }

		if (count == 4){	//And is a 4 digit year
			num_logs++;	//It's a valid old log entry
			mav_total_input.file_pos=ftell(old_log);
			//printf("file POS %d \n",mav_total_input.file_pos);
		}

		//Look for subsequent MAV type totals
		if(totals_flag){

			//Get MAV Type
			MAV_type_old_tot = getfield(tmp2,2);
			tmp2 = strdup(buffer);

			//Get num flights
			to = getfield(tmp2,5);
			num_to = atoi(to);
			tmp2 = strdup(buffer);

			//Get time
			old_time_tot = getfield(tmp2,6);
			tmp2 = strdup(buffer);

			//Convert time to seconds
			tmp3 = strdup(old_time_tot);
			tok = strtok(tmp3, ":");
			h1 = atof(tok);
			tok = strtok(NULL, ":");
			m1 = atof(tok);
			tok = strtok(NULL, ":");
			s1 = atof(tok);

			old_time = 3600*h1 + 60*m1 +s1;

			//Get distance
			old_dist_tot = getfield(tmp2,7);
			tmp2 = strdup(buffer);
			old_dist = 1000*atof( old_dist_tot );	//Convert back to m

			mav_total_input = calc_MAV_dist_total(mav_total_input, MAV_type_old_tot, num_to, old_dist, old_time);
		}

		//Find Totals Line
		totals = getfield(tmp2,1);
		tmp2 = strdup(buffer);
		if (strcmp(totals,"Totals:") == 0){

			//Get MAV Type
			MAV_type_old_tot = getfield(tmp2,2);
			tmp2 = strdup(buffer);

			//Get num flights
			to = getfield(tmp2,5);
			num_to = atoi(to);
			tmp2 = strdup(buffer);

			//Get time
			old_time_tot = getfield(tmp2,6);
			tmp2 = strdup(buffer);

			//Convert time to seconds
			tmp3 = strdup(old_time_tot);
			tok = strtok(tmp3, ":");
			h1 = atof(tok);
			tok = strtok(NULL, ":");
			m1 = atof(tok);
			tok = strtok(NULL, ":");
			s1 = atof(tok);

			old_time = 3600*h1 + 60*m1 +s1;

			//Get distance
			old_dist_tot = getfield(tmp2,7);
			tmp2 = strdup(buffer);
			old_dist = 1000*atof( old_dist_tot );	//Convert back to m

			mav_total_input = calc_MAV_dist_total(mav_total_input, MAV_type_old_tot, num_to, old_dist, old_time);

			totals_flag = 1;
		}
	}

	printf("\nRead %d log(s) from previous file\n",num_logs);


	//Remove old total values be moving the file pointer
	fseek(old_log, mav_total_input.file_pos, SEEK_SET);	//If using a+, rewind to beginning of the file

	return mav_total_input;
}

//MAIN
int main(int argc, char *argv[]) {

	// Iterate through arguments, print argument names
	for (i=1; i< argc; i++) {

		//Print Help (flag -h or --help)
		if ((strcmp(argv[i],"-h") == 0) || (strcmp(argv[i],"--help") == 0)){
			printf("\n\nDescription: This program processes ArduPilot telemetry log .csv files and creates a flight logbook csv file."
					"The Mission Planner software is needed to convert the .tlog files into .csv files for input into this program \n"
					"\n\nUsage: Logbook [options] [arg_name...]\n"
					"\n"
					"-h, --help \t\t Displays this usage syntax and exits\n"
					"-i [input_files] \t The input .csv file name(s)/path(s) to be processed. Maximum is 10 \n"
					"-n [new_output_file] \t Name of the .csv Logbook file to be created \n"
					"-a [old_output_file] \t Name of the exisisting .csv Logbook file to be updated\n");

			return 0;
		}

		//Process input .csv files (flag -i)
		if (strcmp(argv[i],"-i") == 0){

			//Cycle through list of input files
			for (j=i+1; j< argc; j++) {

				//If we hit the next flag, break out of the loop
				if ((strcmp(argv[j],"-n") == 0) || (strcmp(argv[j],"-a") == 0)){
					break;
				}
				else{

					//Don't overload the array of input log files
					if( (j-2) < MAX_LOGS){

						input_array[j-2] = argv[j];
						//printf("*** file name %d %s \n",j-1, input_array[j-2]);
						num_input_files++;
					}
					else{
						printf("Too many logs, max is %d \n", MAX_LOGS);
					}
				}
			}
		}

		//Open a new output file (flag -n)
		if (strcmp(argv[i],"-n") == 0){

			//Open output  file
			fileOut = fopen(argv[i+1], "w");

			if(fileOut == NULL)
			{
				perror("Error creating output file");
				return(-1);
			}
			else{
				printf("\nCreated Logbook: %s \n", argv[i+1]);
			}

			//Print Column Header
			fprintf(fileOut,"Date, MAV Type, Autopilot Type, LOS/FPV, # TO, Time (H:M:S), Distance (km)\n");

		}

		//Append to an existing output file (flag -a)
		if (strcmp(argv[i],"-a") == 0){

			//Open output  file
			fileOut = fopen(argv[i+1], "r+");	//Read and append (a+)

			if(fileOut == NULL)
			{
				perror("Error opening existing output file");
				return(-1);
			}
			else{
				printf("Opened Existing Logbook: %s \n", argv[i+1]);
			}

			//Extract old total values
			mav_totals = extract_totals(fileOut, mav_totals);

			//Print totals read to console but not to output file
			printf("\nExisiting Totals by MAV Type:\n");
			print_totals( fileOut, mav_totals, 0);

		}
	}


	//Iterate through input logs
	for(k = 0; k<num_input_files; k++){

		//Open log file
		stream = fopen(input_array[k], "r");

		if(fopen(input_array[k], "r") == NULL)
		{
			perror("Error opening log file");
			printf("Error opening file %s \n", input_array[k]);
			return(-1);
		}
		else{
			printf("\nOpened Flight Log %s", input_array[k]);
		}

		//Read Log File
		while(fgets(line,sizeof(line),stream)!=NULL){

			//puts(line);
			tmp = strdup(line);

			//Get Date and Start Time
			if(datetime_flag == 0){
				dtg = get_datetime(tmp);
				date = strtok(dtg, "T");  //Separate date from time
				printf( "\nDate: %s\n", date );
				t_start = strtok(NULL, "T");
				//printf( "Start Time: %s\n", t_start );
				datetime_flag = 1;
			}
			tmp = strdup(line);


			//Get MAV Type
			if(type_flag == 0){
				MAV_type = get_MAV_type(tmp);
			}
			tmp = strdup(line);


			//Get Autopilot type
			if(autopilot_flag == 0){
				MAV_autopilot = get_autopilot_type(tmp);
			}
			tmp = strdup(line);

			//Calculate distance traveled
			distance = calc_distance(tmp, distance);
			distance.total = distance.dist_calc + distance.total;

			free(tmp);
		}

		printf("Total Distance: %f (m) \n", distance.total);

		//Get End Time
		dtg = get_datetime(line);
		date = strtok(dtg, "T");  //Separate date from time
		t_end = strtok(NULL, "T");
		//printf( "End Time: %s\n", t_end );

		time_sec = calc_time(t_start, t_end);
		//sec_to_hms(time_sec, time_out);
		time_out = sec_to_hms(time_sec);

		printf( "Elapsed Time: %s (H:M:S) \n", time_out);

		num_to = 1; //Hardcoded for now that 1 log = 1 take off

		//Update Platform totals
		mav_totals = calc_MAV_dist_total(mav_totals, MAV_type, num_to, distance.total, time_sec);

		//Print Row
		print_entry(fileOut, date, MAV_type, num_to, MAV_autopilot,distance.total, time_out);

		//Reset flags
		datetime_flag = 0;
		type_flag = 0;
		autopilot_flag = 0;

		//Reset GPS struct variables
		memset(&distance, 0, sizeof(distance));

	}

	//Print Platform Totals to console and output file
	printf("\nFinal Totals by MAV Type:\n");
	print_totals( fileOut, mav_totals, 1);

	//Finished
	printf("\nProcessing Complete \n");

	//Close File Pointers
	fclose(stream);
	fclose(fileOut);

	return(0);
}
