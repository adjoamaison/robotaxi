/**
*Agatha Adjoa Maison and Michael Kwabena Annor
*Robotics final Project
*Fall Semester 2015
*A header file for the taxis
*/
#define X_SIZE 8
#define Y_SIZE 8
#define MAP_AREA X_SIZE * Y_SIZE
#define FREE 0
#define OBST 1
#define GOAL 2
#define VH_COST 10
#define DIAGONAL_COST 14
#define CONNECTIVITY 8

//an 8x8 world map
int world_map[][] = {	{0,0,0,0,0,0,0,1},
						{1,0,0,0,0,0,0,0},
						{0,0,1,0,0,0,0,0},
						{0,0,0,0,1,0,0,1},
						{0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,0},
						{1,1,0,0,0,0,0,0}
					};

//variables to store current location of the passenger
int start_x;
int start_y;
//location for two taxis
int taxi1base_x=0;
int taxi1base_y=7;
int taxi2base_x=7;
int taxi2base_y=0;

//variables to store final destination of the passenger
int goal_x;
int goal_y;

