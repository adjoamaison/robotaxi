/**
*Agatha Adjoa Maison and Michael Kwabena Annor
*Robotics final Project
*Fall Semester 2015
*A header file for the base station
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
//message id for taxis to get location and destination from passenger
#define TAXI1LOC 11 
#define TAXI1DEST 12
#define TAXI2LOC 21
#define TAXI2DEST 22

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

//locations of the two taxis (taxi1 and taxi2)
int taxi1base_x=0;
int taxi1base_y=7;
int taxi2base_x=7;
int taxi2base_y=0;
