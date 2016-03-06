/**
*Michael Annor and Agatha Maison
*A* Implementation for Robotics Final Project 
**/

#include "planning_map.h"

#define NORTH 0
#define NORTHEAST 1
#define EAST 2
#define SOUTHEAST 3
#define SOUTH 4
#define SOUTHWEST 5
#define WEST 6
#define NORTHWEST 7


//properties of a cell
typedef struct{
	int x;//ij
	int y;//jj
	int val;//map
	int f; int g; int h;//g(0),h(calc), f(0)
	int parentx;//-1
	int parenty;//-1
	bool onopen;//f
	bool onclosed;//f
} node;


//creating a 2 dimensional node map
node node_map[Y_SIZE][X_SIZE];
//node array to track neighbours
node current_neighbours[CONNECTIVITY];
//open list
node open[MAP_AREA];
int front;
int rear;

//function to set h cost for a given cell
int set_h_cost(int x, int y){
	int h;
	int dx;
	int dy;
	dx=goal_x-x;
	dy=goal_y-y;
	h = sqrt((dx*dx)+(dy*dy))* VH_COST;
	h = abs(h);
	return h;
}

//function to initialize the node/cell with all the properties a cell should have
void init_node(node *cell){
	cell->val=world_map[cell->y][cell->x];
	cell->f=0;
	cell->g=0;
	cell->h=set_h_cost(cell->x,cell->y);
	cell->parentx=-1;
	cell->parenty=-1;
	cell->onopen=false;
	cell->onclosed=false;
}

//function to initialize the whole node map
void init_map(){
	for (int i=0; i<Y_SIZE; i++){
		for (int j=0; j<X_SIZE; j++){
			node_map[i][j].x=j;
			node_map[i][j].y=i;
			init_node(&node_map[i][j]);
		}
	}
}

//intialize open list
void createopen(){
	front = rear = -1;
}

//add to open list by priority
void insert_by_priority(node newNode){
	if(rear >= MAP_AREA - 1){//list full
		return;
	}
	newNode.onopen=true;
	if((front==-1)&&(rear==-1)){//first entry
		front++;
		rear++;

		memcpy(&open[rear], &newNode, sizeof(open[rear]));
		node_map[newNode.y][newNode.x].onopen=true;
		return;
	}
	else{
		for(int i=0; i<=rear; i++){
			if(newNode.f <= open[i].f){
				for(int j=rear+1; j>i; j--){
					memcpy(&open[j], &open[j-1], sizeof(open[j]));
				}
				rear++;
				memcpy(&open[i], &newNode, sizeof(open[i]));
				node_map[newNode.y][newNode.x].onopen=true;
				return;
			}
		}
		rear++;
		memcpy(&open[rear], &newNode, sizeof(open[rear]));
		node_map[newNode.y][newNode.x].onopen=true;
	}
}

//function to get the node with the lowest f cost from the open list
void get_lowest_f_cost(int &lowx, int &lowy){
	//remove function from the priority queue
	int i;
	if ((front==-1) && (rear==-1)){
		//Queue is empty no elements to remove
		return;
	}
	lowx=open[0].x;
	lowy=open[0].y;
	node_map[lowy][lowx].onopen=false;//remove from open
	for (i = 0; i <= rear; i++){
		memcpy(open[i], &open[i + 1], sizeof(open[i]));
	}
	rear--;

	if (rear == -1){
		front = -1;
	}
}

//function to add a node to the close list (i.e. sets its onclosed attribute to true)
void add_to_closed(int x, int y){
	node_map[y][x].onclosed=true;
}

//function to check if a given node is the goal node
bool is_goal(int x, int y){
	if(((x==goal_x) && (y==goal_y))!=true){
		return false;
	}
	return true;
}

//function to set the g anf f cost of a given cell(cx,cy). This is relation to the parent (x,y) of the given cell
void set_cost(int cx, int cy, int x, int y){
	if ((cx != x) && (cy != y)){
		node_map[cy][cx].g = node_map[y][x].g+DIAGONAL_COST;
		node_map[cy][cx].f=node_map[cy][cx].g + node_map[cy][cx].h;
	}
	else {
		node_map[cy][cx].g = node_map[y][x].g+VH_COST;
		node_map[cy][cx].f=node_map[cy][cx].g + node_map[cy][cx].h;
	}
}

//function to get the neighbours of a given cell (north, north-east, east, south-east, south, south-west, west, north-west)
void get_neighbours(int x, int y){
	//north
	if (((y-1)<Y_SIZE) && ((y-1)>=0) && ((x)<X_SIZE) && ((x)>=0)){ //checks if node is out of bounds/ out of the world
		if(node_map[y-1][x].g==0){ //checks if g cost has not been set. if not, set it.
			set_cost(node_map[y-1][x].x,node_map[y-1][x].y,x,y);
		}
		memcpy(&current_neighbours[0], &node_map[y-1][x], sizeof(current_neighbours[0]));//copy cell from neighbours array into node map
	}
	else {
		current_neighbours[0].val=1; //else neighbour is an obstacle so set its value attribute to 1
	}

	if (((y)<Y_SIZE) && ((y)>=0) && ((x+1)<X_SIZE) && ((x+1)>=0)){
		if(node_map[y][x+1].g==0){
			set_cost(node_map[y][x+1].x,node_map[y][x+1].y,x,y);
		}
		memcpy(&current_neighbours[2], &node_map[y][x+1], sizeof(current_neighbours[2]));//east
	}
	else {
		current_neighbours[2].val=1;
	}
	if (((y+1)<Y_SIZE) && ((y+1)>=0) && ((x)<X_SIZE) && ((x)>=0)){
		if(node_map[y+1][x].g==0){
			set_cost(node_map[y+1][x].x,node_map[y+1][x].y,x,y);
		}
		memcpy(&current_neighbours[4], &node_map[y+1][x], sizeof(current_neighbours[4]));//south
	}
	else {
		current_neighbours[4].val=1;
	}
	if (((y)<Y_SIZE) && ((y)>=0) && ((x-1)<X_SIZE) && ((x-1)>=0)){
		if(node_map[y][x-1].g==0){
			set_cost(node_map[y][x-1].x,node_map[y][x-1].y,x,y);
		}
		memcpy(&current_neighbours[6], &node_map[y][x-1], sizeof(current_neighbours[6]));//west
	}
	else {
		current_neighbours[6].val=1;
	}
	if (((y-1)<Y_SIZE) && ((y-1)>=0) && ((x+1)<X_SIZE) && ((x+1)>=0)&&(current_neighbours[0].val==FREE)&&(current_neighbours[2].val==FREE)){
		if(node_map[y-1][x+1].g==0){
			set_cost(node_map[y-1][x+1].x,node_map[y-1][x+1].y,x,y);
		}
		memcpy(&current_neighbours[1], &node_map[y-1][x+1], sizeof(current_neighbours[1]));//north-east
	}
	else {
		current_neighbours[1].val=1;
	}
	if (((y+1)<Y_SIZE) && ((y+1)>=0) && ((x+1)<X_SIZE) && ((x+1)>=0)&&(current_neighbours[2].val==FREE)&&(current_neighbours[4].val==FREE)){
		if(node_map[y+1][x+1].g==0){
			set_cost(node_map[y+1][x+1].x,node_map[y+1][x+1].y,x,y);
		}
		memcpy(&current_neighbours[3], &node_map[y+1][x+1], sizeof(current_neighbours[3]));//south-east
	}
	else {
		current_neighbours[3].val=1;
	}
	if (((y-1)<Y_SIZE) && ((y-1)>=0) && ((x-1)<X_SIZE) && ((x-1)>=0)&&(current_neighbours[0].val==FREE)&&(current_neighbours[6].val==FREE)){
		if(node_map[y-1][x-1].g==0){
			set_cost(node_map[y-1][x-1].x,node_map[y-1][x-1].y,x,y);
		}
		memcpy(&current_neighbours[7], &node_map[y-1][x-1], sizeof(current_neighbours[7]));//north-west
	}
	else {
		current_neighbours[7].val=1;
	}
	if (((y+1)<Y_SIZE) && ((y+1)>=0) && ((x-1)<X_SIZE) && ((x-1)>=0)&&(current_neighbours[4].val==FREE)&&(current_neighbours[6].val==FREE)){
		if(node_map[y+1][x-1].g==0){
			set_cost(node_map[y+1][x-1].x,node_map[y+1][x-1].y,x,y);
		}
		memcpy(&current_neighbours[5], &node_map[y+1][x-1], sizeof(current_neighbours[5]));//south-west
	}
	else {
		current_neighbours[5].val=1;
	}
}

//function to get the g cost a given cell with respect to its parent
int get_g_cost(int cx, int cy, int x, int y){
	if ((cx != x) && (cy != y)){
		return (node_map[cy][cx].g + DIAGONAL_COST);
	}
	else {
		return (node_map[cy][cx].g + VH_COST);
	}
}

//an array to store the path from the start to the goal after computing using a*
int path[MAP_AREA];


//move to the next tile using encoder to measure distance
void moveforward(){
	nMotorEncoder[motorC] = 0;

	while (nMotorEncoder[motorC] < 600){
		motor[motorC]=30;
		motor[motorB]=30;
	}

	motor[motorC]=0;
	motor[motorB]=0;
	wait1Msec(100);
}

//stop when goal is found
void goalFound(){
	//while(true){
	motor[motorC]=0;
	motor[motorB]=0;
//	displayTextLine(1, "Pickup.");
	wait1Msec(100);
	//}
}

/* Routine to enable robot accurately turn 90 in place (both clockwise and
*counter clockwise).
*/

//turn 90 degrees left
void turnLeft(){
	nMotorEncoder[motorC] = 0;
	nMotorEncoder[motorB] = 0;
	while (nMotorEncoder[motorB] < 172){
		motor[motorC]=-35;
		motor[motorB]=40;
	}

	motor[motorB]=0;
	motor[motorC]=0;
}

//turn 45 degrees left
void diagonalLeft(){
	nMotorEncoder[motorC] = 0;
	nMotorEncoder[motorB] = 0;
	while (nMotorEncoder[motorB] < 84){
		motor[motorC]=-35;
		motor[motorB]=40;
	}

	motor[motorB]=0;
	motor[motorC]=0;
}

//turn 90 degrees right
void turnRight(){
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;
	while (nMotorEncoder[motorC] < 172){
		motor[motorB]=-35;
		motor[motorC]=40;
	}

	motor[motorB]=0;
	motor[motorC]=0;
}

//turn 45 degrees left
void diagonalRight(){
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;
	while (nMotorEncoder[motorC] < 84){
		motor[motorB]=-35;
		motor[motorC]=40;
	}

	motor[motorB]=0;
	motor[motorC]=0;
}

//move to the next tile using encoder to measure diagonal distance
void diagonalForward(){
	nMotorEncoder[motorC] = 0;

	while (nMotorEncoder[motorC] < 840){
		motor[motorC]=30;
		motor[motorB]=30;
	}

	motor[motorC]=0;
	motor[motorB]=0;
	wait1Msec(100);
}

//reorient to North
void reorient(int orient){
	if (orient == NORTH){
		return;
	}

	if (orient == NORTHEAST){
		diagonalLeft();
	}

	if (orient == EAST){
		turnLeft();
	}

	if (orient == SOUTHEAST){
		turnLeft();
		diagonalLeft();
	}

	if (orient == SOUTH){
		turnLeft();
		turnLeft();
	}

	if (orient == SOUTHWEST){
		turnRight();
		diagonalRight();
	}

	if (orient == WEST){
		turnRight();
	}

	if (orient == NORTHWEST){
		diagonalRight();
	}


}

//function to aid in navigation
void navigation(int count){
	int index = count;
	int curx = path[--index];
	int cury = path[--index];

	int orient = NORTH;

	for (int i=(count/2); i>0; i--){
		if(index==0){goalFound();break;}
		if (orient == NORTH) {
			if ((path[index-1] == curx) && (path[index-2]== cury-1)) { //north
				moveforward();
				orient = NORTH;

				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury-1)) { //north east
				diagonalRight();
				diagonalforward();
				orient = NORTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury)) { //east
				turnRight();
				moveforward();
				orient = EAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury+1)) { //south east
				turnRight();
				diagonalRight();
				diagonalforward();
				orient = SOUTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx) && (path[index-2]== cury+1)) { //south
				turnRight();
				turnRight();
				moveforward();
				orient = SOUTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury+1)) { //south west
				turnLeft();
				diagonalLeft();
				diagonalforward();
				orient = SOUTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury)) { //west
				turnLeft();
				moveforward();
				orient = WEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury-1)) { //north west
				diagonalLeft();
				diagonalforward();
				orient = NORTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
		}

		//NORTHEAST
		else if (orient == NORTHEAST) {
			if ((path[index-1] == curx) && (path[index-2]== cury-1)) { //north
				diagonalLeft();
				diagonalforward();
				orient = NORTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury-1)) { //north east
				moveforward();
				orient = NORTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury)) { //east
				diagonalRight();
				diagonalforward();
				orient = EAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury+1)) { //south east
				turnRight();
				moveforward();
				orient = SOUTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx) && (path[index-2]== cury+1)) { //south
				turnRight();
				diagonalRight();
				diagonalforward();
				orient = SOUTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury+1)) { //south west
				turnLeft();
				turnLeft();
				moveforward();
				orient = SOUTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury)) { //west
				diagonalLeft();
				turnLeft();
				diagonalforward();
				orient = WEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury-1)) { //north west
				turnLeft();
				moveforward();
				orient = NORTHWEST;
				curx = path[--index];
				cury = path[--index];
			}

		}
		//EAST
		else if (orient == EAST) {
			if ((path[index-1] == curx) && (path[index-2]== cury-1)) { //north
				turnLeft();
				moveforward();
				orient = NORTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] ==curx+1 ) && (path[index-2]== cury-1 )) { //north east
				diagonalLeft();
				diagonalforward();
				orient = NORTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury)) { //east
				moveforward();
				orient = EAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury+1)) { //south east
				diagonalRight();
				diagonalforward();
				orient = SOUTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx) && (path[index-2]== cury+1)) { //south
				turnRight();
				moveforward();
				orient = SOUTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury+1)) { //south west
				turnRight();
				diagonalRight();
				diagonalforward();
				orient = SOUTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury)) { //west
				turnLeft();
				turnLeft();
				moveforward();
				orient = WEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury-1)) { //north west
				turnLeft();
				diagonalLeft();
				diagonalforward();
				orient = NORTHWEST;
				curx = path[--index];
				cury = path[--index];
			}

		}
		//SOUTHEAST
		else if (orient == SOUTHEAST) {
			if ((path[index-1] == curx) && (path[index-2]== cury-1)) { //north
				diagonalLeft();
				turnLeft();
				diagonalforward();
				orient = NORTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury-1)) { //north east
				turnLeft();
				moveforward();
				orient = NORTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury)) { //east
				diagonalLeft();
				diagonalforward();
				orient = EAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury+1)) { //south east
				moveforward();
				orient = SOUTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx) && (path[index-2]== cury+1)) { //south
				diagonalRight();
				diagonalforward();
				orient = SOUTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury+1)) { //south west
				turnRight();
				moveforward();
				orient = SOUTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury)) { //west
				diagonalRight();
				turnRight();
				diagonalforward();
				orient = WEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury-1)) { //north west
				turnLeft();
				turnLeft();
				moveforward();
				orient = NORTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
		}

		//South
		else if (orient == SOUTH) {
			if ((path[index-1] == curx) && (path[index-2]== cury-1)) { //north
				turnLeft();
				turnLeft();
				moveforward();
				orient = NORTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury-1)) { //north east
				turnLeft();
				diagonalLeft();
				diagonalforward();
				orient = NORTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury)) { //east
				turnLeft();
				moveforward();
				orient = EAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury+1)) { //south east
				diagonalLeft();
				diagonalforward();
				orient = SOUTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx) && (path[index-2]== cury+1)) { //south
				moveforward();
				orient = SOUTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury+1)) { //south west
				diagonalRight();
				diagonalforward();
				orient = SOUTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury)) { //west
				turnRight();
				moveforward();
				orient = WEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury-1)) { //north west
				turnRight();
				diagonalRight();
				diagonalforward();
				orient = NORTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
		}

		//SOUTHWEST
		else if (orient == SOUTHWEST) {
			if ((path[index-1] == curx) && (path[index-2]== cury-1)) { //north
				turnRight();
				diagonalRight();
				diagonalforward();
				orient = NORTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury-1)) { //north east
				turnRight();
				turnRight();
				moveforward()
				orient = NORTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury)) { //east
				turnLeft();
				diagonalLeft();
				diagonalforward();
				orient = EAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury+1)) { //south east
				turnLeft();
				moveforward();
				orient = SOUTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx) && (path[index-2]== cury+1)) { //south
				diagonalLeft();
				diagonalforward();
				orient = SOUTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury+1)) { //south west
				moveforward();
				orient = SOUTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury)) { //west
				diagonalRight();
				diagonalforward();
				orient = WEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury-1)) { //north west
				turnRight();
				moveforward();
				orient = NORTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
		}

		//WEST
		else if (orient == WEST) {
			if ((path[index-1] == curx) && (path[index-2]== cury-1)) { //north
				turnRight();
				moveforward();
				orient = NORTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury-1)) { //north east
				turnRight();
				diagonalRight();
				diagonalforward();
				orient = NORTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury)) { //east
				turnRight();
				turnRight();
				moveforward();
				orient = EAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury+1)) { //south east
				turnLeft();
				diagonalLeft();
				diagonalforward();
				orient = SOUTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx) && (path[index-2]== cury+1)) { //south
				turnLeft();
				moveforward();
				orient = SOUTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury+1)) { //south west
				diagonalLeft();
				diagonalforward();
				orient = SOUTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury)) { //west
				moveforward();
				orient = WEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury-1)) { //north west
				diagonalRight();
				diagonalforward();
				orient = NORTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
		}

		//NORTHWEST
		else if (orient == NORTHWEST) {
			if ((path[index-1] == curx) && (path[index-2]== cury-1)) { //north
				diagonalRight();
				diagonalforward();
				orient = NORTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury-1)) { //north east
				turnRight();
				moveforward();
				orient = NORTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury)) { //east
				diagonalRight();
				turnRight();
				diagonalforward();
				orient = EAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx+1) && (path[index-2]== cury+1)) { //south east
				turnRight();
				turnRight();
				moveforward();
				orient = SOUTHEAST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx) && (path[index-2]== cury+1)) { //south
				diagonalLeft();
				turnLeft();
				diagonalforward();
				orient = SOUTH;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury+1)) { //south west
				turnLeft();
				moveforward();
				orient = SOUTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury)) { //west
				diagonalLeft();
				diagonalforward();
				orient = WEST;
				curx = path[--index];
				cury = path[--index];
			}
			else if ((path[index-1] == curx-1) && (path[index-2]== cury-1)) { //north west
				moveforward();
				orient = NORTHWEST;
				curx = path[--index];
				cury = path[--index];
			}
		}


		wait1Msec(100);
	}
	reorient(orient);
}

//function to get path after finding goal
void get_path(){
	int x, y,px,py;
	int count=0;
	px=goal_x;
	py=goal_y;
	displayTextLine(2, "g(%d, %d)s(%d, %d)",py,px,start_y, start_x);
	wait1Msec(100);
	while (!((px==start_x)&&(py==start_y))){
		x=px;y=py;
		px = node_map[y][x].parentx;
		py = node_map[y][x].parenty;
		//displayTextLine(2, "g(%d, %d)",py,px);
		path[count++]=y;
		path[count++]=x;
	}

	x=px;y=py;
	px = node_map[y][x].parentx;
	py = node_map[y][x].parenty;

	path[count++]=y;
	path[count++]=x;


	for(int i=0; i<count; i++){
		displayTextLine(2, "go to(%d, %d)", path[i],path[++i]);
		wait1Msec(100);
	}
	navigation(count);

}

/*
void get_distance(){
	int x, y,px,py, count;
	px=goal_x;
	py=goal_y;
	count = 1;
	while (!((px==start_x)&&(py==start_y))){
		x=px;y=py;
		px = node_map[y][x].parentx;
		py = node_map[y][x].parenty;
		count++;
	}
	displayTextLine(2, "dist: %d steps", count);
	wait1Msec(100);
}
*/
//function to find goal
void findGoal(){
	int lowestx;
	int lowesty;

	init_map();

	createopen();
	insert_by_priority(node_map[start_y][start_x]);

	while(true){
		get_lowest_f_cost(lowestx, lowesty);
		add_to_closed(lowestx, lowesty);
		if(is_goal(lowestx, lowesty)){
			displayTextLine(2, "found path");
			wait1Msec(100);
			get_path();
			//get_distance();
			return;
		}
		//neighbours
		get_neighbours(lowestx, lowesty);
		for(int i=0; i<CONNECTIVITY;i++){
			if((current_neighbours[i].val==OBST)||(current_neighbours[i].onclosed==true)){}
			else if((current_neighbours[i].g>(get_g_cost(current_neighbours[i].x,current_neighbours[i].y,lowestx, lowesty)))||
				current_neighbours[i].onopen==false){
				set_cost(current_neighbours[i].x,current_neighbours[i].y,lowestx, lowesty);
				current_neighbours[i].parentx=lowestx;
				current_neighbours[i].parenty=lowesty;
				memcpy(&node_map[current_neighbours[i].y][current_neighbours[i].x],&current_neighbours[i],sizeof(node_map[current_neighbours[i].y][current_neighbours[i].x]));
				if(current_neighbours[i].onopen==false){
					insert_by_priority(current_neighbours[i]);
				}
			}
		}
	}

}
task main()
{

	int sourcex = -1;
	int sourcey = -1;
	int destx = -1;
	int desty = -1;
	//receiving messages via bluetooth
	while (true){
		//	displayTextLine(1, "me ");
		displayTextLine(1, "got %d", messageParm[0]);
		displayTextLine(2, "got2.. %d", messageParm[1]);

		if (messageParm[0]==21){ //if messageid is 21, store location of passenger
			//displayTextLine(1, "work? %d", messageParm[0]);
			sourcex=messageParm[1];
			sourcey=messageParm[2];
			displayTextLine(3, "got %d", messageParm[1]);
			wait1Msec(100);
			ClearMessage();
		}

		if (messageParm[0]==22){ //if messageid is 22, store destination of passenger
			destx=messageParm[1];
			desty=messageParm[2];
			displayTextLine(4, "got again %d", messageParm[1]);
			wait1Msec(100);
			ClearMessage();
		}
		if ((sourcex!=-1) && (sourcey!=-1) && (destx!=-1) && (desty!=-1)){
			break;
		}
	}

	start_x = taxi2base_x;
	start_y = taxi2base_y;
	goal_x = sourcex;
	goal_y = sourcey;
	 
	 //find goal from taxi station to passenger's location
	findGoal();

		wait1Msec(9000);

	displayTextLine(1, "Going to dropoff ");
	wait1Msec(100);

	start_x = sourcex;
	start_y = sourcey;
	goal_x = destx;
	goal_y = desty;

	//find goal to destination from passenger's location
	findGoal();
		displayTextLine(1, "DONE! :) ");
	wait1Msec(1000);

}
