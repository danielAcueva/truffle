#include "quadrant.h"

using namespace std;
using namespace geometry_msgs; 
using namespace Eigen;

#define ROW_OFFSET 4  // number of columns per row

/////////Quadrant Coordinate System\\\\\\\\\\

    /*   Col0 . Col1 . Col2 . Col3
    *   ______.______.______._______
    *  |      .      .      .      |
    *  |  Q0  .  Q1  .  Q2  .  Q3  |   Row0
    *  |...........................|.....
    * _|      .      .      .      |_
    *|    Q4  .  Q5  .  Q6  .  Q7    | Row1
    *|...............................|...
    *|        .      .      .        |
    *|_   Q8  .  Q9  .  Q10 .  Q11  _| Row2
    *  |...........................|.....
    *  |      .      .      .      |  
    *  |  Q12 .  Q13 .  Q14 .  Q15 |   Row3
    *  |______.______.______.______|   
    *
    */

//////////////////////\\\\\\\\\\\\\\\\\\\\\\\

//get the quadrant row of an item from the XY coordinates
int get_quadrant_row(Vector2d field_item)
{
	double y_pos = field_item(1);		//Get the y position
	int row_num = 0;					//Init row number variable
	//Find the row
	if (y_pos > (FIELD_HEIGHT/4))		//Row 3 cutoff
	{
		row_num = 3;					//We are in row 3
	}
	else if (y_pos > 0)					//Row 2 cutoff
	{
		row_num = 2;					//We are in row 2
	} 
	else if (y_pos > -(FIELD_HEIGHT/4))	//Row 1 cutoff
	{
		row_num = 1;					//We are in row 1
	}	
	else								//Row 0 cutoff
	{ 
		row_num = 0;					//We are in row 0 
	}	
	return row_num;						//Return the row number						
}

//get the quadrant column of an item from the XY coordinates
int get_quadrant_column(Vector2d field_item)
{
	double x_pos = field_item(0);		//Get the x position
	int column_num = 0;					//init the column number variable
	//Find the row
	if (x_pos > (FIELD_WIDTH/4))		//Row 3
	{
		column_num = 3;					//We are in column 3
	}
	else if (x_pos > 0)					//Row 2
	{
		column_num = 2;					//We are in column 2
	} 
	else if (x_pos > -(FIELD_WIDTH/4))	//Row 1
	{
		column_num = 1;					//We are in column 1
	}	
	else								//Row 0
	{ 
		column_num = 0;					//We are in column 0
	}	
	return column_num;					//Return the column number		
}

//Get the quadrant number from the XY coordinates.
//See the above quadrant coordinate system as a reference
int get_quadrant(Vector2d field_item)
{
	//The coordinate system has an origin (0,0) at the center of the field
	//Positions increase when you move right and up from there
	//Positions decrease left and down from there, being negative_
	int row_number = get_quadrant_row(field_item);			//Get the row
	int column_number = get_quadrant_column(field_item);	//Get the column
	int quadrant = (row_number*ROW_OFFSET) + column_number;	//Compute the quadrant
	return quadrant;										//Return the Quadrant
}

//Return the row number from the quadrant number
int return_row(int quadrant)
{
	return quadrant/4;					//This will return the row. It will round down
										//From divided value to return row
}

//Return the column number from the quadrant number
int return_column(int quadrant)
{
	return quadrant%4;					//This will return the column. It will return
										//the modded value to return column
}

// This function returns the center position of the quadrant
// This is useful if you need to move to a quadrant
Vector2d get_quadrant_center(int quadrant)
{
	Vector2d position;								//Create a position variable, 
													//to hold return value
	position(0) = 0;								//Init x to 0
	position(1) = 0;								//init y to 0
	int row = return_row(quadrant);					//Get the row. for switch statements
	int column = return_column(quadrant);			//Get the column. for switch statements
	switch (row)									//Switch on row to find y position
	{
		case 0:										//Row 0
		{
			position(0) = -(3*(FIELD_HEIGHT/8)); 	//(3/8) down the field
			break;
		}
		case 1:										//Row 1
		{
			position(0) = -(FIELD_HEIGHT/8); 		//(1/8) down the field
			break;
		}
		case 2:										//Row 2
		{
			position(0) = (FIELD_HEIGHT/8); 		//(1/8) up the field
			break;
		}
		default:									//Row 3
		{
			position(0) = 3*(FIELD_HEIGHT/8); 		//(3/8) up the field
			break;			
		}
	}
	switch (column)									//Switch on column to find x position
	{
		case 0:										//column 0
		{
			position(1) = -(3*(FIELD_WIDTH/8)); 	//(3/8) down the field
			break;
		}
		case 1:										//column 1
		{
			position(1) = -(FIELD_WIDTH/8); 		//(1/8) down the field
			break;
		}
		case 2:										//column 2
		{
			position(1) = (FIELD_WIDTH/8); 			//(1/8) up the field
			break;
		}
		default:									//column 3
		{
			position(1) = 3*(FIELD_WIDTH/8); 		//(3/8) up the field
			break;			
		}
	}
	return position;								//Return the calculated position
}

