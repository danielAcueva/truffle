#include <stdio.h>
#include "globals.h"

Vector2d ball_values[10];

int tail = 0;
int head = 0;

void reset_ball_values()
{
	//init all queue values to 0
	for (int i = 0; i < 10; i++)
	{
		ball_values[i](0) = 0;
		ball_values[i](1) = 0;
	}
	tail = 1;
	head = 0;
}

void push_ball_values(Vector2d data)
{
	int next_index = ((head+1)%10);	//Next index with rollover logic
	if (next_index == tail)			//The queue is full
	{
		//When queue is full, overwrite push. Get rid of old element
		tail = ((tail + 1) % 10);	//Increment the tail with rollover logic
	} 
	ball_values[next_index] = data;
	head = next_index;
}

Vector2d *return_ball_values()
{
	Vector2d return_values[10];
	int index = 0;
	int temp_tail = tail;
	int temp_head = head;
	while ((temp_tail != temp_head) || (index < 10))
	{
		return_values[index] = ball_values[temp_tail];
		temp_tail = ((temp_tail + 1) % 10);
	}
	return return_values;
}

Vector2d avg_distance_between_samples()
{
	Vector2d* samples = return_ball_values();
	//Vector2d avg_distance[9];
	Vector2d sum_distance;
	sum_distance(0) = 0;
	sum_distance(1) = 0;
	for (int i = 1; i < 10; i++)
	{
		sum_distance(0) += samples[i](0) - samples[i-1](0);
		sum_distance(1) += samples[i](1) - samples[i-1](1);
	}
	return (sum_distance/9);
}