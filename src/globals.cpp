#include <stdio.h>
#include "globals.h"

//Vector2d ball_values[10];
#define QUEUE_SIZE 10

using namespace std;

queue<Vector2d> stored_ball_values;

int tail = 0;
int head = 0;

void reset_ball_values()
{
	//init all queue values to 0
	for (int i = 0; i < QUEUE_SIZE; i++)
	{
		Vector2d queue_data;
		queue_data(0) = 0;
		queue_data(1) = 0;
		stored_ball_values.push(queue_data);
	}
}

void push_ball_values(Vector2d data)
{
	if (stored_ball_values.size() < QUEUE_SIZE)
	{
		stored_ball_values.push(data);
	}
	else
	{
		stored_ball_values.pop();
		stored_ball_values.push(data);
	}
}

Vector2d *return_ball_values()
{
	Vector2d return_values[10];
	queue<Vector2d> temp_queue = stored_ball_values;
	int index = 0;
	for (int i =0; i < QUEUE_SIZE; i++)
	{
		return_values[index] = temp_queue.front();
		temp_queue.pop();
		index++;
	}
	return return_values;
}

Vector2d avg_distance_between_samples()
{
	Vector2d samples[10];
	queue<Vector2d> temp_queue = stored_ball_values;
	int index = 0;
	for (int i =0; i < QUEUE_SIZE; i++)
	{
		samples[index] = temp_queue.front();
		temp_queue.pop();
		index++;
	}
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