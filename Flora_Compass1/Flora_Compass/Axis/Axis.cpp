#include "Axis.h"
#include <math.h>

// Axis Constructor
Axis::Axis(float min_value,  float max_value, float target_value)
{
	set_values(min_value, max_value, target_value);
	find_scale();
	find_offset();
	set_scaled_values();
}

void Axis::set_values (float min_value, float max_value, float target_value)
{
	original_min = min_value;
	original_max = max_value;
	original_target = target_value;
}

void Axis::find_scale()
{
	float range = fabs(original_max) + fabs(original_min);
	scale = 1.0/range;
}

void Axis::find_offset()
{
	offset = 1.0-(scale * max);
}

void Axis::set_scaled_values()
{
	max = 1.0;
	min = 0.0;
	target = scale_value(original_target);
}

float Axis::scale_value(float value)
{
	value = (value * scale) + offset;

	if (value > max)
		return max;
	else if (value < min)
		return min;
	else
		return value;
}