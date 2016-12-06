#include <stdio.h>

int rng(int low, int high) {
	if (low > high)
	{
		return rng(high, low);
	} else 
	{		
		return (rand() % (high - low + 1)) + low;
	}
}
