#ifndef MAP_H_
#define MAP_H_

#include <vector>


class Map
{
public:
	struct landmark
	{
		int id;
		float x; // x-position in the map(global coordinates)
		float y; // y-position in the map(global coordinates)
	};

	std::vector<landmark> landmarks;
};

#endif // !MAP_H_