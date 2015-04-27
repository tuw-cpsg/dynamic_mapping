#ifndef MAPPING_H
#define MAPPING_H

#define WALL_RED 		0
#define WALL_GREEN 		0
#define WALL_BLUE 		0

#define FREE_RED 		200
#define FREE_GREEN 		254
#define FREE_BLUE 		200

#define UNKNOWN_RED 	205
#define UNKNOWN_GREEN 	205
#define UNKNOWN_BLUE 	205

#define DYN_STEP		40

#define DYN_RED 		254
#define DYN_GREEN 		(DYN_RED - DYN_STEP)
#define DYN_BLUE 		(DYN_RED - DYN_STEP)

#define PI (3.1415)

#define IMPORT_MAP_WEIGHT	 0.8
#define SLAM_MAP_WEIGHT 	(1 - IMPORT_MAP_WEIGHT)

#endif
