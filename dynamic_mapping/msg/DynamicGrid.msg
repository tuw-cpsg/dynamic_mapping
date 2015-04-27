# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.
# For each cell the dynamic value is also stored.

Header header 

# MetaData for the map (e.g. resolution)
nav_msgs/MapMetaData info

# The reference value for dynamic area colorsiation
# This value will be the deepest red, all values above will also be deep red.
uint32 dynamic_time_max

# The map data, in row-major order, starting with (0,0).  Occupancy
# probabilities are in the range [0,100].  Unknown is -1.
int8[] data

# The exponential moving average time until change in milliseconds for each cell, starting with (0,0). max value 49 days
uint32[] dynamic
