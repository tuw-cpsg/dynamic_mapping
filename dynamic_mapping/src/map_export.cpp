#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "mapping.h"
#include "dynamic_mapping/DynamicGrid.h"
#include "dynamic_mapping/ExportMap.h"

using namespace std;

class MapGenerator 
{

public:
	MapGenerator(const std::string& mapname) : mapname_(mapname), saved_map_(false)
	{
		ros::NodeHandle nh("~");
		ros::NodeHandle n;
		ROS_INFO("Requesting map on service ExportMap");
		
		
		// if not 0 -> export map in grey values
		nh.param("grey_val", grey_val, 0);		
		
		// export dynamic_trend
		nh.param("trend", trend, 0);	
		
	  // export dynamic_time
		nh.param("time", time, 0);	
		
		// export dynamic_time_max
		nh.param("time_max", time_max, 0);	

		client = n.serviceClient<dynamic_mapping::ExportMap>("export_map");
		srv.request.init = 1;
		if(!client.call(srv))	{
			ROS_ERROR("Couldn't call service ExportMap\n");
		}

		ROS_INFO("Received a %d X %d dyn_map @ %.3f m/pix",
			srv.response.dynamic_map.info.width,
			srv.response.dynamic_map.info.height,
			srv.response.dynamic_map.info.resolution);

		std::string mapdatafile = mapname_ + ".pgm";
		ROS_INFO("Writing dynamic_map occupancy data to %s", mapdatafile.c_str());
		uint32_t dynamic_time_max = srv.response.dynamic_time_max;
		if(grey_val)
		{		
			FILE* out = fopen(mapdatafile.c_str(), "w");
			if (!out)
			{
				ROS_ERROR("Couldn't save dynamic_map file to %s", mapdatafile.c_str());
				return;
			}
			// the next lines make a PGM file
			fprintf(out, "P5\n# CREATOR: dynamic_mapping %.3f m/pix\n%d %d\n255\n",
				srv.response.dynamic_map.info.resolution, srv.response.dynamic_map.info.width, srv.response.dynamic_map.info.height);
			for(unsigned int y = 0; y < srv.response.dynamic_map.info.height; y++) {
				for(unsigned int x = 0; x < srv.response.dynamic_map.info.width; x++) {
					unsigned int i = x + (srv.response.dynamic_map.info.height - y - 1) * srv.response.dynamic_map.info.width;
					if (srv.response.dynamic_map.data[i] == 0) {
						fputc(254, out);
					} else if (srv.response.dynamic_map.data[i] == -1) {
						fputc(000, out);
					} else {
						fputc(srv.response.dynamic_map.data[i]+100, out);
					}
				}
			}

			fclose(out);
		}
		else	
		{
			// create a ppm file (color)
			FILE *fp = fopen((mapname_ + ".ppm").c_str(), "wb");
			(void) fprintf(fp, "P6\n%d %d\n255\n", srv.response.dynamic_map.info.width, srv.response.dynamic_map.info.height);
			for(unsigned int y = 0; y < srv.response.dynamic_map.info.height; y++) {
				for(unsigned int x = 0; x < srv.response.dynamic_map.info.width; x++) {
					static unsigned char color[3];
					unsigned int i = x + (srv.response.dynamic_map.info.height - y - 1) * srv.response.dynamic_map.info.width;
					unsigned char data = (int8_t) ((100 - srv.response.dynamic_map.data[ i ]) * 255 / 100);
					uint32_t dynamic_val = srv.response.dynamic_map.dynamic[ i ];
					
					if ( dynamic_val > srv.response.dynamic_map.dynamic_time_max)
						dynamic_val = srv.response.dynamic_map.dynamic_time_max;
						
					unsigned char dynamic = (int8_t) ((100 - (double)(dynamic_val*100) / (double)dynamic_time_max)* 255 / 100);
					// unknown space
					if(srv.response.dynamic_map.data[ i ] == -1)	{
						color[0] = 200;
						color[1] = 200;
						color[2] = 255;
					}
					// known spacce
					else	{
						color[0] = data;
						color[1] = data;
						color[2] = data;
						// dynamic area
						if(srv.response.dynamic_map.dynamic[ i ] > 0)	{
							color[0] = min(dynamic + 0, 255);
							color[1] = max(data / 5, 0);
							color[2] = max(data / 5, 0);
						}
					}
					(void) fwrite(color, 1, 3, fp);
				}
			}
			(void) fclose(fp);
		}

		std::string mapmetadatafile = mapname_ + ".yaml";
		ROS_INFO("Writing map header file %s", mapmetadatafile.c_str());
		FILE* header = fopen(mapmetadatafile.c_str(), "w");



		geometry_msgs::Quaternion orientation = srv.response.dynamic_map.info.origin.orientation;
		tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
		double yaw, pitch, roll;
		mat.getEulerYPR(yaw, pitch, roll);

		// basename may modify content, so make copy
		char* mapname_copy = strdup(mapname_.c_str());
		char* base = basename(mapname_copy);
		std::string relative_mapdatafile; 
		if(grey_val)
			relative_mapdatafile= std::string(base) + ".pgm";
		else
			relative_mapdatafile= std::string(base) + ".ppm";
			
		free(mapname_copy);

		fprintf(header, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
			relative_mapdatafile.c_str(), srv.response.dynamic_map.info.resolution, srv.response.dynamic_map.info.origin.position.x, srv.response.dynamic_map.info.origin.position.y, yaw);
			
	/*
	 * Export dynamic values
	 */
		fprintf(header, "map_array_size: %d\n", srv.response.dynamic_map.info.height * srv.response.dynamic_map.info.width);
		fprintf(header, "dynamic: [");
		for(unsigned int y = 0; y < srv.response.dynamic_map.info.height; y++) {
				for(unsigned int x = 0; x < srv.response.dynamic_map.info.width; x++) {
					unsigned int i = x + (srv.response.dynamic_map.info.height - y - 1) * srv.response.dynamic_map.info.width;
					if(x==0 && y == 0)
						fprintf(header, "%d", srv.response.dynamic_map.dynamic[ i ]);
					else
						fprintf(header, ",%d", srv.response.dynamic_map.dynamic[ i ]);
				}
		}
		fprintf(header, "]\n");
		
	/*
	 * Export dynamic_trend if param "trend" is not 0
	 */
		if(trend)	{
			fprintf(header, "dynamic_trend: [");
			for(unsigned int y = 0; y < srv.response.dynamic_map.info.height; y++) {
				for(unsigned int x = 0; x < srv.response.dynamic_map.info.width; x++) {
					unsigned int i = x + (srv.response.dynamic_map.info.height - y - 1) * srv.response.dynamic_map.info.width;
					if(x==0 && y == 0)
						fprintf(header, "%d", srv.response.dynamic_trend[ i ]);
					else
						fprintf(header, ",%d", srv.response.dynamic_trend[ i ]);				
				}
			}
			fprintf(header, "]\n");
		}
		
	/*
	 * Export dynamic_time if param "time" is not 0
	 */
		if(trend)	{
			fprintf(header, "dynamic_time: [");
			for(unsigned int y = 0; y < srv.response.dynamic_map.info.height; y++) {
				for(unsigned int x = 0; x < srv.response.dynamic_map.info.width; x++) {
					unsigned int i = x + (srv.response.dynamic_map.info.height - y - 1) * srv.response.dynamic_map.info.width;
					if(x==0 && y == 0)
						fprintf(header, "%d", srv.response.dynamic_time[ i ]);
					else
						fprintf(header, ",%d", srv.response.dynamic_time[ i ]);				
				}
			}
			fprintf(header, "]\n");
		}
			
		/*
	 * Export dynamic_time_max if param "time_max" is not 0
	 */
		if(time_max)	{	
			fprintf(header, "dynamic_time_max: %d\n", srv.response.dynamic_time_max);		
		}
				
		if(grey_val)
			fprintf(header, "dynamic_map: false\n");
		else
			fprintf(header, "dynamic_map: true\n");
			
		fclose(header);

		ROS_INFO("Done\n");
		saved_map_ = true;
	}

	std::string mapname_;
	ros::Subscriber map_sub_;
	ros::Subscriber dyn_map_sub_;
	bool saved_map_;
	dynamic_mapping::ExportMap srv;
	ros::ServiceClient client;
	
	//parameters
	int grey_val;
	int time;
	int trend;
	int time_max;

};

#define USAGE "Usage: \n" \
	"  map_export -h\n"\
	"  map_export [-f <mapname>] [parameter]\n "\
	"  the topic 'dyn_map' is exported \n "\
	"  As default a PPM color map is exported \n "\
	"  a grey-value map can be exported by adding '_grey_val:=1' \n "\ 
	"  the dynamic_trend can be exported by adding '_trend:=1' \n "\ 
	"  the dynamic_time can be exported by adding '_time:=1' \n "\ 
	"  the dynamic_time_max can be exported by adding '_time_max:=1' \n "\ 

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "map_export");
	std::string mapname = "map";


	for(int i=1; i<argc; i++)
	{
		if(!strcmp(argv[i], "-h"))
		{
			puts(USAGE);
			return 0;
		}
		else if(!strcmp(argv[i], "-f"))
		{
			if(++i < argc)
				mapname = argv[i];
			else
			{
				puts(USAGE);
				return 1;
			}
		}
	}

	MapGenerator mg(mapname);

	while(!mg.saved_map_)
		ros::spinOnce();

	return 0;
}


