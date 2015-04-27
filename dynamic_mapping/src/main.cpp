#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

#include <signal.h>
#include <termios.h>

#include "nav_msgs/GetMap.h"
#include "dynamic_mapping/GetDynamicMap.h"
#include "nav_msgs/MapMetaData.h"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include "dynamic_mapping/DynamicGrid.h"
#include "dynamic_mapping/ExportMap.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <nav_msgs/GridCells.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>
#include "ros/console.h"
#include "image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"
#include "mapping.h"
#include <SDL/SDL_image.h>

//#define DEBUG

using namespace std;

nav_msgs::OccupancyGrid::ConstPtr map_;
nav_msgs::OccupancyGrid::ConstPtr slam_map_;


class DynamicMapping
{
public:

	DynamicMapping(tf::TransformListener *listener)
	{
		tf_listener = listener;
		double res = 0.0;
		std::string mapfname = "";   
		double origin[3];
		int negate;
		uint32_t map_array_size;
		uint32_t *dynamic_imp;
		double occ_th, free_th;
		ros::NodeHandle nh("~");
		/*
		 * read command line params
		 */
		// yaml file to be importet (map import)
		nh.param("file_name", fname, std::string(""));
		
		// size of the area to check if there are unknown pixels arround a pixel
		nh.param("area_size", area_size, 10);
		
		// size of the area to check if there are free pixels arround a pixel
		nh.param("free_area_size", free_area_size, 3);
		
		// weight factor map_weight = dyn_map / slam_map at the map merging
		// near to 1 means the dyn_map pixel have more impact on the merged map
		// near to 0 means the slam_map pixel have more impact on the merged map
		nh.param("map_weight", map_weight, 0.8);		
		
		// windows size for the deltation algorithm
		// bigger value leads to more delation
		nh.param("dilate_size", dilate_size, 5);	
	
		// dynamic_weight for dynamic rate calculation
		nh.param("dynamic_weight", dynamic_weight, 0.8);	
		
		// dynamic_time_max for calculating dynamic areas in milliseconds
		//nh.param("dynamic_time_max", dynamic_time_max, 600000);	// 10 minutes
		nh.param("dynamic_time_max", dynamic_time_max, 60000);	// 1 minutes
		
		// dynamic_remove_time for resetting dynamic areas in milliseconds
		nh.param("dynamic_remove_time", dynamic_remove_time, 86400000);	// 1 day
		
		// time_period for updating dynamic_time values in milliseconds
		nh.param("time_period", time_period, 5000);		
		
		// hysteresis for calculating the trend
		// set this to 1 or higher to avoid extrema caused by measurement errors
		nh.param("hysteresis", hysteresis, 0);	
		
		// laser_map topic usage
		// set this to 1 to use the laser_map topic instead of the probability for generating dynamic areas
		nh.param("laser_map", use_laser_map, 1);
			
		init_dynamic_trend = true;
		init_dynamic_time = true;

		deprecated = (res != 0);
		if(strcmp(fname.c_str(), "") != 0)	{
			map_is_imported = true;
			ROS_INFO("Starting import of map '%s'... \n", fname.c_str());
			if (!deprecated) {
				std::ifstream fin(fname.c_str());
				if (fin.fail()) {
					ROS_ERROR("Map_import could not open %s.", fname.c_str());
					exit(-1);
				}
				YAML::Parser parser(fin);   
				YAML::Node doc;
				parser.GetNextDocument(doc);
				
				try { 
					doc["resolution"] >> res; 
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
					exit(-1);
				}
				try { 
					doc["negate"] >> negate; 
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("The map does not contain a negate tag or it is invalid.");
					exit(-1);
				}
				try { 
					doc["map_array_size"] >> map_array_size; 
					dynamic_imp = (uint32_t*) malloc(map_array_size * sizeof(uint32_t));
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("The map does not contain a negate tag or it is invalid.");
					exit(-1);
				}
				try { 
					const YAML::Node& dynamics = doc["dynamic"];
					for(int i = 0; i < dynamics.size(); i++)	{		
						dynamics[i] >> dynamic_imp[i]; 		
					}	
					ROS_INFO("dynamic value import successful\n");		
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("The map does not contain a dynamic tag or it is invalid.");
					exit(-1);
				}
				try { 
					if(const YAML::Node *trend= doc.FindValue("dynamic_trend"))	{				
						int32_t *dynamic_tmp = (int32_t*) malloc(map_array_size * sizeof(int32_t));	
						dynamic_trend = (int8_t*) malloc(map_array_size * sizeof(int8_t));	
						for(int i = 0; i < (*trend).size(); i++)	{		
							(*trend)[i] >> dynamic_tmp[i]; 	
							dynamic_trend[i] = (int8_t) dynamic_tmp[i];	
						}		
						init_dynamic_trend = false;
						ROS_INFO("dynamic_trend import successful\n");
					}	
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("The file does not contain a dynamic_trend tag or it is invalid.");
					exit(-1);
				}
				try { 
					if(const YAML::Node *time = doc.FindValue("dynamic_time"))	{		
						dynamic_time = (uint32_t*) malloc(map_array_size * sizeof(uint32_t));	
						for(int i = 0; i < (*time).size(); i++)	{		
							(*time)[i] >> dynamic_time[i]; 		
						}		
						init_dynamic_time = false;
					}	
					ROS_INFO("dynamic_time import successful\n");
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("TThe file does not contain a dynamic_time tag or it is invalid.");
					exit(-1);
				}
				try { 
					doc["occupied_thresh"] >> occ_th; 
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
					exit(-1);
				}
				try { 
					doc["free_thresh"] >> free_th; 
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
					exit(-1);
				}
				try { 
					doc["origin"][0] >> origin[0]; 
					doc["origin"][1] >> origin[1]; 
					doc["origin"][2] >> origin[2]; 
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("The map does not contain an origin tag or it is invalid.");
					exit(-1);
				}
				try { 
					doc["image"] >> mapfname; 
					if(mapfname.size() == 0)
					{
						ROS_ERROR("The image tag cannot be an empty string.");
						exit(-1);
					}
					if(mapfname[0] != '/')
					{
						// dirname can modify what you pass it
						char* fname_copy = strdup(fname.c_str());
						mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
						free(fname_copy);
					}
				} catch (YAML::InvalidScalar) { 
					ROS_ERROR("The map does not contain an image tag or it is invalid.");
					exit(-1);
				}
			} else {
				nh.param("negate", negate, 0);
				nh.param("occupied_thresh", occ_th, 0.65);
				nh.param("free_thresh", free_th, 0.196);
				mapfname = fname;
				origin[0] = origin[1] = origin[2] = 0.0;
			}

			ROS_INFO("Loading map from \"%s\"", fname.c_str());
			FILE* file = fopen(fname.c_str(), "r");
			char buf[512];
			size_t nread;
			if(file)	{
				while((nread = fread(buf, 1, sizeof(buf), file)) > 0)
					fwrite(buf, 1, nread, stdout);
			}
			loadMapFromFile(&dyn_map_import,mapfname.c_str(),res,negate,occ_th,free_th, origin, dynamic_imp);
			dyn_map_import.map.info.map_load_time = ros::Time::now();
			dyn_map_import.map.header.frame_id = "dyn_map";
			dyn_map_import.map.header.stamp = ros::Time::now();
			dyn_map_import.map.dynamic_time_max = dynamic_time_max;
			ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
				dyn_map_import.map.info.width,
				dyn_map_import.map.info.height,
				dyn_map_import.map.info.resolution);
			meta_data_message_ = dyn_map_import.map.info;
			dyn_map_out = dyn_map_import;

			map_import.map.info = dyn_map_import.map.info;
			map_import.map.header = dyn_map_import.map.header;
			map_import.map.data = dyn_map_import.map.data;
		}	
		else	{
			ROS_INFO("Starting with an empty and new map from the scratch... \n");
			map_is_imported = false;
			dyn_map_import.map.info.map_load_time = ros::Time::now();
			dyn_map_import.map.header.frame_id = "dyn_map";
			dyn_map_import.map.header.stamp = ros::Time::now();
			dyn_map_import.map.dynamic_time_max = dynamic_time_max;
			dyn_map_out = dyn_map_import;
		}
		// subscribe to slam_map topic
		slam_map = n.subscribe("slam_map", 1, &DynamicMapping::slamMapCallback, this);

		// subscribe to initial position topic
		init_pose = n.subscribe("initialpose", 1, &DynamicMapping::initPoseCallback, this);

		// subscribe to laser scan topic
		sub_laser = n.subscribe("scan", 1, &DynamicMapping::laserCallback, this);

		// service for map export
		service = n.advertiseService("export_map", &DynamicMapping::export_map, this);
		
		if(use_laser_map == 1)	{
			laser_map = n.subscribe("laser_map", 1, &DynamicMapping::laserMapCallback, this);
			hasLaserMap = false;
			hasOldLaserMap = false;
		}

		// Latched publisher for metadata
		metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
		map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
		dyn_map_pub = n.advertise<dynamic_mapping::DynamicGrid>("dyn_map", 1, true);  
		if(map_is_imported)	{
			map_out.map.info = dyn_map_out.map.info;
			map_out.map.header.frame_id = "map";
			map_out.map.data = dyn_map_out.map.data;
			map_pub.publish( map_out.map );
			dyn_map_pub.publish( dyn_map_out.map );
			metadata_pub.publish( meta_data_message_ );
		}

		// Init merging variables
		position_ready = false;
		x_position_final = 0;
		y_position_final = 0;
		angle_final = 0;
		first_time = true;
		
		// start timer
		time_update = nh.createTimer(ros::Duration(time_period/1000), &DynamicMapping::dynamicTimeUpdateCallback, this);
	}

private:
	/*
	 * PRIVATE VARIBALES
	 */
	// publisher, subscriber, services, listener
	ros::NodeHandle n;
	ros::Publisher map_pub;
	ros::Publisher dyn_map_pub;
	ros::Publisher dyn_area_pub;
	ros::Publisher metadata_pub;
	ros::Subscriber slam_map;
	ros::Subscriber laser_map;
	ros::Subscriber init_pose;
	ros::Subscriber sub_laser;
	ros::ServiceServer service;
	ros::Timer time_update;
	tf::TransformListener *tf_listener;
	
	// dynamic value calculation variables
	int8_t *dynamic_trend;
	bool init_dynamic_trend;
	uint32_t *dynamic_time;
	bool init_dynamic_time;
	int time_period;
	int hysteresis;
	int use_laser_map;
	bool hasLaserMap, hasOldLaserMap;

	// map data is cached here
	nav_msgs::MapMetaData meta_data_message_;
	nav_msgs::GetMap::Response map_import, map_out, map_slam, map_laser, map_laser_old;
	dynamic_mapping::GetDynamicMap::Response dyn_map_import;
	dynamic_mapping::GetDynamicMap::Response dyn_map_out;

	// dynamic update area
	geometry_msgs::PolygonStamped update_area;
	ros::Publisher laser_polygon_pub;
	unsigned int update_area_size;

	// variables of the postion of the robot in the imported map
	bool position_ready;
	double x_position_final;
	double y_position_final;
	double angle_final;
	
	// params to be overwritten by commandline	
	std::string fname;
	int area_size;
	int free_area_size;
	double map_weight;
	int dilate_size;
	double dynamic_weight;
	int dynamic_time_max;
	int dynamic_remove_time;
	
	// auxilary variables
	bool deprecated;
	bool map_is_imported;
	bool first_time;


	/*
	 * PRIVATE FUNCIONS
	 */

 /*
	* check if point is in update_area polygon
	* if yes, increase its dynamic_time value
	*/
	void dynamicTimeUpdateCallback(const ros::TimerEvent& event)	{
		tf::StampedTransform transform;
		tf_listener->lookupTransform("/update_area", "/dyn_map", ros::Time(0), transform);

		for(unsigned int x = 0; x < dyn_map_import.map.info.width; ++x)	{
			for(unsigned int y = 0; y < dyn_map_import.map.info.height; ++y)	{
				unsigned int i = x + (dyn_map_import.map.info.height - y -1) * dyn_map_import.map.info.width;
				if(dyn_map_import.map.data[i] != -1)	{
					tf::Vector3 map_point(
					((int) x - (int) (dyn_map_import.map.info.width / 2)) * dyn_map_import.map.info.resolution, 
					 ((int) (dyn_map_import.map.info.height / 2) - (int) y) * dyn_map_import.map.info.resolution,
					 0.0);
					try {			
						// updating dynamic_time values if they are in the scanned area
							tf::Vector3 laser_point;															
							laser_point = transform * map_point;
							if(pointInPolygon(laser_point.x(), laser_point.y()))	{
								if(!use_laser_map || use_laser_map && laser_map_static_area(i, 10, true))	{
									dynamic_time[i] = dynamic_time[i] + time_period;
									if(dynamic_time[i] > dynamic_remove_time)
										dyn_map_import.map.dynamic[i] = 0;
									}
							}						
					}
					catch(tf::TransformException& e)	{
						ROS_ERROR("Transform error: %s", e.what());
					}
				}
			}
		}
	}

	void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr init_pose)	{
		if(!position_ready)	{
			// set initialpose of the map if not already set
			x_position_final = init_pose->pose.pose.position.x;
			y_position_final = init_pose->pose.pose.position.y;
			// matrix to angle calculation (euler numeric integration)
			angle_final = atan2(2*(init_pose->pose.pose.orientation.z * (- init_pose->pose.pose.orientation.w)),
				1-2*(init_pose->pose.pose.orientation.z*init_pose->pose.pose.orientation.z));	
			position_ready = true;
			// take the whole importet map at first
			dyn_map_out = dyn_map_import;
			ROS_INFO("-----> Initial robot Position: (x,y,angle) : (%f, %f, %f)\n",  x_position_final, y_position_final, angle_final);

		}
	}

	bool slam_map_known_area(unsigned int slam, int window_size)	{
		for(int h = -window_size / 2; h < window_size / 2; h++)	{
			for(int w = -window_size / 2; w < window_size / 2; w++)	{
				int index = slam + (h*map_slam.map.info.width) + w;
				if(index < map_slam.map.info.width * map_slam.map.info.height &&
					index >= 0 &&
					map_slam.map.data[index] == -1)	 {
						return false;
				}
			}
		}
		return true;
	}

	bool dyn_map_known_area(unsigned int dyn, int window_size)	{
		for(int h = -window_size / 2; h < window_size / 2; h++)	{
			for(int w = -window_size / 2; w < window_size / 2; w++)	{
				int index = dyn + (h*dyn_map_out.map.info.width) + w;
				if(index < dyn_map_out.map.info.width * dyn_map_out.map.info.height &&
					index >= 0 &&
					dyn_map_out.map.data[index] == -1)	 {
						return false;
				}
			}
		}
		return true;
	}
	
	bool laser_map_static_area(unsigned int dyn, int window_size, bool isOldMap)	{
		for(int h = -window_size / 2; h < window_size / 2; h++)	{
			for(int w = -window_size / 2; w < window_size / 2; w++)	{
				if(isOldMap)	{
					int index = dyn + (h*map_laser_old.map.info.width) + w;
					if(index > map_laser_old.map.info.width * map_laser_old.map.info.height || index < 0)
						continue;
					if(map_laser_old.map.data[index] == 100)	 {					
							return false;
					}
				}
				else {
					int index = dyn + (h*map_laser.map.info.width) + w;
					if(index > map_laser.map.info.width * map_laser.map.info.height || index < 0)
						continue;
					if(map_laser.map.data[index] == 100)	 {					
							return false;
					}
				}
			}
		}
		return true;
	}
	
	void dyn_map_dilation(unsigned int dyn, int window_size)	{
		for(int k = 0; k < 2; k++)	{
			int first = -window_size - 1;
			int first_dyn = 0;
			int second = -window_size - 1;
			int second_dyn = 0;
			for(int h = -window_size / 2; h < window_size / 2; h++)	{
				int index = 0;
				if(k == 0)
					index = dyn + (h*dyn_map_out.map.info.width);
				else
					index = dyn + h;
				if(index < dyn_map_out.map.info.width * dyn_map_out.map.info.height 
				&& index >= 0 && dyn_map_out.map.dynamic[index])	 {
							if(h < 0)	{
								first = h;
								first_dyn = dyn_map_out.map.dynamic[index];
							}
							if(h > 0)	{
								second = h;
								second_dyn = dyn_map_out.map.dynamic[index];
							}
						
					}
			}
			if(first > -window_size - 1 && second > -window_size - 1)	{
				for(int i = first; i < second; i++)	{
					int index = 0;
						if(k == 0)
							index = dyn + (i*dyn_map_out.map.info.width);
						else
							index = dyn + i;
					dyn_map_out.map.dynamic[index] = (first_dyn + second_dyn) / 2;
				}
			}
		}
	}


	/* 
	* merging the slam_map from the gmapping module and the importet dyn_map 
	* and publish it as dyn_map type and map type
	*/
	int8_t mergeDynamicMaps(unsigned int dyn, unsigned int slam)	
	{
		/*
		* case slam_map area is known, dyn_map area is unknown
		*/ 

		if(map_slam.map.data[dyn] != -1 && dyn_map_out.map.data[dyn] != -1)	{
		
			// init trend
			if(dynamic_trend[dyn] == -1)	{
				if(dyn_map_out.map.data[dyn] == 0)
					dynamic_trend[dyn] = 0;	
				else				
					dynamic_trend[dyn] = 1;
			}
		
			if(slam_map_known_area(slam, area_size) && dyn_map_known_area(dyn, area_size)) { 			
				// use map_laser or map_slam for dynamic area generation
				if(use_laser_map == 1)	{
					if(hasOldLaserMap && (map_laser.map.data[slam] == 100 || map_laser_old.map.data[slam] == 100) && map_laser.map.data[slam] != map_laser_old.map.data[slam])	{
						if((map_laser.map.data[slam] == 100 && laser_map_static_area(slam, 10, true)) ||  
								(map_laser_old.map.data[slam] == 100 && laser_map_static_area(slam, 10, false)))	{
							if(dyn_map_out.map.dynamic[dyn] == 0)	{
				 					if(dynamic_time[dyn] == 0)
				 						dyn_map_out.map.dynamic[dyn] =  time_period;
				 					else
				 						dyn_map_out.map.dynamic[dyn] =  dynamic_time[dyn];
				 			}
				 			else	{
								dyn_map_out.map.dynamic[dyn] = dyn_map_out.map.dynamic[dyn] * dynamic_weight +  dynamic_time[dyn] * (1 - dynamic_weight);
							}
								dynamic_time[dyn] = 0;
						}
					}
				}			
				// change dynamic value
				else if(map_slam.map.data[slam] != dyn_map_out.map.data[dyn] )	{
						// Tiefpunkt or Hochpunkt
						if((map_slam.map.data[dyn] > dyn_map_out.map.data[dyn] && dynamic_trend[dyn] == 0) ||
							(map_slam.map.data[dyn] < dyn_map_out.map.data[dyn] && dynamic_trend[dyn] == 1))	{
							#ifdef DEBUG
								printf("dynamic_time[%d]: %d\n", dyn, dynamic_time[dyn]);
							#endif
			 				if(dyn_map_out.map.dynamic[dyn] == 0)	{
			 					if(dynamic_time[dyn] == 0)
			 						dyn_map_out.map.dynamic[dyn] =  time_period;
			 					else
			 						dyn_map_out.map.dynamic[dyn] =  dynamic_time[dyn];
			 				}
			 				else	{
								dyn_map_out.map.dynamic[dyn] = dyn_map_out.map.dynamic[dyn] * dynamic_weight +  dynamic_time[dyn] * (1 - dynamic_weight);
							}
							dynamic_time[dyn] = 0;
						}
				}
			}
		 /*
			* calculate trend
			*/	
			if(map_slam.map.data[dyn] > dyn_map_out.map.data[dyn] + hysteresis)
				dynamic_trend[dyn] = 1;
			else if(map_slam.map.data[dyn] + hysteresis < dyn_map_out.map.data[dyn])
				dynamic_trend[dyn] = 0;
		}
			
	}

	int8_t mergeDataMaps(unsigned int dyn, unsigned int slam)	{
		dyn_map_out.map.data[dyn] = (int8_t) (map_slam.map.data[slam] * (float) (1.0-map_weight)
			+ (float) dyn_map_out.map.data[dyn] * map_weight);
	}


	/*
	* Laser Callback for determining the update area
	*/
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr &lscan)
	{
		int laserSize = (lscan->angle_max - lscan->angle_min) / lscan->angle_increment;
		float polyX[laserSize];
		float polyY[laserSize];
		
		update_area_size = laserSize + 1;
		update_area.polygon.points.resize(update_area_size);
		update_area.header.seq = 0;
		update_area.header.frame_id = "/update_area";
		update_area.header.stamp = ros::Time(0);	// ros::Time::now() will lead to ROS problems

		for(int i = 0; i < laserSize; i++)	
		{
		update_area.polygon.points[i].x = lscan->ranges[i] * sin(lscan->angle_min + i * lscan->angle_increment);
		update_area.polygon.points[i].y = lscan->ranges[i] * cos(lscan->angle_min + i * lscan->angle_increment);
		update_area.polygon.points[i].z = 0;

		polyX[i] = lscan->ranges[i] * sin(lscan->angle_min + i * lscan->angle_increment);
		polyY[i] = lscan->ranges[i] * cos(lscan->angle_min + i * lscan->angle_increment);

		}
		// add the robot origin as last point
		update_area.polygon.points[laserSize].x = 0;
		update_area.polygon.points[laserSize].y = 0;
		update_area.polygon.points[laserSize].z = 0;

		laser_polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("update_area", 1, true);   
		laser_polygon_pub.publish( update_area );
	}
	
	bool pointInPolygon(float x, float y) {
		int   i = 0, j=update_area_size-1 ;
		bool  oddNodes=false;
		for (i=0; i<update_area_size; i++) {
			if (update_area.polygon.points[i].y < y && update_area.polygon.points[j].y >= y
			||  update_area.polygon.points[j].y < y && update_area.polygon.points[i].y >= y) {
				if (update_area.polygon.points[i].x +
				(y-update_area.polygon.points[i].y) / (update_area.polygon.points[j].y - update_area.polygon.points[i].y)*
				(update_area.polygon.points[j].x - update_area.polygon.points[i].x)<x) {
					oddNodes=!oddNodes; 
				}
			}
			j=i; 
		}
		return oddNodes; 
	}

	void laserMapCallback(const nav_msgs::OccupancyGridConstPtr& laser_map)	{
		if(position_ready)	{
			if(dyn_map_import.map.info.resolution != laser_map->info.resolution)	{
				// wrong resolution error
				ROS_ERROR("Laser map must have a resolution of %f.", dyn_map_import.map.info.resolution);
			}
			else	{
				int laser_map_width = (int) laser_map->info.width;
				int laser_map_height = (int) laser_map->info.height;
			
				// save old laser value
				if(hasLaserMap)	{
					// deep copy of laser_map
					map_laser_old.map.data.resize(laser_map_width * laser_map_height);
					for(int x = 0; x < laser_map_width; x++) {
						for(int y = 0; y < laser_map_height; y++) {
							unsigned int i = x + (laser_map_height - y -1) * laser_map_width;
							map_laser_old.map.data[i] = map_laser.map.data[i];
						}
					}
					map_laser_old.map.header = map_laser.map.header;
					map_laser_old.map.info = map_laser.map.info;
					hasOldLaserMap = true;
				}
			
				map_laser.map.data = laser_map->data;
				map_laser.map.header = laser_map->header;
				map_laser.map.info = laser_map->info;

				// position of the robot in the laser map
				int x_position_laser_map = laser_map_width / 2;
				int y_position_laser_map = laser_map_height / 2;		
				int x_position_import_map;
				int y_position_import_map;
				double angle;

			// only merge the maps if the position of the roboter is estimated good enought

				x_position_import_map = (x_position_final / laser_map->info.resolution) + x_position_laser_map;
				y_position_import_map = laser_map_height  - ((y_position_final / laser_map->info.resolution) + y_position_laser_map);
				angle = angle_final;;

				// init new rotated laser_map map
				for(int x = 0; x < laser_map_width; x++) {
					for(int y = 0; y < laser_map_height; y++) {
						unsigned int i = x + (laser_map_height - y -1) * laser_map_width;
						map_laser.map.data[i] = 0;
					}
				}

				for(int x = 0; x < laser_map_width; x++) {
					for(int y = 0; y < laser_map_height; y++) {

						int xt = x - x_position_laser_map;
						int yt = y - y_position_laser_map;

						double sinma = sin(-angle);
						double cosma = cos(-angle);

						int x_old = (int)round((cosma * xt - sinma * yt) + x_position_laser_map);
						int y_old = (int)round((sinma * xt + cosma * yt) + y_position_laser_map);
						// calculate position in the rotated laser map
						unsigned int i = x + (laser_map_height - y -1) * laser_map_width;
						// calculate position in the unrotated laser map
						unsigned int j = x_old + (laser_map_height - y_old -1) * laser_map_width;

						if(x_old >= 0 && x_old < laser_map_width && y_old >= 0 && y_old < laser_map_height) {
							// rotate pixel j to position i
							map_laser.map.data[i] = laser_map->data[j];
						}
					}
				}
				hasLaserMap = true;
			}
		}
	}

	void slamMapCallback(const nav_msgs::OccupancyGridConstPtr& slam_map)
	{
		map_slam.map.data = slam_map->data;
		map_slam.map.header = slam_map->header;
		map_slam.map.info = slam_map->info;

		update_area.header.seq = 0;
		update_area.header.frame_id = "/update_area";
		update_area.header.stamp = ros::Time(0);

		laser_polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("update_area", 1, true);    
		laser_polygon_pub.publish( update_area );

		if(!map_is_imported)	{
			map_slam.map.header.frame_id = "map";
			// Latched publisher for metadata
			metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
			metadata_pub.publish( map_slam.map.info );

			// Latched publisher for data
			map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
			map_pub.publish( map_slam.map );


			dyn_map_import.map.info = slam_map->info;
			dyn_map_import.map.header = slam_map->header;
			dyn_map_import.map.header.frame_id = "dyn_map";
			dyn_map_import.map.data = slam_map->data;
			dyn_map_import.map.dynamic.resize(dyn_map_import.map.info.width * dyn_map_import.map.info.height);
			dyn_map_import.map.dynamic_time_max = dynamic_time_max;

			dynamic_trend = (int8_t*) malloc(dyn_map_import.map.info.width * dyn_map_import.map.info.height);
			if(dynamic_trend == NULL)	{
				ROS_ERROR("MALLOC dynamic_trend array failed.");
				exit(-1);		
			}
		
			dynamic_time = (uint32_t*) malloc(dyn_map_import.map.info.width * dyn_map_import.map.info.height * sizeof(uint32_t));
			if( dynamic_time == NULL)	{
				ROS_ERROR("MALLOC dynamic_time array failed.");
				exit(-1);		
			}
			
			for(int i = 0; i < dyn_map_import.map.info.width * dyn_map_import.map.info.height; i++)	{			
				dyn_map_import.map.dynamic[i] = 0;
				dynamic_trend[i] = -1;
				dynamic_time[i] = 0;
			}
			dyn_map_out = dyn_map_import;
			dyn_map_pub = n.advertise<dynamic_mapping::DynamicGrid>("dyn_map", 1, true);  
			dyn_map_pub.publish( dyn_map_out.map );

			x_position_final = 0;
			y_position_final = 0;
			angle_final = 0;	
			position_ready = true;
			map_is_imported = true;
			init_dynamic_trend = false;
			init_dynamic_time = false;
			return;
		}
		else {
			// init dynamic trend
			if(init_dynamic_trend)	{
				dynamic_trend = (int8_t*) malloc(dyn_map_import.map.info.width * dyn_map_import.map.info.height);
				if(dynamic_trend == NULL)	{
					ROS_ERROR("MALLOC dynamic_trend array failed.");
					exit(-1);		
				}
			}
			//init dynamic time
			if(init_dynamic_time)	{
				dynamic_time = (uint32_t*) malloc(dyn_map_import.map.info.width * dyn_map_import.map.info.height * sizeof(uint32_t));
				if( dynamic_time == NULL)	{
					ROS_ERROR("MALLOC dynamic_time array failed.");
					exit(-1);		
				}
			}
			for(int i = 0; i < dyn_map_import.map.info.width * dyn_map_import.map.info.height; i++)	{			
				if(init_dynamic_trend)	
					dynamic_trend[i] = -1;
				if(init_dynamic_time)
					dynamic_time[i] = 0;
			}
			init_dynamic_trend = false;
			init_dynamic_time = false;
		}

		ROS_INFO("Dynamic map %d X %d map @ %.3lf m/cell",
			dyn_map_import.map.info.width,
			dyn_map_import.map.info.height,
			dyn_map_import.map.info.resolution);

		ROS_INFO("SLAM map %d X %d map @ %.3lf m/cell",
			slam_map->info.width,
			slam_map->info.height,
			slam_map->info.resolution);

		if(dyn_map_import.map.info.resolution != slam_map->info.resolution)	{
			// wrong resolution error
			ROS_ERROR("SLAM map must have a resolution of %f.", dyn_map_import.map.info.resolution);
		}
		else	{
			int slam_map_width = (int) slam_map->info.width;
			int slam_map_height = (int) slam_map->info.height;

			// position of the robot in the slam map
			int x_position_slam_map = slam_map_width / 2;
			int y_position_slam_map = slam_map_height / 2;		
			int x_position_import_map;
			int y_position_import_map;
			double angle;

			// only merge the maps if the position of the roboter is estimated good enought
			if(position_ready)	{
				x_position_import_map = (x_position_final / slam_map->info.resolution) + x_position_slam_map;
				y_position_import_map = slam_map_height  - ((y_position_final / slam_map->info.resolution) + y_position_slam_map);
				angle = angle_final;;

				#ifdef DEBUG
				printf("final_x: %d\n", x_position_import_map);
				printf("final_y: %d\n", y_position_import_map);
				printf("final_winkel: %f\n", angle);
				#endif

				// init new rotated slam_map map
				for(int x = 0; x < slam_map_width; x++) {
					for(int y = 0; y < slam_map_height; y++) {
						unsigned int i = x + (slam_map_height - y -1) * slam_map_width;
						map_slam.map.data[i] = -1;
					}
				}

				/*
				* rotate slam_map to fit in the imported map
				*/
				for(int x = 0; x < slam_map_width; x++) {
					for(int y = 0; y < slam_map_height; y++) {

						int xt = x - x_position_slam_map;
						int yt = y - y_position_slam_map;

						double sinma = sin(-angle);
						double cosma = cos(-angle);

						int x_old = (int)round((cosma * xt - sinma * yt) + x_position_slam_map);
						int y_old = (int)round((sinma * xt + cosma * yt) + y_position_slam_map);
						// calculate position in the rotated slam map
						unsigned int i = x + (slam_map_height - y -1) * slam_map_width;
						// calculate position in the unrotated slam map
						unsigned int j = x_old + (slam_map_height - y_old -1) * slam_map_width;

						if(x_old >= 0 && x_old < slam_map_width && y_old >= 0 && y_old < slam_map_height) {
							// rotate pixel j to position i
							map_slam.map.data[i] = slam_map->data[j];
						}
					}
				}

				int map_import_width = (int) dyn_map_import.map.info.width;
				int map_import_height = (int) dyn_map_import.map.info.height;

				/*
				* calculate startpoint (top-left) and endpoint (right-bottom) 
				* of the imported map 
				*/
				int x_start = max((int) (x_position_import_map - map_import_width / 2), 0);
				int y_start = max((int) (y_position_import_map - map_import_height / 2), 0);
				int x_end = min((int) (x_position_import_map + x_position_slam_map), map_import_width);
				int y_end = min((int) (y_position_import_map + y_position_slam_map), map_import_height);
				int window_width = x_end - x_start;
				int window_height = y_end - y_start;

				/*
				* calculate startpoint (top-left) and endpoint (right-bottom) 
				* of the slam generated map (topic /slam_map)
				*/
				int x_start_s = max(slam_map_width - x_end, 0);
				int y_start_s = max(slam_map_height - y_end, 0);
				int x_end_s = min(x_start_s + window_width, slam_map_width);
				int y_end_s = min(y_start_s + window_height, slam_map_height);

				#ifdef DEBUG
				printf("WINDOW: width: %d, height: %d\n", window_width, window_height);
				printf("IMPORT: startpoint (%d, %d) endpoint (%d, %d)\n", x_start, y_start, x_end, y_end);
				printf("SLAM:   startpoint (%d, %d) endpoint (%d, %d)\n", x_start_s, y_start_s, x_end_s, y_end_s);
				#endif

				// dynamic: merge it with the slam map
				for(unsigned int x = 0; x < window_width; ++x)	{
					for(unsigned int y = 0; y < window_height; ++y)	{
						// calculate position in the importet map
						unsigned int i = x_start + x + (map_import_height - (y_start + y) - 1) * map_import_width;
						// calculate position in the slam map
						unsigned int j = x_start_s + x + (slam_map_height - (y_start_s + y) -1) * slam_map_width;
						// merging logic for dynamic values
						if(dyn_map_out.map.data[i] != -1 && map_slam.map.data[j] != -1)	{
								mergeDynamicMaps(i, j);			
								if(dilate_size > 0)				
									dyn_map_dilation(i, dilate_size);
						}
					}
				}

				// data: merge it with the slam map
				for(unsigned int x = 0; x < window_width; ++x)	{
					for(unsigned int y = 0; y < window_height; ++y)	{
						// calculate position in the importet map
						unsigned int i = x_start + x + (map_import_height - (y_start + y) - 1) * map_import_width;
						// calculate position in the slam map
						unsigned int j = x_start_s + x + (slam_map_height - (y_start_s + y) -1) * slam_map_width;
						// merging logic
						mergeDataMaps(i, j);
					}
				}
				meta_data_message_ = dyn_map_import.map.info;

				ROS_INFO("Publishing merged map now...\n");

				map_out.map.info = dyn_map_out.map.info;
				map_out.map.header.frame_id = "map";
				map_out.map.data = dyn_map_out.map.data;

				// Latched publisher for metadata
				metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
				metadata_pub.publish( meta_data_message_ );

				// Latched publisher for data
				map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
				map_pub.publish( map_out.map );


				// publish dyn_map
				dyn_map_pub = n.advertise<dynamic_mapping::DynamicGrid>("dyn_map", 1, true);    
				dyn_map_pub.publish( dyn_map_out.map );
			}
		}
	}
	
	/*
	 * Service for map export
	 */
	bool export_map(dynamic_mapping::ExportMap::Request &req, dynamic_mapping::ExportMap::Response &res)	{
		ROS_INFO("Service export_map called\n");			
		if(req.init == 1)	{
			printf("init\n");
			res.dynamic_trend.resize(dyn_map_out.map.info.width * dyn_map_out.map.info.height);
			res.dynamic_time.resize(dyn_map_out.map.info.width * dyn_map_out.map.info.height);
			res.dynamic_map.data.resize(dyn_map_out.map.info.width * dyn_map_out.map.info.height);
			res.dynamic_map.dynamic.resize(dyn_map_out.map.info.width * dyn_map_out.map.info.height);
			res.dynamic_map.info = dyn_map_out.map.info;
			res.dynamic_map.header = dyn_map_out.map.header;
			for(int i = 0; i < dyn_map_out.map.info.width * dyn_map_out.map.info.height; i++)	{
				res.dynamic_trend[i] = dynamic_trend[i];
				res.dynamic_time[i] = dynamic_time[i];
				res.dynamic_map.data[i] = dyn_map_out.map.data[i];
				res.dynamic_map.dynamic[i] = dyn_map_out.map.dynamic[i];
			}
			#ifdef DEBUG
				printf("dyn_map_import.map.dynamic_time_max: %d\n", dyn_map_import.map.dynamic_time_max);
			#endif
			res.dynamic_time_max = dyn_map_import.map.dynamic_time_max;
		}
		return true;
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dynamic"); 
	tf::TransformListener listener;
	try
	{
		DynamicMapping ms = DynamicMapping(&listener);
		ros::spin();
	}
	catch(std::runtime_error& e)
	{
		ROS_ERROR("Dynamic Mapping Exception: %s", e.what());
		return -1;
	}

}


