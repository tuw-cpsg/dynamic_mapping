/*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*    Copyright © 2010 Srećko Jurić-Kavelj <srecko.juric-kavelj@fer.hr>
*/

/*
*    This was tested on 32-bit Jaunty, Matlab 2009b.
*    To compile, get the flags with:
*    $ rospack cflags-only-I rosrecord
*    $ rospack libs-only-L rosrecord
*    $ rospack libs-only-l rosrecord
*
*    Additionaly, I needed to add "-lboost_system-mt".
*/

#include <string>
#include "matrix.h"

#include "ros/ros.h"
#include "rosrecord/Player.h"
#include "sensor_msgs/LaserScan.h"

#include "mex.h"

#define NUMBER_OF_FIELDS (sizeof(field_names)/sizeof(*field_names))

//extern void _main();

const char *field_names[] = {"scanId",
"timestamp",
"minAngle",
"maxAngle",
"resolution",
"minRange",
"maxRange",
"ranges",
"intensities"};

mxArray *scans;
int numScans;

void scan_handler(std::string name,   // Topic name
        sensor_msgs::LaserScan* scan, // Message pointer
        ros::Time t,                  // Message timestamp
        ros::Time t_shift,            // Shifted time
        void* n)                      // Void pointer
{
    // Do something with the sensor_msgs::LaserScan
    if (numScans) {
        mxSetN(scans, numScans + 1);
        mxSetData(scans,
                mxRealloc(mxGetData(scans),
                (numScans+1)*NUMBER_OF_FIELDS*sizeof(mxArray*)) );
    }
    mxSetFieldByNumber(scans, numScans, 0, mxCreateDoubleScalar(scan->header.seq)); //timestamp
    mxSetFieldByNumber(scans, numScans, 1, mxCreateDoubleScalar(scan->header.stamp.toSec())); //timestamp
    mxSetFieldByNumber(scans, numScans, 2, mxCreateDoubleScalar(scan->angle_min)); //min_angle
    mxSetFieldByNumber(scans, numScans, 3, mxCreateDoubleScalar(scan->angle_max)); //max_angle
    mxSetFieldByNumber(scans, numScans, 4, mxCreateDoubleScalar(scan->angle_increment)); //angle_increment (resolution)
    mxSetFieldByNumber(scans, numScans, 5, mxCreateDoubleScalar(scan->range_min)); //min_range
    mxSetFieldByNumber(scans, numScans, 6, mxCreateDoubleScalar(scan->range_max)); //max_range
    mxArray *tmp = mxCreateDoubleMatrix(1, scan->ranges.size(), mxREAL);
    double *array = mxGetPr(tmp);
    for(int i = 0; i < scan->ranges.size(); i++)
        array[i] = scan->ranges[i];
    mxSetFieldByNumber(scans, numScans, 7, tmp); //ranges
    tmp = mxCreateDoubleMatrix(1, scan->intensities.size(), mxREAL);
    array = mxGetPr(tmp);
    for(int i = 0; i < scan->intensities.size(); i++)
        array[i] = scan->intensities[i];
    mxSetFieldByNumber(scans, numScans, 8, tmp); //intensities
    array = NULL;
    tmp = NULL;
    numScans++;
}

void mexFunction(
        int           nlhs,
        mxArray       *plhs[],
        int           nrhs,
        const mxArray *prhs[]
        ) {
    numScans = 0;
    if (nrhs != 1) {
        mexErrMsgTxt("One input argument is required; path to bag file.");
    }
    if(nlhs > 1){
        mexErrMsgTxt("Too many output arguments.");
    }
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("Input argument must be string.");
    }
    
    ros::record::Player player;
    
    char *path = mxArrayToString(prhs[0]);
    
    if (player.open(std::string(path), ros::Time())) {
        player.addHandler<sensor_msgs::LaserScan>(
                std::string("/scan"), // Topic name
                &scan_handler,        // Handler
                NULL);                // void*
    } else {
        mexErrMsgTxt("rosrecord::Player couldn't open bag file.");
    }
    
    plhs[0] = mxCreateStructMatrix(1, 1, NUMBER_OF_FIELDS, field_names);
    scans = plhs[0];
    
    mxFree(path);
    // Spin until we have consumed the bag
    while(player.nextMsg())  {}
    
    scans = NULL;
}
