/*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the Willow Garage, Inc. nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/*
* This file contains helper functions for loading images as maps.
* 
* Author: Brian Gerkey
*/

#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>

// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

#include "image_loader.h"
#include "mapping.h"
#include <tf/tf.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace std
{

	void
		loadMapFromFile(dynamic_mapping::GetDynamicMap::Response* resp,
		const char* fname, double res, bool negate,
		double occ_th, double free_th, double* origin, uint32_t *dynamic_imp)
	{
		SDL_Surface* img;

		unsigned char* pixels;
		unsigned char* p;
		int rowstride, n_channels;
		unsigned int i,j;
		int k;
		double occ;
		int color_sum;
		double color_avg;

		int fname_size = strlen(fname);

		// Load the image using SDL.  If we get NULL back, the image load failed.
		if(!(img = IMG_Load(fname)))
		{
			std::string errmsg = std::string("failed to open image file \"") + 
				std::string(fname) + std::string("\"");
			throw std::runtime_error(errmsg);
		}

		// Copy the image data into the map structure
		resp->map.info.width = img->w;
		resp->map.info.height = img->h;
		resp->map.info.resolution = res;
		resp->map.info.origin.position.x = *(origin);
		resp->map.info.origin.position.y = *(origin+1);
		resp->map.info.origin.position.z = 0.0;
		tf::Quaternion q;
		q.setRPY(0,0, *(origin+2));
		resp->map.info.origin.orientation.x = q.x();
		resp->map.info.origin.orientation.y = q.y();
		resp->map.info.origin.orientation.z = q.z();
		resp->map.info.origin.orientation.w = q.w();

		// Allocate space to hold the data
		resp->map.data.resize(resp->map.info.width * resp->map.info.height);
		resp->map.dynamic.resize(resp->map.info.width * resp->map.info.height);

		// Get values that we'll need to iterate through the pixels
		rowstride = img->pitch;
		n_channels = img->format->BytesPerPixel;
		ROS_INFO("Bytes per Pixel: %d\n", n_channels);

		// Copy pixel data into the map structure
		pixels = (unsigned char*)(img->pixels);
		int dyn_area = 1;
		for(j = 0; j < resp->map.info.height; j++)
		{
			for (i = 0; i < resp->map.info.width; i++)
			{
				static unsigned char color[3];
				// Compute mean of RGB for this pixel
				p = pixels + j*rowstride + i*n_channels;
				color_sum = 0;
				// ppm file rgb
				if(n_channels == 3)	{
					color[0] = (int) *p;  	   //red
					color[1] = (int) *(p+1);  //green
					color[2] = (int) *(p+2);  //blue

					// blue pixel
					if(color[1] < color[2])	{					
						resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = -1;
						resp->map.dynamic[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = 0;
					}
					// red pixel
					else if(color[0] > color[1])	{
						resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = 100 - ((color[1]) * 5 * 100) / 255;
						resp->map.dynamic[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = dynamic_imp[resp->map.info.height*j + i];
					}
					// grey valued pixel
					else	{					
						resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = 100 - ((color[1]) * 100) / 255;
						resp->map.dynamic[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = dynamic_imp[resp->map.info.height*j + i];
					}

				}
				//pgm grey valued file
				else	{
						color[0] = (int) *p;  	   //grey
						
						if(color[0] == 254)
							resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = 0;
						else if(color[0] == 0)
							resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = -1;
						else
							resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = (color[0] - 100);
			
						resp->map.dynamic[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = 0;
				}
			}
		}

		SDL_FreeSurface(img);
	}

}
