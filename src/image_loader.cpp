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
#include <SDL2/SDL_image.h>

// Use Bullet's Quaternion object to create one from Euler angles
#include <LinearMath/btQuaternion.h>

#include "multimap_server/image_loader.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace multimap_server
{
  void loadMapFromFile(nav_msgs::GetMap::Response *resp, const char *fname, double res, bool negate, double occ_th,
                       double free_th, double *origin, MapMode mode)
  {

    // unsigned char* pixels;
    // unsigned char* p;
    // unsigned char value;
    // int rowstride, n_channels, avg_channels;
    // unsigned int i, j;
    // int k;
    // double occ;
    // int alpha;
    // int color_sum;
    // double color_avg;

    //  std::cout <<"Loading image "<< fname<< std::endl;
    SDL_Surface *raw_image = IMG_Load(fname);
    //  std::cout<<"Image \""<< fname<< "\" loaded"<< std::endl;
    if (!raw_image)
    {
      std::string errmsg =
          std::string("failed to open image file \"") + std::string(fname) + std::string("\": ") + IMG_GetError();
      throw std::runtime_error(errmsg);
    }

    std::unique_ptr<SDL_Surface, void (*)(SDL_Surface *)> img(raw_image, &SDL_FreeSurface);

    //  std::cout << "Image \"" << fname << "\" converted" << std::endl;
    const int width = img->w;
    const int height = img->h;
    //  std::cout << "Image \"" << fname << "\" is " << width << " x " << height << std::endl;

    // Copy the image data into the map structure
    resp->map.info.width = width;
    resp->map.info.height = height;
    resp->map.info.resolution = res;
    resp->map.info.origin.position.x = *(origin);
    resp->map.info.origin.position.y = *(origin + 1);
    resp->map.info.origin.position.z = 0.0;
    btQuaternion q;
    // setEulerZYX(yaw, pitch, roll)
    q.setEulerZYX(*(origin + 2), 0, 0);
    resp->map.info.origin.orientation.x = q.x();
    resp->map.info.origin.orientation.y = q.y();
    resp->map.info.origin.orientation.z = q.z();
    resp->map.info.origin.orientation.w = q.w();

    // const varables for easier access in loop
    const int rowstride = img->pitch;
    const int n_channels = img->format->BytesPerPixel;
    const bool has_alpha = img->format->Amask != 0;
    const int avg_channels = (mode == TRINARY || !has_alpha) ? n_channels : (n_channels - 1);

    const uint8_t *pixels = static_cast<const uint8_t *>(img->pixels);

    //  std::cout << "Image \"" << fname << "\": " << n_channels << " channels, " << (has_alpha ? "has" : "no") << " alpha" << std::endl;
    std::vector<int8_t> local_data;
    local_data.resize(static_cast<size_t>(width) * static_cast<size_t>(height));

    //  std::cout << "Image \"" << fname << "\": Processing map data (" << (negate ? "" : "un") << "negate, occ_th=" << occ_th << ", free_th=" << free_th << ")" << std::endl;
    
    for (int j = 0; j < height; ++j)
    {
      const uint8_t *scanline = pixels + j * rowstride;
      const int dest_row = height - j - 1;
      int8_t *dest_ptr = reinterpret_cast<int8_t *>(local_data.data() + static_cast<size_t>(dest_row) * static_cast<size_t>(width));

      for (int i = 0; i < width; ++i)
      {
        const uint8_t *p = scanline + i * n_channels;

        int color_sum = 0;
        for (int k = 0; k < avg_channels; ++k)
          color_sum += p[k];
        const double color_avg = static_cast<double>(color_sum) / static_cast<double>(avg_channels);

        const int alpha = (n_channels == 1) ? 255 : static_cast<int>(p[n_channels - 1]);

        double adjusted = color_avg;
        if (negate)
          adjusted = 255.0 - adjusted;

        if (mode == RAW)
        {
          // RAW: store raw averaged grayscale (clamped to [0,255])
          int raw_val = static_cast<int>(std::round(adjusted));
          raw_val = std::min(255, std::max(0, raw_val));
          dest_ptr[i] = static_cast<int8_t>(raw_val);
          continue;
        }

        const double occ = (255.0 - adjusted) / 255.0;

        int val;
        if (occ > occ_th)
          val = 100;
        else if (occ < free_th)
          val = 0;
        else if (mode == TRINARY || alpha < 1)
          val = -1;
        else
        {
          const double ratio = (occ - free_th) / (occ_th - free_th);
          val = static_cast<int>(std::lround(99.0 * ratio));
          val = std::min(99, std::max(0, val));
        }

        dest_ptr[i] = static_cast<int8_t>(val);
      }
    }
    // std::cout << "Image \"" << fname << "\": Map data processing complete" << std::endl;
    resp->map.data = std::move(local_data);
    // std::cout << "Image \"" << fname << "\": Map data copied" << std::endl;
    // SDL_FreeSurface(raw_image);
    // std::cout << "Image \"" << fname << "\": Image memory freed" << std::endl;
  }
}
