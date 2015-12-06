//
//    Copyright 2013 Christopher D. McMurrough
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

/***********************************************************************************************************************
FILENAME:   openni2_pcl.cpp
AUTHORS:    Christopher D. McMurrough

DESCRIPTION:
Provides an example of using OpenNI2Grabber to assemble point clouds from depth and rgb images

REVISION HISTORY:
08.12.2013  CDM     original file creation
09.01.2013  CDM     published under GPL
***********************************************************************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "OpenNI2Grabber.h"
#include <iostream>
#include <SDL.h>
#include <SDL_image.h>

using namespace cv;

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point
***********************************************************************************************************************/
int main(int argc, char** argv)
{
    // create the cloud viewer object
    pcl::visualization::CloudViewer viewer("Point Cloud");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

        if (SDL_Init(SDL_INIT_VIDEO) != 0){
		std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
		return 1;
	}
	
	//Now create a window with title "Hello World" at 100, 100 on the screen with w:640 h:480 and show it
	SDL_Window *win = SDL_CreateWindow("Hello World!", 100, 100, 640, 480, SDL_WINDOW_SHOWN | SDL_WINDOW_FULLSCREEN_DESKTOP);
	//Make sure creating our window went ok
	if (win == NULL){
		std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
		return 1;
	}

        
	//Create a renderer that will draw to the window, -1 specifies that we want to load whichever
	//video driver supports the flags we're passing
	//Flags: SDL_RENDERER_ACCELERATED: We want to use hardware accelerated rendering
	//SDL_RENDERER_PRESENTVSYNC: We want the renderer's present function (update screen) to be
	//synchornized with the monitor's refresh rate
	SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (ren == NULL){
		SDL_DestroyWindow(win);
		std::cout << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
		SDL_Quit();
		return 1;
	}

	//SDL 2.0 now uses textures to draw things but SDL_LoadBMP returns a surface
	//this lets us choose when to upload or remove textures from the GPU
	std::string imagePath = "grass2.jpg";
	SDL_Surface *bmp = IMG_Load(imagePath.c_str());
	if (bmp == NULL){
		SDL_DestroyRenderer(ren);
		SDL_DestroyWindow(win);
		std::cout << "SDL_LoadJPG Error: " << SDL_GetError() << std::endl;
		SDL_Quit();
		return 1;
	}

	//To use a hardware accelerated texture for rendering we can create one from
	//the surface we loaded
	SDL_Texture *tex = SDL_CreateTextureFromSurface(ren, bmp);
	//We no longer need the surface
	SDL_FreeSurface(bmp);
	if (tex == NULL){
		SDL_DestroyRenderer(ren);
		SDL_DestroyWindow(win);
		std::cout << "SDL_CreateTextureFromSurface Error: " << SDL_GetError() << std::endl;
		SDL_Quit();
		return 1;
	}
	
	//A sleepy rendering loop, wait for 3 seconds and render and present the screen each time
	for (int i = 0; i < 3; ++i){
		//First clear the renderer
		SDL_RenderClear(ren);
		//Draw the texture
		SDL_RenderCopy(ren, tex, NULL, NULL);
		//Update the screen
		SDL_RenderPresent(ren);
		//Take a quick break after all that hard work
		SDL_Delay(3000);
	}
	//Clean up our objects and quit
	// SDL_Delay(3000);

	SDL_DestroyRenderer(ren);
	SDL_DestroyWindow(win);
    bool isRunning = false;
    OpenNI2Grabber grabber;

    // attempt to start the grabber
    if(grabber.initialize(true, false, true, false))
    {
        if(grabber.start())
        {
            std::cout<<"Grabber is running...\n";
            isRunning = grabber.isRunning();
        }
    }
    else
    {
        std::cout << "Unable to initialize OpenNI2Grabber, program terminating!" << std::endl;
        return 0;
    }

    // acquire frames until program termination
    std::cout << "Press 'q' to halt acquisition..." << std::endl;
    Mat depthImage, colorImage;
    Mat depthImageDraw;
    Mat disparityImage;
    Mat depthImageMeters;
    while(isRunning)
    {
        // acquire an image frame
        if(grabber.waitForFrame(1000))
        {
            // display the acquired images
            if(grabber.getDepthFrame(depthImage))
            {
                // multiply the 11-bit depth values by 32 to extend the color range to a full 16-bits
                depthImage.convertTo(depthImageDraw, -1, 32);
                cv::imshow("depth", depthImageDraw);

            }
            if(grabber.getColorFrame(colorImage))
            {
                cv::imshow("rgb", colorImage);
            }
        }
        else
        {
        }

        // check for program termination
        char key = (char) cv::waitKey(1);
        if(key == 'q' || key == 'Q' || key == 27)
        {
            std::cout << "Terminating program..." << std::endl;
            isRunning = false;
        }

        // render the point cloud
        if(key == 'p' || key == 'P')
        {
            //grabber.makeCloud(depthImageMeters, colorImage, cloud);
            grabber.makeCloud(depthImage, colorImage, cloud);
            std::cout << "rendering point cloud with " << cloud->size() << " points..." << std::endl;
            viewer.showCloud(cloud);
        }
        
        //Test stuff
        if(key == 's')
        {
            std::cout << depthImage << std::endl;
        }

    }

    grabber.stop();

    return 0;
}
