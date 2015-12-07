
/***********************************************************************************************************************
FILENAME:   openni2_pcl.cpp
AUTHORS:    Aditya Kadur
Based on Examples by Christopher D. McMurrough

DESCRIPTION:
Provides an example of using OpenNI2Grabber to assemble point clouds from depth and rgb images

***********************************************************************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "OpenNI2Grabber.h"
#include <iostream>
#include <SDL.h>
#include <SDL_image.h>

#include <string>

using namespace cv;

//Screen attributes
const int SCREEN_WIDTH  = 640;
const int SCREEN_HEIGHT = 480;
//We'll be scaling our tiles to be 40x40
const int TILE_SIZE = 40;

/*
 * Log an SDL error with some error message to the output stream of our choice
 * @param os The output stream to write the message too
 * @param msg The error message to write, format will be msg error: SDL_GetError()
 */
void logSDLError(std::ostream &os, const std::string &msg){
	os << msg << " error: " << SDL_GetError() << std::endl;
}
/*
 * Loads an image into a texture on the rendering device
 * @param file The image file to load
 * @param ren The renderer to load the texture onto
 * @return the loaded texture, or NULL if something went wrong.
 */
SDL_Texture* loadTexture(const std::string &file, SDL_Renderer *ren){
	SDL_Texture *texture = IMG_LoadTexture(ren, file.c_str());
	if (texture == NULL){
		logSDLError(std::cout, "LoadTexture");
	}
	return texture;
}
/*
 * Draw an SDL_Texture to an SDL_Renderer at position x, y, with some desired
 * width and height
 * @param tex The source texture we want to draw
 * @param rend The renderer we want to draw too
 * @param x The x coordinate to draw too
 * @param y The y coordinate to draw too
 * @param w The width of the texture to draw
 * @param h The height of the texture to draw
 */
void renderTexture(SDL_Texture *tex, SDL_Renderer *ren, int x, int y, int w, int h){
	//Setup the destination rectangle to be at the position we want
	SDL_Rect dst;
	dst.x = x;
	dst.y = y;
	dst.w = w;
	dst.h = h;
	SDL_RenderCopy(ren, tex, NULL, &dst);
}
/*
 * Draw an SDL_Texture to an SDL_Renderer at position x, y, preserving
 * the texture's width and height
 * @param tex The source texture we want to draw
 * @param rend The renderer we want to draw too
 * @param x The x coordinate to draw too
 * @param y The y coordinate to draw too
 */
void renderTexture(SDL_Texture *tex, SDL_Renderer *ren, int x, int y){
	int w, h;
	SDL_QueryTexture(tex, NULL, NULL, &w, &h);
	renderTexture(tex, ren, x, y, w, h);
}

int main(int, char**){

/***************** Setup OpenNI **********************************/

    OpenNI2Grabber grabber;
    bool isRunning = false;
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

/*********** Setup OpenCV *******************/
    Mat depthImage, colorImage;
    Mat depthImageDraw;

/*************Setup PCL ********************/

	//Start up SDL and make sure it went ok
	if (SDL_Init(SDL_INIT_VIDEO) != 0){
		logSDLError(std::cout, "SDL_Init");
		return 1;
	}
	std::cout<<"1\n";
	//Setup our window and renderer
	SDL_Window *window = SDL_CreateWindow("Lesson 3", 100, 100, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
	if (window == NULL){
		logSDLError(std::cout, "CreateWindow");
		SDL_Quit();
		return 1;
	}
	SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (renderer == NULL){
		logSDLError(std::cout, "CreateRenderer");

		SDL_Quit();
		return 1;
	}

/********************************************/

	//The textures we'll be using
	bool firstframe = true;
	SDL_Texture *background = loadTexture("green.jpg", renderer);
	SDL_Texture *image = loadTexture("mudTile.jpg", renderer);
	SDL_Texture *flowers = loadTexture("unnamed.png", renderer);
	//Make sure they both loaded ok
	if (background == NULL || image == NULL){
		SDL_Quit();
		return 1;
	}

	SDL_Event e;
	//A sleepy rendering loop, wait for 3 seconds and render and present the screen each time
	while(isRunning){

		// acquire an image frame
        if(grabber.waitForFrame(1000))
        {
            // display the acquired images
            if(grabber.getDepthFrame(depthImage))
            {
                // multiply the 11-bit depth values by 32 to extend the color range to a full 16-bits
                depthImage.convertTo(depthImageDraw, -1, 32);
                pyrDown(depthImageDraw, depthImageDraw);
                pyrDown(depthImageDraw, depthImageDraw);
                pyrDown(depthImageDraw, depthImageDraw);
                pyrDown(depthImage, depthImage);
                pyrDown(depthImage, depthImage);
                pyrDown(depthImage, depthImage);
                cv::imshow("depth", depthImageDraw);
                std::cout << depthImage.at<unsigned short>(40,30) <<std::endl;
            }
        }

        // check for program termination
        char key = (char) cv::waitKey(1);
        if(key == 'q' || key == 'Q' || key == 27)
        {
            std::cout << "Terminating program..." << std::endl;
            isRunning = false;
        }

		//Clear the window
		SDL_RenderClear(renderer);

		//Determine how many tiles we'll need to fill the screen
		int xTiles = SCREEN_WIDTH / TILE_SIZE;
		int yTiles = SCREEN_HEIGHT / TILE_SIZE;

		//Draw the tiles by calculating their positions
		// for (int i = 0; i < xTiles * yTiles; ++i){
		// 	int x = i % xTiles;
		// 	int y = i / xTiles;
		// 	renderTexture(background, renderer, x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE);
		// }
		renderTexture(background, renderer, 0,0, 640, 480);
		//Draw our image in the center of the window
		//We need the foreground image's width to properly compute the position
		//of it's top left corner so that the image will be centered
		int iW, iH;
		SDL_QueryTexture(image, NULL, NULL, &iW, &iH);
		for(int x=0; x<640/8; x++)
			for(int y=0; y<480/8; y++){
				if(depthImage.at<unsigned short>(y,x) > 500 && depthImage.at<unsigned short>(y,x) < 900)
					renderTexture(image, renderer, x*8, y*8);
			}
		// int x = SCREEN_WIDTH / 2 - iW / 2;
		// int y = SCREEN_HEIGHT / 2 - iH / 2;
		// renderTexture(image, renderer, x, y);

		SDL_QueryTexture(flowers, NULL, NULL, &iW, &iH);
		int x = SCREEN_WIDTH / 2 - iW / 2;
		int y = SCREEN_HEIGHT / 2 - iH / 2;
		SDL_Rect dst;
		dst.x = x;
		dst.y = y;
		dst.w = iW/2;
		dst.h = iH;
		SDL_Rect clip;
		clip.x = (firstframe? 0:iW/2);;
		clip.y = 0;
		clip.w = iW/2;
		clip.h = iH;
		SDL_RenderCopy(renderer, flowers, &clip, &dst);
		firstframe =!firstframe;
		//Update the screen
		SDL_RenderPresent(renderer);
		//For exit - Handle events on queue
		while( SDL_PollEvent( &e ) != 0 )
		{
			//User requests quit
			if( e.type == SDL_QUIT )
			{
				isRunning = false;
			}
			//Handle key presses
			else if( e.type == SDL_KEYDOWN )
			{
				//Increase alpha on w
				if( e.key.keysym.sym == SDLK_q )
					isRunning = false;			
			}
		}
	}

	SDL_DestroyTexture(image);
	grabber.stop();	
	IMG_Quit();
	SDL_Quit();

	return 0;
}
