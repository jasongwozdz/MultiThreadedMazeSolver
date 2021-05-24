#pragma once

#include "dataTypes.h"

#include <GL/freeglut.h>
#include <GL/glut.h>

//	This draws a colored multi-segment traveler
void drawTraveler(const Traveler& traveler);

//	Defined in main.c
void speedupTravelers(void);
void slowdownTravelers(void);
void drawTravelers(void);
void updateMessages(void);

void drawGrid(void);
void drawMessages(int numMessages, char** message);
void handleKeyboardEvent(unsigned char c, int x, int y);

void initializeFrontEnd(int argc, char** argv);
float** createTravelerColors(unsigned int numTravelers);

