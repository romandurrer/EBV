/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT


const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;


int TextColor;
const int Border = 2;
const float avgFac = 0.95;
float bgrImg[IMG_SIZE];
const int frgLimit = 100;

int16 imgDx[IMG_SIZE];
int16 imgDy[IMG_SIZE];

struct OSC_VIS_REGIONS ImgRegions;
uint16 MinArea = 1000;

void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();
void AngleBox();
void ChangeDetection();



void ChangeDetection() {
	 int r, c;
	//set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);

	//loop over the rows (heigth)
	for(r = Border*nc; r < (nr-Border)*nc; r += nc)
	{
	//loop over the columns (width)
	for(c = Border; c < (nc-Border); c++)
	{
	unsigned char* p = &data.u8TempImage[SENSORIMG][r+c];

	/* implement Sobel filter in x-direction */
	int32 dx = -(int32) *(p-nc-1) + 0 + (int32) *(p-nc+1)
	-2* (int32) *(p-1) + 0 + 2* (int32) *(p+1)
	-(int32) *(p+nc-1) + 0 + (int32) *(p+nc+1);

	int32 dy = -(int32) *(p-nc-1) - 2* (int32) *(p-nc) - (int32) *(p-nc+1)
	+ 0 + 0 + 0	+ (int32) *(p+nc-1) + 2* (int32) *(p+nc) + (int32) *(p+nc+1);

	/* check if norm is larger than threshold */
	int32 df2 = dx*dx+dy*dy;
	int32 thr2 = data.ipc.state.nThreshold*data.ipc.state.nThreshold;
	if(df2 > thr2) //avoid square root
	{
	//set pixel value to 255 in THRESHOLD image for gui
	data.u8TempImage[THRESHOLD][r+c] = 255;
	}
	//store derivatives (int16 is enough)
	imgDx[r+c] = (int16) dx;
	imgDy[r+c] = (int16) dy;

	//possibility to visualize data
	data.u8TempImage[BACKGROUND][r+c] = (uint8) MAX(0, MIN(255,128+(dy+dx)/2));
	//Opening(); // Don't make opening here, because of the ammount of content switching
	}
	}
	}

void Erode_3x3(int InIndex, int OutIndex)
{
int c, r;
for(r = Border*nc; r < (nr-Border)*nc; r += nc) {
for(c = Border; c < (nc-Border); c++) {
unsigned char* p = &data.u8TempImage[InIndex][r+c];
data.u8TempImage[OutIndex][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
*(p-1) & *p & *(p+1) &
*(p+nc-1) & *(p+nc) & *(p+nc+1);
}
}
}

void Dilate_3x3(int InIndex, int OutIndex)
{
int c, r;
for(r = Border*nc; r < (nr-Border)*nc; r += nc) {
for(c = Border; c < (nc-Border); c++) {
unsigned char* p = &data.u8TempImage[InIndex][r+c];
data.u8TempImage[OutIndex][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
*(p-1) | *p | *(p+1) |
*(p+nc-1) | *(p+nc) | *(p+nc+1);
}
}
}

void ResetProcess()
{

}
void ProcessFrame()
{
	//char Text[] = "testat2";
	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {

		ChangeDetection();
		Erode_3x3(THRESHOLD, INDEX0);
		Dilate_3x3(INDEX0, THRESHOLD);
		DetectRegions();
		DrawBoundingBoxes();
		AngleBox();
		//DrawString(200, 200, strlen(Text), TINY, TextColor, Text);

	}
}

void DetectRegions() {
struct OSC_PICTURE Pic;
int i;
//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
for(i = 0; i < IMG_SIZE; i++) {
data.u8TempImage[INDEX0][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
}
//wrap image INDEX0 in picture struct
Pic.data = data.u8TempImage[INDEX0];
Pic.width = nc;
Pic.height = nr;
Pic.type = OSC_PICTURE_BINARY;
//now do region labeling and feature extraction
OscVisLabelBinary( &Pic, &ImgRegions);
OscVisGetRegionProperties( &ImgRegions);
}


void AngleBox()
{
char text[7];
double angle;
// Loop over all boxes
for(uint16_t o = 0; o < ImgRegions.noOfObjects; o++)
{// Loop only over big boxes
if(ImgRegions.objects[o].area > MinArea)
{
uint16_t result[4] = {0,0,0,0};
memcpy(text, "       ", 7);
struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;
while(currentRun != NULL)
{
for (uint16_t currentCol = currentRun->startColumn; currentCol <= currentRun->endColumn; currentCol++)
{
int r = currentRun->row;


angle = atan2(imgDy[r * nc + currentCol], imgDx[r * nc + currentCol]);


if(angle < 0)
angle += M_PI;
else if(angle > M_PI)
angle -= M_PI;

if ((angle <= (M_PI / 8)) && (angle >= 0))
result[0]++;
else if ((angle >= (1 * M_PI / 8)) && (angle <= (3 * M_PI / 8)))
result[1]++;
else if ((angle >= (3 * M_PI / 8)) && (angle <= (5 * M_PI / 8)))
result[2]++;
else if ((angle >= (5 * M_PI / 8)) && (angle <= (7 * M_PI / 8)))
result[3]++;
else if (angle <= M_PI)
result[0]++;
}
currentRun = currentRun->next;
}

uint16_t n = 0;
for(uint16 x=0; x<4; x++)
{

if(result[x] > result[n])
n = x;
}

// Show String for box
if(n == 0)
memcpy(text, "0 deg  ", 7);
else if(n == 1)
memcpy(text, "45 deg ", 7);
else if(n == 2)
memcpy(text, "90 deg ", 7);
else if(n == 3)
memcpy(text, "135 deg", 7);
DrawString(ImgRegions.objects[o].centroidX,ImgRegions.objects[o].centroidY,7,GIANT,BLUE,text);
}
}
}

void DrawBoundingBoxes() {
uint16 o;
for(o = 0; o < ImgRegions.noOfObjects; o++) {
if(ImgRegions.objects[o].area > MinArea) {
DrawBoundingBox(ImgRegions.objects[o].bboxLeft, ImgRegions.objects[o].bboxTop,
ImgRegions.objects[o].bboxRight, ImgRegions.objects[o].bboxBottom, false, GREEN);
}
}
}
