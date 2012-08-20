// SIFT_tracking.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

void on_mouse( int event, int x, int y, int flags, void* param );
void track(ImageSource::InputDevice input, int numBaseClassifier, int searchFactor, char* resultDir, Rect initBB, char* source = NULL);
int mouse_pointX;
int mouse_pointY;
int mouse_value;
bool mouse_exit;
Rect* trackingRect = new Rect;

int _tmain()
{
	cout<<"//////////////////////////////////////////////////////////////////////////"<<endl;
    cout<<"online boosting with SIFT"<<endl;
	cout<<"//////////////////////////////////////////////////////////////////////////"<<endl;

	ImageSource::InputDevice input;
	input = ImageSource::AVI;

	int numBaseClassifier=0;
	int searchFactor=2;                  //search window
	char* source;
	char* resultDir;
	Rect initBB;

	resultDir = new char[100];
    memset(resultDir,'\0',100);
	source=new char[100];
	memset( source, '\0', 100 );
	initBB=Rect(0,0,0,0);
	
	resultDir = "..\\result\\";
	//source = "F:\\testData\\1.avi";
	source = "E:\\MachineVision\\testData\\awt\\2.avi";
	track(input,numBaseClassifier,searchFactor,resultDir,initBB,source);
	delete trackingRect;
	system("PAUSE");
	return 0;
}

void on_mouse( int event, int x, int y, int flags, void* param )
{
	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		trackingRect->upper = y;
		trackingRect->left = x;
		break;
	case CV_EVENT_LBUTTONUP:
		trackingRect->height = y-trackingRect->upper;
		trackingRect->width = x-trackingRect->left;
		break;
	case CV_EVENT_MOUSEMOVE:

		if (flags == CV_EVENT_FLAG_LBUTTON)
		{
			trackingRect->height = y-trackingRect->upper;
			trackingRect->width = x-trackingRect->left;
		}
		break;
	}
}

void track(ImageSource::InputDevice input, int numBaseClassifier, int searchFactor, char* resultDir, Rect initBB, char* source)
{
	IplImage* curFrame=NULL;
	int key;
	//choose the image source
	ImageSource *imageSequenceSource;
	switch (input)
	{
	case ImageSource::AVI:
		imageSequenceSource = new ImageSourceAVIFile(source);
		break;
	case ImageSource::DIRECTORY:
		imageSequenceSource = new ImageSourceDir(source);
		break;
	case ImageSource::USB:
		imageSequenceSource = new ImageSourceUSBCam();
		break;
	default:
		return;
	}

	ImageHandler* imageSequence = new ImageHandler (imageSequenceSource);
	imageSequence->getImage();

	imageSequence->viewImage ("Tracking...", false);
	cvSetMouseCallback( "Tracking...", on_mouse, 0 );

	printf("1.) Select bounding box to initialize tracker.\n");
	printf("2.) Switch to DOS-window and press enter when done.\n");

	trackingRect->upper = -1;
	trackingRect->left = -1;
	trackingRect->width = -1;
	trackingRect->height = -1;
	
	if (initBB.width==0 && initBB.height==0) {
		//mark the object
#if OS_type==2

		while (!kbhit())		
		{			
			if (input == ImageSource::AVI || input == ImageSource::DIRECTORY)
				imageSequence->reloadImage();
			else
				imageSequence->getImage();

			if (trackingRect->height != -1)
			{
				imageSequence->paintRectangle (*trackingRect, Color (255,255,0), 2);
			}
			imageSequence->viewImage ("Tracking...", false);
			cvWaitKey(25);
		}
		getchar();
#endif

#if OS_type==1
		while (!keyboard_pressed)
		{
			if (input == ImageSource::AVI || input == ImageSource::DIRECTORY)
				imageSequence->reloadImage();
			else
				imageSequence->getImage();

			if (trackingRect.height != -1)
			{
				imageSequence->paintRectangle (trackingRect, Color (255,255,0), 2);
			}
			imageSequence->viewImage ("Tracking...", false);
			key = cvWaitKey(25);
			if((char)key !=-1) keyboard_pressed = true;
		}

#endif


		if (trackingRect->height == -1) return;

	} 
	else {
		trackingRect=&initBB;
	}
	curFrame = imageSequence->getIplImage();
	//SIFT_feature* curFrameRep = new SIFT_feature(curFrame,trackingRect); 
	Rect wholeImage;
	wholeImage = imageSequence->getImageSize();

	cout<<"init tracker...";
	//SIFTBoostingTracker* tracker;
	//tracker = new SIFTBoostingTracker (curFrameRep,imageSequence->getIplGrayImage(), trackingRect, wholeImage, numBaseClassifier);
	
	SIFT_feature* trackingTemplateRep = new SIFT_feature(curFrame,*trackingRect); 
	SIFT_navie_tracker *tracker;
	tracker = new SIFT_navie_tracker(trackingTemplateRep,trackingTemplateRep->GetLength(),*trackingRect);
	cout<<" done"<<endl;

	Size trackingRectSize;
	trackingRectSize = *trackingRect;
	cout<<"start tracking (stop by pressing any key)..."<<endl;

	FILE* resultStream;
	if (resultDir[0]!=0)
	{
		char *myBuff = new char[255];
		sprintf_s(myBuff, 255,"%s/SIFTTracker.txt", resultDir);
		resultStream = fopen(myBuff, "w");
		delete[] myBuff;
	}

	int counter= 0;
	IplImage* tracktemplate;
	tracktemplate = cvCreateImage(cvSize(trackingRect->width,trackingRect->height),
		curFrame->depth,
		curFrame->nChannels);
	bool trackerLost = false;
//	ConvertImage(curFrame,tracktemplate,trackingRect);
//	curFrameRep->draw_features(imageSequence,trackingRect);
//	ConvertImage(tracktemplate,curFrame,trackingRect);
	imageSequence->viewImage("Tracking...",false);
	//cvWaitKey(0);
//	key=(char)-1;
//	//tracking loop
//	while (key==(char)-1)
//	{
//		clock_t timeWatch;
//		timeWatch = clock();
//
//		//do tracking
//		counter++;
//
//		imageSequence->getImage();
//		if (curFrame!=NULL)
//			delete[] curFrame;
//
//		curFrame = imageSequence->getGrayImage ();
//		if (curFrame == NULL)
//			break;
//
//		//calculate the patches within the search region
//		//Patches *trackingPatches;	
//		Rect searchRegion;
//		//searchRegion = tracker->getTrackingROI(searchFactor);
//		//trackingPatches = new PatchesRegularScan(searchRegion, wholeImage, trackingRectSize, overlap);
//
//		curFrameRep->setNewImageAndROI(curFrame, searchRegion);
//
//		
//		/*if (!tracker->track(curFrameRep, trackingPatches))
//		{
//			trackerLost = true;
//			break;
//		}*/
//
//		//delete trackingPatches;
//
//		//display results
//		imageSequence->paintRectangle (tracker->getTrackedPatch(), Color (255,255,0), 2);
//		imageSequence->viewImage ("Tracking...", false);
//
//		//write images
//		if (resultDir[0]!=0)
//		{
//			/*if (trackerLost)
//				fprintf (resultStream, "%8d 0 0 0 0 -1\n", counter);
//			else
//				fprintf (resultStream, "%8d %3d %3d %3d %3d %5.3f\n", counter, tracker->getTrackedPatch().left, tracker->getTrackedPatch().upper, tracker->getTrackedPatch().width, tracker->getTrackedPatch().height, tracker->getConfidence());*/
//
//			char *myBuff = new char[255];
//			sprintf (myBuff, "%s/frame%08d.jpg", resultDir, counter+2);
//			imageSequence->saveImage(myBuff);
//		}
//
//		// wait for opencv display
//		key=cvWaitKey(25);
//
//#if OS_type==2
//
//		if (kbhit())
//			key=(char)0;
//#endif
//		timeWatch=clock()-timeWatch;
//		double framesPerSecond=1000.0/timeWatch;
//		//printf("TRACKING: confidence: %5.3f  fps: %5.2f   \r", tracker->getConfidence(), framesPerSecond);
//
//
//	}
//
//	printf ("\n\ntracking stopped\n");
//	if (trackerLost) 
//	{
//		printf ("tracker lost\n");
//#if OS_type==2
//		getch();
//#endif
//#if OS_type==1
//		std::cin.get();
//#endif
//
//	}
	
	/*if (resultDir[0]!=0)
		fclose(resultStream);*/

	//clean up
	key=(char)-1;
	SIFT_feature *curFrameRep;
	Rect TrackingWindow;
	/* tracking window initialized */
	ModifyTrackingWindows(*trackingRect,&TrackingWindow,wholeImage);
	/*TrackingWindow.upper = trackingRect.upper-cvRound(TRACKING_WINDOW_SIZE*trackingRect.height);
	TrackingWindow.left = trackingRect.left-cvRound(TRACKING_WINDOW_SIZE*trackingRect.width);
	TrackingWindow.width = trackingRect.width+cvRound(TRACKING_WINDOW_SIZE*trackingRect.width);
	TrackingWindow.height = trackingRect.height+cvRound(TRACKING_WINDOW_SIZE*trackingRect.height);*/
	
	IplImage* preFrame;

	//tracking loop
	while (key == (char)-1)
	{
		clock_t timewatch;
		timewatch = clock();
		
		/*if (curFrame!=NULL)
		{
			delete[] curFrame;
		}*/
		preFrame = (IplImage*)imageSequence->getIplImage();
		imageSequence->getImage();
		curFrame = (IplImage*)imageSequence->getIplImage();
		curFrameRep = new SIFT_feature(curFrame,TrackingWindow);

		if (curFrame == NULL)
		{
			break;
		}
		
		if (!tracker->tracking(imageSequence,curFrameRep,curFrameRep->GetLength(),&TrackingWindow,trackingRect))
		{
			cout<<"tracking lost!!!"<<endl;
			break;
		}
		/*curFrameRep->draw_features(imageSequence,trackingRect);
		trackingTemplateRep->draw_features(imageSequence,trackingRect);*/
		ModifyTrackingWindows(*trackingRect,&TrackingWindow,wholeImage);
		imageSequence->viewImage("Tracking...",false);
		if (resultDir[0]!=0)
		{
						/*if (trackerLost)
							fprintf (resultStream, "%8d 0 0 0 0 -1\n", counter);
						else
							fprintf (resultStream, "%8d %3d %3d %3d %3d %5.3f\n", counter, tracker->getTrackedPatch().left, tracker->getTrackedPatch().upper, tracker->getTrackedPatch().width, tracker->getTrackedPatch().height, tracker->getConfidence());*/
			
				char *myBuff = new char[255];
				int flag = sprintf (myBuff, "%sframe%08d.jpg", resultDir, counter+2);
				imageSequence->saveImage(myBuff);
		}
		counter++;
		cvWaitKey(20);
		if (curFrameRep)
		{
			delete curFrameRep;
		}
	}
	//delete tracker;
	delete imageSequenceSource;
	delete imageSequence;
	if (curFrame == NULL)
		delete[] curFrame;
	//delete curFrameRep;
}