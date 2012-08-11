#include "stdafx.h"
#include "SIFTBoostingTracker.h"

//#include "imgfeatures.h"
//#include "sift.h"
//#include "sift.c"
#include "utils.h"

SIFTBoostingTracker::SIFTBoostingTracker(void)
{
}

SIFTBoostingTracker::~SIFTBoostingTracker(void)
{
}

SIFTBoostingTracker::SIFTBoostingTracker(SIFT_feature* imageFeature,IplImage* curFrame, Rect initPatch, Rect validROI, int numBaseClassifier)
{
	int numWeakClassifier = numBaseClassifier*5;
	bool useFeatureExchange = true;
	int iterationInit = 50;
	/*Size patchSize;
	patchSize = initPatch;*/

	this->validROI = validROI;

	//struct feature* feat;
	
	//int intvls = SIFT_INTVLS;
	//double sigma = SIFT_SIGMA;
	//double contr_thr = SIFT_CONTR_THR;
	//int curv_thr = SIFT_CURV_THR;
	//int img_dbl = SIFT_IMG_DBL;
	//int descr_width = SIFT_DESCR_WIDTH;
	//int descr_hist_bins = SIFT_DESCR_HIST_BINS;


	/*int n = _sift_features( curFrame, &feat, intvls, sigma, contr_thr, curv_thr,
		img_dbl, descr_width, descr_hist_bins );*/

	/*draw_features( curFrame, feat, n );*/
	imageFeature->draw_features(curFrame);
	//cvNamedWindow( "SIFT tracking", 1 );
	//cvShowImage( "SIFT tracking", curFrame );
	

	/*classifier = new StrongClassifierDirectSelection(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit);

	detector = new Detector (classifier);

	trackedPatch = initPatch;
	Rect trackingROI = getTrackingROI(2.0f);
	Size trackedPatchSize;
	trackedPatchSize = trackedPatch;
	Patches* trackingPatches = new PatchesRegularScan(trackingROI, validROI, trackedPatchSize, 0.99f);

	iterationInit = 50;
	for (int curInitStep = 0; curInitStep < iterationInit; curInitStep++)
	{
		printf ("\rinit tracker... %3.0f %% ", ((float)curInitStep)/(iterationInit-1)*100);	

		classifier->update (image, trackingPatches->getSpecialRect ("UpperLeft"), -1);
		classifier->update (image, trackedPatch, 1);
		classifier->update (image, trackingPatches->getSpecialRect ("UpperRight"), -1);
		classifier->update (image, trackedPatch, 1);
		classifier->update (image, trackingPatches->getSpecialRect ("LowerLeft"), -1);
		classifier->update (image, trackedPatch, 1);
		classifier->update (image, trackingPatches->getSpecialRect ("LowerRight"), -1);
		classifier->update (image, trackedPatch, 1);
	}

	confidence = -1;
	delete trackingPatches;*/

}

//SIFTBoostingTracker::~SIFTBoostingTracker(void)
//{
//	delete detector;
//	delete classifier;
//}

//bool SIFTBoostingTracker::track(ImageRepresentation* image, Patches* patches)
//{
//	detector->classify (image, patches);
//	detector->classifySmooth (image, patches);
//
//	move to best detection
//	if (detector->getNumDetections() <=0)
//	{
//		confidence = 0;
//		return false;
//	}
//
//	trackedPatch = patches->getRect (detector->getPatchIdxOfBestDetection ());
//	confidence  = detector->getConfidenceOfBestDetection ();
//
//	classifier->update (image, patches->getSpecialRect ("UpperLeft"), -1);
//	classifier->update (image, trackedPatch, 1);
//	classifier->update (image, patches->getSpecialRect ("UpperRight"), -1);
//	classifier->update (image, trackedPatch, 1);
//	classifier->update (image, patches->getSpecialRect ("UpperLeft"), -1);
//	classifier->update (image, trackedPatch, 1);
//	classifier->update (image, patches->getSpecialRect ("LowerRight"), -1);
//	classifier->update (image, trackedPatch, 1);
//
//	return true;
//}

Rect SIFTBoostingTracker::getTrackingROI(float searchFactor)
{
	Rect searchRegion;

	searchRegion = trackedPatch*(searchFactor);
	//check
	if (searchRegion.upper+searchRegion.height > validROI.height)
		searchRegion.height = validROI.height-searchRegion.upper;
	if (searchRegion.left+searchRegion.width > validROI.width)
		searchRegion.width = validROI.width-searchRegion.left;

	return searchRegion;
}

//float SIFTBoostingTracker::getConfidence()
//{
//	return confidence/classifier->getSumAlpha();
//}

Rect SIFTBoostingTracker::getTrackedPatch()
{
	return trackedPatch;
}

Point2D SIFTBoostingTracker::getCenter()
{
	Point2D center;
	center.row = trackedPatch.upper + trackedPatch.height/2 ;
	center.col =  trackedPatch.left +trackedPatch.width/2 ;
	return center;
}
