#pragma once
#include "SIFT_feature.h"

class SIFTBoostingTracker
{
public:
	SIFTBoostingTracker(void);
	~SIFTBoostingTracker(void);
	SIFTBoostingTracker( SIFT_feature* imageFeature, IplImage* curFrame,Rect initPatch, Rect validROI, int numBaseClassifier);
	//virtual ~SIFTBoostingTracker();

	bool track(SIFT_feature* imageFeature, Patches* patches);

	Rect getTrackingROI(float searchFactor);
	//float getConfidence();
	Rect getTrackedPatch();
	Point2D getCenter();

private:
	/*StrongClassifier* classifier;
	Detector* detector;*/
	Rect validROI;
	Rect trackedPatch;
	float confidence;
	//vector<feature> siftFeaturePoint;
	//Point2D dxDy;
};