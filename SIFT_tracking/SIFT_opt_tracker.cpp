#include "StdAfx.h"
#include "SIFT_opt_tracker.h"

SIFT_opt_tracker::SIFT_opt_tracker(void)
{
}

SIFT_opt_tracker::~SIFT_opt_tracker(void)
{
}

bool SIFT_opt_tracker::tracking(ImageHandler* imhdr,IplImage* preFrame,SIFT_feature *Sfeat, int Sfeat_num_fp,Rect *trackingWindow,Rect *trackingRect)
{
	int count=0;
	/*struct SIFT_feature_unit *feat_cmp;
	struct SIFT_feature_unit **nbrs;
	int k = 0;
	double d0,d1;*/
	Point2D MovingVector(0,0);
	Point2D TempMovingVector(0,0);
	double MovingVectorx,MovingVectory,TempMovingVectorx,TempMovingVectory;
	double MaxMove=0;
	double TempMoveScale = 0;
	double theta=0.0;
	double orghistogram[12];
	memset(orghistogram,0,12);
	for (int i = 0;i<Sfeat_num_fp;i++)
	{
		MovingVector = getOptFlow(imhdr->getIplImage(),Point2D(tracking_template->feat[i].y,tracking_template->feat[i].x),preFrame);
		optflow.push_back(MovingVector);
		orghistogram[(int)(atan((double)MovingVector.row/(double)MovingVector.col)/(PI/6.0))] =(sqrt(MovingVector.col*MovingVector.col+MovingVector.row*MovingVector.row));
		//feat_cmp = Sfeat->GetFeat(i);
		//k = kdtree_bbf_knn(kd_root,feat_cmp,2,&nbrs,200);
		//if (k==2)
		//{
		//	d0 = descr_dist_sq(feat_cmp,nbrs[0]);
		//	d1 = descr_dist_sq(feat_cmp,nbrs[1]);
		//	if(d0<d1 * 0.5)
		//	{
		//		//TempMovingVector = Point2D(feat_cmp->img_pt.x-nbrs[0]->img_pt.x,feat_cmp->img_pt.y-nbrs[0]->img_pt.y);
		//		TempMovingVectorx = feat_cmp->img_pt.x-(nbrs[0]->img_pt.x+(trackingRect->left-trackingWindow->left));
		//		TempMovingVectory = feat_cmp->img_pt.y-(nbrs[0]->img_pt.y+(trackingRect->upper-trackingWindow->upper));
		//		if ((abs(TempMovingVectory)<=trackingRect->width/5.0)&&(abs(TempMovingVectorx)<=trackingRect->height/5.0) )
		//		{
		//			TempMoveScale = sqrt((double)(TempMovingVectorx*TempMovingVectorx+TempMovingVectory*TempMovingVectory));
		//			if (TempMoveScale>MaxMove)
		//			{
		//				MovingVectorx = TempMovingVectorx;
		//				MovingVectory = TempMovingVectory;
		//				MaxMove = TempMoveScale;
		//			}
		//			feat_cmp->fwd_match = nbrs[0];
		//			imhdr->paintPoint(Point2D(nbrs[0]->y+trackingRect->upper,nbrs[0]->x+trackingRect->left),Color(0,255,0),3);
		//			imhdr->paintPoint(Point2D(feat_cmp->y+trackingWindow->upper,feat_cmp->x+trackingWindow->left),Color(255,0,0),3);
		//			count++;
		//		}
		//	}
		//}
		//free(nbrs);
	}
	double MaxDirection=0,MaxDirectionI;
	for (int i=0;i<12;i++)
	{
		if(orghistogram[i]>MaxDirection)
		{
			MaxDirection = orghistogram[i];
			MaxDirectionI = i;
		}
	}

	MovingVectorx = 
	if((double)count/(double)this->tracking_template->GetLength()<=0)
		return false;

	if (sqrt((double)(MovingVectorx*MovingVectorx+MovingVectory*MovingVectory))>TRACKING_PIXEL_THR)
	{
		MovingVector = Point2D(cvRound(MovingVectory),cvRound(MovingVectorx));
		*trackingRect = *trackingRect+MovingVector; 
	}

	imhdr->paintRectangle(*trackingRect);
	imhdr->paintRectangle(*trackingWindow);
	return true;
}