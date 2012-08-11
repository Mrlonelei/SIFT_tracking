#include "StdAfx.h"
#include "SIFT_navie_tracker.h"

SIFT_navie_tracker::SIFT_navie_tracker(void)
{
}

SIFT_navie_tracker::~SIFT_navie_tracker(void)
{
}
bool SIFT_navie_tracker::tracking(ImageHandler* imhdr,SIFT_feature *Sfeat, int Sfeat_num_fp,Rect *trackingWindow,Rect *trackingRect)
{
	int count=0;
	struct SIFT_feature_unit *feat_cmp;
	struct SIFT_feature_unit **nbrs;
	int k = 0;
	double d0,d1;
	Point2D MovingVector(0,0);
	Point2D TempMovingVector(0,0);
	double MovingVectorx,MovingVectory,TempMovingVectorx,TempMovingVectory;
	double MaxMove=0;
	double TempMoveScale = 0;
	for (int i = 0;i<Sfeat_num_fp;i++)
	{
		feat_cmp = Sfeat->GetFeat(i);
		k = kdtree_bbf_knn(kd_root,feat_cmp,2,&nbrs,200);
		if (k==2)
		{
			d0 = descr_dist_sq(feat_cmp,nbrs[0]);
			d1 = descr_dist_sq(feat_cmp,nbrs[1]);
			if(d0<d1 * 0.5)
			{
				//TempMovingVector = Point2D(feat_cmp->img_pt.x-nbrs[0]->img_pt.x,feat_cmp->img_pt.y-nbrs[0]->img_pt.y);
				TempMovingVectorx = feat_cmp->img_pt.x-(nbrs[0]->img_pt.x+(trackingRect->left-trackingWindow->left));
				TempMovingVectory = feat_cmp->img_pt.y-(nbrs[0]->img_pt.y+(trackingRect->upper-trackingWindow->upper));
				if ((abs(TempMovingVectory)<=trackingRect->width/5.0)&&(abs(TempMovingVectorx)<=trackingRect->height/5.0) )
				{
					TempMoveScale = sqrt((double)(TempMovingVectorx*TempMovingVectorx+TempMovingVectory*TempMovingVectory));
					if (TempMoveScale>MaxMove)
					{
						MovingVectorx = TempMovingVectorx;
						MovingVectory = TempMovingVectory;
						MaxMove = TempMoveScale;
					}
					feat_cmp->fwd_match = nbrs[0];
					imhdr->paintPoint(Point2D(nbrs[0]->y+trackingRect->upper,nbrs[0]->x+trackingRect->left),Color(0,255,0),3);
					imhdr->paintPoint(Point2D(feat_cmp->y+trackingWindow->upper,feat_cmp->x+trackingWindow->left),Color(255,0,0),3);
					count++;
				}
			}
		}
		free(nbrs);
	}
	if((double)count/(double)this->Sfeat_num<=0)
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
/* Get geometric centroid with SIFT feature points*/
//Point2D SIFT_navie_tracker::GetCentroid()
//{
//	double tempX=0,tempY = 0,Volume = 0;;
//	for (int i = 0; i < this->Sfeat_num;i++)
//	{
//		Volume += this->tracking_template->GetMatchCount(i);
//		tempX += this->tracking_template->GetFeat(i)->img_pt.x * this->tracking_template->GetMatchCount(i);
//		tempY += this->tracking_template->GetFeat(i)->img_pt.y * this->tracking_template->GetMatchCount(i);
//	}
//	Point2D Centroid(cvRound(tempY/Volume),cvRound(tempX/Volume));
//	for ()
//	{
//	}
//	dist_sq_2D
//	return ;
//}

/* Get Density of SIFT feature points */
//double SIFT_navie_tracker::GetDensity()
//{
//
//
//}