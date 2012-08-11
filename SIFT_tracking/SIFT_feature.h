#pragma once
#include "Def.h"
#include "utils.h"


class SIFT_feature
{
public:
	SIFT_feature(void);
	~SIFT_feature(void);
	SIFT_feature(IplImage *);
	SIFT_feature(IplImage *,Rect);
	int import_features( char* filename, int type);
	int export_features( char* filename);
	void draw_features( IplImage* img);
	void draw_features(ImageHandler* imgh, Rect trackedRoi);
	

	int import_oxfd_features( char*);
	int export_oxfd_features( char*);
	void draw_oxfd_features( IplImage* );
	void draw_oxfd_features( ImageHandler*, Rect );
	void draw_oxfd_feature( IplImage*,unsigned , CvScalar );
	void draw_oxfd_feature( ImageHandler*,unsigned , CvScalar,Rect );
	int import_lowe_features( char*);
	int export_lowe_features( char*);
	void draw_lowe_features( IplImage*);
	void draw_lowe_feature( IplImage*, unsigned , CvScalar );
	void draw_lowe_features( ImageHandler*,Rect);
	void draw_lowe_feature( ImageHandler*, unsigned , CvScalar ,Rect );
	
	void sift_features( IplImage* img );

	void sift_features( IplImage* img, int intvls,
		double sigma, double contr_thr, int curv_thr,
		int img_dbl, int descr_width, int descr_hist_bins );
	void scale_space_extrema( IplImage***, int, int, double, int, CvMemStorage*);
	void calc_feature_scales( double, int );
	void adjust_for_img_dbl( );
	void calc_feature_oris( IplImage*** );
	void add_good_ori_features( double*, int, double, struct SIFT_feature_unit* );
	void compute_descriptors( IplImage***, int, int );
	SIFT_feature_unit *GetFeat(int pos);
	int GetLength();
	void AddMatchCount(int i);
	int GetMatchCount(int i);
private:
	vector<struct SIFT_feature_unit> feat;
	vector<int> MatchCount;
};

