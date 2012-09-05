#ifndef DEF_H
#define DEF_H

/** FEATURE_OXFD <BR> FEATURE_LOWE */
enum feature_type
{
	FEATURE_OXFD,
	FEATURE_LOWE,
};

/** FEATURE_FWD_MATCH <BR> FEATURE_BCK_MATCH <BR> FEATURE_MDL_MATCH */
enum feature_match_type
{
	FEATURE_FWD_MATCH,
	FEATURE_BCK_MATCH,
	FEATURE_MDL_MATCH,
};

/*define Pi*/
#define PI 3.1415926

/* colors in which to display different feature types */
#define FEATURE_OXFD_COLOR CV_RGB(255,255,0)
#define FEATURE_LOWE_COLOR CV_RGB(255,0,255)

/** max feature descriptor length */
#define FEATURE_MAX_D 128



/** default number of sampled intervals per octave */
#define SIFT_INTVLS 3

/** default sigma for initial gaussian smoothing */
#define SIFT_SIGMA 1.6

/** default threshold on keypoint contrast |D(x)| */
#define SIFT_CONTR_THR 0.04

/** default threshold on keypoint ratio of principle curvatures */
#define SIFT_CURV_THR 10

/** double image size before pyramid construction? */
#define SIFT_IMG_DBL 1

/** default width of descriptor histogram array */
#define SIFT_DESCR_WIDTH 4

/** default number of bins per histogram in descriptor array */
#define SIFT_DESCR_HIST_BINS 8

/* assumed gaussian blur for input image */
#define SIFT_INIT_SIGMA 0.5

/* width of border in which to ignore keypoints */
#define SIFT_IMG_BORDER 5

/* maximum steps of keypoint interpolation before failure */
#define SIFT_MAX_INTERP_STEPS 5

/* default number of bins in histogram for orientation assignment */
#define SIFT_ORI_HIST_BINS 36

/* determines gaussian sigma for orientation assignment */
#define SIFT_ORI_SIG_FCTR 1.5

/* determines the radius of the region used in orientation assignment */
#define SIFT_ORI_RADIUS 3.0 * SIFT_ORI_SIG_FCTR

/* number of passes of orientation histogram smoothing */
#define SIFT_ORI_SMOOTH_PASSES 2

/* orientation magnitude relative to max that results in new feature */
#define SIFT_ORI_PEAK_RATIO 0.8

/* determines the size of a single descriptor orientation histogram */
#define SIFT_DESCR_SCL_FCTR 3.0

/* threshold on magnitude of elements of descriptor vector */
#define SIFT_DESCR_MAG_THR 0.2

/* factor used to convert floating-point descriptor to unsigned char */
#define SIFT_INT_DESCR_FCTR 512.0

/* returns a feature's detection data */
#define feat_detection_data(f) ( (struct detection_data*)(f->feature_data) )

/** holds feature data relevant to detection */
struct detection_data
{
	int r;
	int c;
	int octv;
	int intvl;
	double subintvl;
	double scl_octv;
};


/*Sift feature struct*/
struct SIFT_feature_unit
{
	double x;                      /**< x coord */
	double y;                      /**< y coord */
	double a;                      /**< Oxford-type affine region parameter */
	double b;                      /**< Oxford-type affine region parameter */
	double c;                      /**< Oxford-type affine region parameter */
	double scl;                    /**< scale of a Lowe-style feature */
	double ori;                    /**< orientation of a Lowe-style feature */
	int d;                         /**< descriptor length */
	double descr[FEATURE_MAX_D];   /**< descriptor */
	int type;                      /**< feature type, OXFD or LOWE */
	int category;                     /**< all-purpose feature class */
	struct SIFT_feature_unit* fwd_match;     /**< matching feature from forward image */
	struct SIFT_feature_unit* bck_match;     /**< matching feature from backmward image */
	struct SIFT_feature_unit* mdl_match;     /**< matching feature from model */
	CvPoint2D64f img_pt;           /**< location in image */
	CvPoint2D64f mdl_pt;           /**< location in model */
	void* feature_data;            /**< user-definable data */
};

void ConvertImage(IplImage* source, IplImage* target, Rect Roi);
/* when tracking window move over this threshold */
#define TRACKING_PIXEL_THR 2

/* Tracking window size factor the template size */
#define TRACKING_WINDOW_SIZE 0.3

/* Optical flow point area*/
#define OPTICAL_FLOW_POINT_AREA 10
 
#endif