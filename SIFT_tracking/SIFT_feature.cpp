#include "StdAfx.h"
#include "SIFT_feature.h"
#include "utils.h"

SIFT_feature::SIFT_feature(void)
{
}

SIFT_feature::~SIFT_feature(void)
{
}

SIFT_feature::SIFT_feature(IplImage * img)
{
	sift_features(img);
}

SIFT_feature::SIFT_feature(IplImage *img,Rect trackingROI)
{
	IplImage* Tracking_template;
	Tracking_template = cvCreateImage(cvSize(trackingROI.width,trackingROI.height),
		img->depth,
		img->nChannels);
	ConvertImage(img,Tracking_template,trackingROI);
	sift_features(Tracking_template);
	this->MatchCount.assign((this->feat.end()-this->feat.begin()),0);
}
SIFT_feature_unit* SIFT_feature::GetFeat(int pos)
{
	return (&(this->feat[pos]));
}

int SIFT_feature::GetLength()
{
	return this->feat.size();
}

void SIFT_feature::AddMatchCount(int i)
{
	this->MatchCount[i]++;
}

int SIFT_feature::GetMatchCount(int i)
{
	return this->MatchCount[i];
}
/*
Reads image features from file.  The file should be formatted as from
the code provided by the Visual Geometry Group at Oxford:


@param filename location of a file containing image features
@param type determines how features are input.  If \a type is FEATURE_OXFD,
the input file is treated as if it is from the code provided by the VGG
at Oxford:

http://www.robots.ox.ac.uk:5000/~vgg/research/affine/index.html

If \a type is FEATURE_LOWE, the input file is treated as if it is from
David Lowe's SIFT code:

http://www.cs.ubc.ca/~lowe/keypoints  
@param features pointer to an array in which to store features

@return Returns the number of features imported from filename or -1 on error
*/

IplImage* create_init_img( IplImage*, int, double );
IplImage* convert_to_gray32( IplImage* );
IplImage*** build_gauss_pyr( IplImage*, int, int, double );
IplImage* downsample( IplImage* );
IplImage*** build_dog_pyr( IplImage***, int, int );
int is_extremum( IplImage***, int, int, int, int );
struct SIFT_feature_unit* interp_extremum( IplImage***, int, int, int, int, int, double);
void interp_step( IplImage***, int, int, int, int, double*, double*, double* );
CvMat* deriv_3D( IplImage***, int, int, int, int );
CvMat* hessian_3D( IplImage***, int, int, int, int );
double interp_contr( IplImage***, int, int, int, int, double, double, double );
struct SIFT_feature_unit* new_feature( void );
int is_too_edge_like( IplImage*, int, int, int );

double* ori_hist( IplImage*, int, int, int, int, double );
int calc_grad_mag_ori( IplImage*, int, int, double*, double* );
void smooth_ori_hist( double*, int );
double dominant_ori( double*, int );

struct SIFT_feature_unit* clone_feature( struct SIFT_feature_unit* );
double*** descr_hist( IplImage*, int, int, double, double, int, int );
void interp_hist_entry( double***, double, double, double, double, int, int);
void hist_to_descr( double***, int, int, struct SIFT_feature_unit* );
void normalize_descr( struct SIFT_feature_unit* );
bool feature_cmp( struct SIFT_feature_unit , struct SIFT_feature_unit );
void release_descr_hist( double****, int );
void release_pyr( IplImage****, int, int );

int SIFT_feature::import_features( char* filename, int type)
{
	int n;

	switch( type )
	{
	case FEATURE_OXFD:
		n = import_oxfd_features( filename);
		break;
	case FEATURE_LOWE:
		n = import_lowe_features( filename);
		break;
	default:
		fprintf( stderr, "Warning: import_features(): unrecognized feature" \
			"type, %s, line %d\n", __FILE__, __LINE__ );
		return -1;
	}

	if( n == -1 )
		fprintf( stderr, "Warning: unable to import features from %s,"	\
		" %s, line %d\n", filename, __FILE__, __LINE__ );
	return n;
}



/*
Exports a feature set to a file formatted depending on the type of
features, as specified in the feature struct's type field.

@param filename name of file to which to export features
@param feat feature array
@param n number of features 

@return Returns 0 on success or 1 on error
*/
int SIFT_feature::export_features( char* filename)
{
	int r, type;

	if( feat.size() <= 0 )
	{
		fprintf( stderr, "Warning: no features to export, %s line %d\n",
			__FILE__, __LINE__ );
		return 1;
	}
	
	type = feat[0].type;
	switch( type )
	{
	case FEATURE_OXFD:
		r = export_oxfd_features( filename );
		break;
	case FEATURE_LOWE:
		r = export_lowe_features( filename );
		break;
	default:
		fprintf( stderr, "Warning: export_features(): unrecognized feature" \
			"type, %s, line %d\n", __FILE__, __LINE__ );
		return -1;
	}

	if( r )
		fprintf( stderr, "Warning: unable to export features to %s,"	\
		" %s, line %d\n", filename, __FILE__, __LINE__ );
	return r;
}


/*
Draws a set of features on an image

@param img image on which to draw features
@param feat array of Oxford-type features
@param n number of features
*/
void SIFT_feature::draw_features( IplImage* img )
{
	int type;

	if( feat.size() <= 0 )
	{
		fprintf( stderr, "Warning: no features to draw, %s line %d\n",
			__FILE__, __LINE__ );
		return;
	}
	type = feat[0].type;
	switch( type )
	{
	case FEATURE_OXFD:
		draw_oxfd_features( img );
		break;
	case FEATURE_LOWE:
		draw_lowe_features( img );
		break;
	default:
		fprintf( stderr, "Warning: draw_features(): unrecognized feature" \
			" type, %s, line %d\n", __FILE__, __LINE__ );
		break;
	}
}

void SIFT_feature::draw_features( ImageHandler* imgh , Rect trackedRoi)
{
	int type;

	if( feat.size() <= 0 )
	{
		fprintf( stderr, "Warning: no features to draw, %s line %d\n",
			__FILE__, __LINE__ );
		return;
	}
	type = feat[0].type;
	switch( type )
	{
	case FEATURE_OXFD:
		draw_oxfd_features( imgh ,trackedRoi);
		break;
	case FEATURE_LOWE:
		draw_lowe_features( imgh ,trackedRoi);
		break;
	default:
		fprintf( stderr, "Warning: draw_features(): unrecognized feature" \
			" type, %s, line %d\n", __FILE__, __LINE__ );
		break;
	}
}

/***************************** Local Functions *******************************/


/*
Reads image features from file.  The file should be formatted as from
the code provided by the Visual Geometry Group at Oxford:

http://www.robots.ox.ac.uk:5000/~vgg/research/affine/index.html

@param filename location of a file containing image features
@param features pointer to an array in which to store features

@return Returns the number of features imported from filename or -1 on error
*/
int SIFT_feature::import_oxfd_features( char* filename)
{
	struct SIFT_feature_unit* f;
	int i, j, n, d;
	double x, y, a, b, c, dv;
	FILE* file;

	/*if( ! features )
		fatal_error( "NULL pointer error, %s, line %d",  __FILE__, __LINE__ );*/

	if( ! ( file = fopen( filename, "r" ) ) )
	{
		fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
			filename, __FILE__, __LINE__ );
		return -1;
	}

	/* read dimension and number of features */
	if( fscanf( file, " %d %d ", &d, &n ) != 2 )
	{
		fprintf( stderr, "Warning: file read error, %s, line %d\n",
			__FILE__, __LINE__ );
		return -1;
	}
	if( d > FEATURE_MAX_D )
	{
		fprintf( stderr, "Warning: descriptor too long, %s, line %d\n",
			__FILE__, __LINE__ );
		return -1;
	}


	for( i = 0; i < n; i++ )
	{
		f = new struct SIFT_feature_unit;
		/* read affine region parameters */
		if( fscanf( file, " %lf %lf %lf %lf %lf ", &x, &y, &a, &b, &c ) != 5 )
		{
			fprintf( stderr, "Warning: error reading feature #%d, %s, line %d\n",
				i+1, __FILE__, __LINE__ );
			delete f;
			return -1;
		}
		f->img_pt.x = f->x = x;
		f->img_pt.y = f->y = y;
		f->a = a;
		f->b = b;
		f->c = c;
		f->d = d;
		f->type = FEATURE_OXFD;

		/* read descriptor */
		for( j = 0; j < d; j++ )
		{
			if( ! fscanf( file, " %lf ", &dv ) )
			{
				fprintf( stderr, "Warning: error reading feature descriptor" \
					" #%d, %s, line %d\n", i+1, __FILE__, __LINE__ );
				delete f;
				return -1;
			}
			f->descr[j] = dv;
		}

		f->scl = f->ori = 0;
		f->category = 0;
		f->fwd_match = f->bck_match = f->mdl_match = NULL;
		f->mdl_pt.x = f->mdl_pt.y = -1;
		f->feature_data = NULL;
		this->feat.insert(feat.begin(),*f);
		delete f;
	}

	if( fclose(file) )
	{
		fprintf( stderr, "Warning: file close error, %s, line %d\n",
			__FILE__, __LINE__ );
		delete f;
		return -1;
	}
	
	
	return n;
}




/*
Exports a feature set to a file formatted as one from the code provided
by the Visual Geometry Group at Oxford:

http://www.robots.ox.ac.uk:5000/~vgg/research/affine/index.html

@param filename name of file to which to export features
@param feat feature array
@param n number of features

@return Returns 0 on success or 1 on error
*/
int SIFT_feature::export_oxfd_features( char* filename)
{
	FILE* file;
	int i, j, d;
	int pointlength = feat.size();
	if( feat.size() <= 0 )
	{
		fprintf( stderr, "Warning: feature count %d, %s, line %s\n",
			feat.size(), __FILE__, __LINE__ );
		return 1;
	}
	if( ! ( file = fopen( filename, "w" ) ) )
	{
		fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
			filename, __FILE__, __LINE__ );
		return 1;
	}

	d = feat[0].d;
	fprintf( file, "%d\n%d\n", d, pointlength );
	for( i = 0; i < pointlength; i++ )
	{
		fprintf( file, "%f %f %f %f %f", feat[i].x, feat[i].y, feat[i].a,
			feat[i].b, feat[i].c );
		for( j = 0; j < d; j++ )
			fprintf( file, " %f", feat[i].descr[j] );
		fprintf( file, "\n" );
	}

	if( fclose(file) )
	{
		fprintf( stderr, "Warning: file close error, %s, line %d\n",
			__FILE__, __LINE__ );
		return 1;
	}

	return 0;
}



/*
Draws Oxford-type affine features

@param img image on which to draw features
@param feat array of Oxford-type features
@param n number of features
*/
void SIFT_feature::draw_oxfd_features( IplImage* img)
{
	CvScalar color = CV_RGB( 255, 255, 255 );

	if( img-> nChannels > 1 )
		color = FEATURE_OXFD_COLOR;
	for( unsigned i = 0; i < feat.size(); i++ )
		draw_oxfd_feature( img, i, color );
}

void SIFT_feature::draw_oxfd_features( ImageHandler* imgh , Rect trackedRoi)
{
	CvScalar color = CV_RGB( 255, 255, 255 );

	/*if( imgh-> nChannels > 1 )*/
		color = FEATURE_OXFD_COLOR;
	for( unsigned i = 0; i < feat.size(); i++ )
		draw_oxfd_feature( imgh, i, color , trackedRoi);
}


/*
Draws a single Oxford-type feature

@param img image on which to draw
@param feat feature to be drawn
@param color color in which to draw
*/
void SIFT_feature::draw_oxfd_feature( IplImage* img, unsigned pos, CvScalar color )
{
	double m[4] = { feat[pos].a, feat[pos].b, feat[pos].b, feat[pos].c };
	double v[4] = { 0 };
	double e[2] = { 0 };
	CvMat M, V, E;
	double alpha, l1, l2;

	/* compute axes and orientation of ellipse surrounding affine region */
	cvInitMatHeader( &M, 2, 2, CV_64FC1, m, CV_AUTOSTEP );
	cvInitMatHeader( &V, 2, 2, CV_64FC1, v, CV_AUTOSTEP );
	cvInitMatHeader( &E, 2, 1, CV_64FC1, e, CV_AUTOSTEP );
	cvEigenVV( &M, &V, &E, DBL_EPSILON ,-1,-1);
	l1 = 1 / sqrt( e[1] );
	l2 = 1 / sqrt( e[0] );
	alpha = -atan2( v[1], v[0] );
	alpha *= 180 / CV_PI;

	cvEllipse( img, cvPoint( feat[pos].x, feat[pos].y ), cvSize( l2, l1 ), alpha,
		0, 360, CV_RGB(0,0,0), 3, 8, 0 );
	cvEllipse( img, cvPoint( feat[pos].x, feat[pos].y ), cvSize( l2, l1 ), alpha,
		0, 360, color, 1, 8, 0 );
	cvLine( img, cvPoint( feat[pos].x+2, feat[pos].y ), cvPoint( feat[pos].x-2, feat[pos].y ),
		color, 1, 8, 0 );
	cvLine( img, cvPoint( feat[pos].x, feat[pos].y+2 ), cvPoint( feat[pos].x, feat[pos].y-2 ),
		color, 1, 8, 0 );
}

void SIFT_feature::draw_oxfd_feature( ImageHandler* imgh, unsigned pos, CvScalar color ,Rect trackedRoi)
{
	double m[4] = { feat[pos].a, feat[pos].b, feat[pos].b, feat[pos].c };
	double v[4] = { 0 };
	double e[2] = { 0 };
	CvMat M, V, E;
	double alpha, l1, l2;

	/* compute axes and orientation of ellipse surrounding affine region */
	cvInitMatHeader( &M, 2, 2, CV_64FC1, m, CV_AUTOSTEP );
	cvInitMatHeader( &V, 2, 2, CV_64FC1, v, CV_AUTOSTEP );
	cvInitMatHeader( &E, 2, 1, CV_64FC1, e, CV_AUTOSTEP );
	cvEigenVV( &M, &V, &E, DBL_EPSILON ,-1,-1);
	l1 = 1 / sqrt( e[1] );
	l2 = 1 / sqrt( e[0] );
	alpha = -atan2( v[1], v[0] );
	alpha *= 180 / CV_PI;

	/*cvEllipse( img, cvPoint( feat[pos].x, feat[pos].y ), cvSize( l2, l1 ), alpha,
		0, 360, CV_RGB(0,0,0), 3, 8, 0 );
	cvEllipse( img, cvPoint( feat[pos].x, feat[pos].y ), cvSize( l2, l1 ), alpha,
		0, 360, color, 1, 8, 0 );
	cvLine( img, cvPoint( feat[pos].x+2, feat[pos].y ), cvPoint( feat[pos].x-2, feat[pos].y ),
		color, 1, 8, 0 );
	cvLine( img, cvPoint( feat[pos].x, feat[pos].y+2 ), cvPoint( feat[pos].x, feat[pos].y-2 ),
		color, 1, 8, 0 );*/
    
	imgh->paintCircle(Point2D(feat[pos].y,feat[pos].x),12,Color(0,0,0),1);
	imgh->paintCircle(Point2D(feat[pos].y,feat[pos].x),12,Color(color.val[0],color.val[1],color.val[2]),1);
	imgh->paintLine(Point2D(feat[pos].y+trackedRoi.upper,feat[pos].x+2+trackedRoi.left) , 
		Point2D(feat[pos].y-2+trackedRoi.upper, feat[pos].x+trackedRoi.left), Color(color.val[0],color.val[1],color.val[2]), 1);
	imgh->paintLine(Point2D(feat[pos].y+2+trackedRoi.upper,feat[pos].x+trackedRoi.left) ,
		Point2D(feat[pos].y+trackedRoi.upper, feat[pos].x-2+trackedRoi.left), Color(color.val[0],color.val[1],color.val[2]), 1);

}

/*
Reads image features from file.  The file should be formatted as from
the code provided by David Lowe:

http://www.cs.ubc.ca/~lowe/keypoints/

@param filename location of a file containing image features
@param features pointer to an array in which to store features

@return Returns the number of features imported from filename or -1 on error
*/
int SIFT_feature::import_lowe_features( char* filename )
{
	struct SIFT_feature_unit* f;
	int i, j, n=0, d=0;
	double x, y, s, o, dv;
	FILE* file;

	/*if( ! features )
		fatal_error( "NULL pointer error, %s, line %d",  __FILE__, __LINE__ );*/

	if( ! ( file = fopen( filename, "r" ) ) )
	{
		fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
			filename, __FILE__, __LINE__ );
		return -1;
	}

	/* read number of features and dimension */
	if( fscanf( file, " %d %d ", &n, &d ) != 2 )
	{
		fprintf( stderr, "Warning: file read error, %s, line %d\n",
			__FILE__, __LINE__ );
		return -1;
	}
	if( d > FEATURE_MAX_D )
	{
		fprintf( stderr, "Warning: descriptor too long, %s, line %d\n",
			__FILE__, __LINE__ );
		return -1;
	}

	
	for( i = 0; i < n; i++ )
	{
		f = new struct SIFT_feature_unit;
		/* read affine region parameters */
		if( fscanf( file, " %lf %lf %lf %lf ", &y, &x, &s, &o ) != 4 )
		{
			fprintf( stderr, "Warning: error reading feature #%d, %s, line %d\n",
				i+1, __FILE__, __LINE__ );
			delete( f );
			return -1;
		}
		f->img_pt.x = f->x = x;
		f->img_pt.y = f->y = y;
		f->scl = s;
		f->ori = o;
		f->d = d;
		f->type = FEATURE_LOWE;

		/* read descriptor */
		for( j = 0; j < d; j++ )
		{
			if( ! fscanf( file, " %lf ", &dv ) )
			{
				fprintf( stderr, "Warning: error reading feature descriptor" \
					" #%d, %s, line %d\n", i+1, __FILE__, __LINE__ );
				delete( f );
				return -1;
			}
			f->descr[j] = dv;
		}

		f->a = f->b = f->c = 0;
		f->category = 0;
		f->fwd_match = f->bck_match = f->mdl_match = NULL;
		f->mdl_pt.x = f->mdl_pt.y = -1;
		this->feat.insert(feat.begin(),*f);
	}

	if( fclose(file) )
	{
		fprintf( stderr, "Warning: file close error, %s, line %d\n",
			__FILE__, __LINE__ );
		delete( f );
		return -1;
	}

	return n;
}



/*
Exports a feature set to a file formatted as one from the code provided
by David Lowe:

http://www.cs.ubc.ca/~lowe/keypoints/

@param filename name of file to which to export features
@param feat feature array
@param n number of features

@return Returns 0 on success or 1 on error
*/
int SIFT_feature::export_lowe_features( char* filename)
{
	FILE* file;
	int i, j, d;
	int pointlength = feat.size();
	if( pointlength <= 0 )
	{
		fprintf( stderr, "Warning: feature count %d, %s, line %s\n",
			pointlength, __FILE__, __LINE__ );
		return 1;
	}
	if( ! ( file = fopen( filename, "w" ) ) )
	{
		fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
			filename, __FILE__, __LINE__ );
		return 1;
	}

	d = feat[0].d;
	fprintf( file, "%d %d\n", pointlength, d );
	for( i = 0; i < pointlength; i++ )
	{
		fprintf( file, "%f %f %f %f", feat[i].y, feat[i].x,
			feat[i].scl, feat[i].ori );
		for( j = 0; j < d; j++ )
		{
			/* write 20 descriptor values per line */
			if( j % 20 == 0 )
				fprintf( file, "\n" );
			fprintf( file, " %d", (int)(feat[i].descr[j]) );
		}
		fprintf( file, "\n" );
	}

	if( fclose(file) )
	{
		fprintf( stderr, "Warning: file close error, %s, line %d\n",
			__FILE__, __LINE__ );
		return 1;
	}

	return 0;
}


/*
Draws Lowe-type features

@param img image on which to draw features
@param feat array of Oxford-type features
@param n number of features
*/
void SIFT_feature::draw_lowe_features( IplImage* img)
{
	CvScalar color = CV_RGB( 255, 255, 255 );
	

	if( img-> nChannels > 1 )
		color = FEATURE_LOWE_COLOR;
	for(unsigned i = 0; i < feat.size(); i++ )
		draw_lowe_feature( img, i, color );
}
void SIFT_feature::draw_lowe_features( ImageHandler* imgh, Rect trackedRoi)
{
	CvScalar color = CV_RGB( 255, 255, 255 );


	/*if( img-> nChannels > 1 )*/
		color = FEATURE_LOWE_COLOR;
	for(unsigned i = 0; i < feat.size(); i++ )
		draw_lowe_feature( imgh, i, color , trackedRoi);
}


/*
Draws a single Lowe-type feature

@param img image on which to draw
@param feat feature to be drawn
@param color color in which to draw
*/
void SIFT_feature::draw_lowe_feature( IplImage* img,unsigned pos, CvScalar color)
{
	int len, hlen, blen, start_x, start_y, end_x, end_y, h1_x, h1_y, h2_x, h2_y;
	double scl, ori;
	double scale = 5.0;
	double hscale = 0.75;
	CvPoint start, end, h1, h2;

	/* compute points for an arrow scaled and rotated by feat's scl and ori */
	start_x = cvRound( feat[pos].x );
	start_y = cvRound( feat[pos].y );
	scl = feat[pos].scl;
	ori = feat[pos].ori;
	len = cvRound( scl * scale );
	hlen = cvRound( scl * hscale );
	blen = len - hlen;
	end_x = cvRound( len *  cos( ori ) ) + start_x;
	end_y = cvRound( len * -sin( ori ) ) + start_y;
	h1_x = cvRound( blen *  cos( ori + CV_PI / 18.0 ) ) + start_x;
	h1_y = cvRound( blen * -sin( ori + CV_PI / 18.0 ) ) + start_y;
	h2_x = cvRound( blen *  cos( ori - CV_PI / 18.0 ) ) + start_x;
	h2_y = cvRound( blen * -sin( ori - CV_PI / 18.0 ) ) + start_y;
	start = cvPoint( start_x, start_y );
	end = cvPoint( end_x, end_y );
	h1 = cvPoint( h1_x, h1_y );
	h2 = cvPoint( h2_x, h2_y );

	cvLine( img, start, end, color, 2, 8, 0 );
	cvLine( img, end, h1, color, 1, 8, 0 );
	cvLine( img, end, h2, color, 1, 8, 0 );
}

void SIFT_feature::draw_lowe_feature( ImageHandler* imgh, unsigned pos, CvScalar color  ,Rect trackedRoi)
{
	int len, hlen, blen, start_x, start_y, end_x, end_y, h1_x, h1_y, h2_x, h2_y;
	double scl, ori;
	double scale = 5.0;
	double hscale = 0.75;
	Point2D start, end, h1, h2;

	/* compute points for an arrow scaled and rotated by feat's scl and ori */
	start_x = cvRound( feat[pos].x );
	start_y = cvRound( feat[pos].y );
	scl = feat[pos].scl;
	ori = feat[pos].ori;
	len = cvRound( scl * scale );
	hlen = cvRound( scl * hscale );
	blen = len - hlen;
	end_x = cvRound( len *  cos( ori ) ) + start_x;
	end_y = cvRound( len * -sin( ori ) ) + start_y;
	h1_x = cvRound( blen *  cos( ori + CV_PI / 18.0 ) ) + start_x;
	h1_y = cvRound( blen * -sin( ori + CV_PI / 18.0 ) ) + start_y;
	h2_x = cvRound( blen *  cos( ori - CV_PI / 18.0 ) ) + start_x;
	h2_y = cvRound( blen * -sin( ori - CV_PI / 18.0 ) ) + start_y;
	start = Point2D( start_y + trackedRoi.upper , start_x + trackedRoi.left );
	end = Point2D( end_y + trackedRoi.upper, end_x + trackedRoi.left );
	h1 = Point2D( h1_y + trackedRoi.upper , h1_x + trackedRoi.left);
	h2 = Point2D( h2_y + trackedRoi.upper, h2_x + trackedRoi.left );

	/*cvLine( img, start, end, color, 2, 8, 0 );
	cvLine( img, end, h1, color, 1, 8, 0 );
	cvLine( img, end, h2, color, 1, 8, 0 );*/

	imgh->paintLine(start,end,Color(color.val[0],color.val[1],color.val[2]),1);
	imgh->paintLine(end,h1,Color(color.val[0],color.val[1],color.val[2]),1);
	imgh->paintLine(end,h2,Color(color.val[0],color.val[1],color.val[2]),1);
}

void SIFT_feature::sift_features( IplImage* img )
{
	 sift_features( img, SIFT_INTVLS, SIFT_SIGMA, SIFT_CONTR_THR,
		SIFT_CURV_THR, SIFT_IMG_DBL, SIFT_DESCR_WIDTH,
		SIFT_DESCR_HIST_BINS );
}

void SIFT_feature::sift_features( IplImage* img, int intvls,
				   double sigma, double contr_thr, int curv_thr,
				   int img_dbl, int descr_width, int descr_hist_bins )
{
	IplImage* init_img;
	IplImage*** gauss_pyr, *** dog_pyr;
	CvMemStorage* storage;
	//CvSeq* features;
	int octvs, i, n = 0;
	double start_time;
	double during_time;

	/* check arguments */
	if( ! img )
		fatal_error( "NULL pointer error, %s, line %d",  __FILE__, __LINE__ );

	/*if( ! feat )
		fatal_error( "NULL pointer error, %s, line %d",  __FILE__, __LINE__ );*/

	/* build scale space pyramid; smallest dimension of top level is ~4 pixels */
	init_img = create_init_img( img, img_dbl, sigma );
	octvs = (int)log( (double)MIN( init_img->width, init_img->height ) ) / log(2.0) - 2;

	start_time = clock();
	gauss_pyr = build_gauss_pyr( init_img, octvs, intvls, sigma );
	during_time = (clock() - start_time)/CLOCKS_PER_SEC;
	printf("time of build gauss_pyr:%f\n",during_time);

	start_time = clock();
	dog_pyr = build_dog_pyr( gauss_pyr, octvs, intvls );
	during_time = (clock() - start_time)/CLOCKS_PER_SEC;
	printf("time of build dob_pyr:%f\n",during_time);

	storage = cvCreateMemStorage( 0 );
	start_time = clock();
	scale_space_extrema( dog_pyr, octvs, intvls, contr_thr,
		curv_thr, storage );
	during_time = (clock() - start_time)/CLOCKS_PER_SEC;
	printf("time of scale_space_extrema:%f\n",during_time);

	start_time = clock();
	calc_feature_scales( sigma, intvls );

	if( img_dbl )
		adjust_for_img_dbl(  );
	calc_feature_oris(  gauss_pyr );
	compute_descriptors( gauss_pyr, descr_width, descr_hist_bins );
	during_time = (clock() - start_time)/CLOCKS_PER_SEC;
	printf("time of compute_descriptors:%f\n",during_time);

	/* sort features by decreasing scale and move from CvSeq to array */
	//cvSeqSort(  (CvCmpFunc)feature_cmp, NULL );
	sort(feat.begin(),feat.end(),feature_cmp);
	//n = features->total;
	//*feat = (struct feature *)calloc( n, sizeof(struct feature) );
	//*feat = (struct feature *)cvCvtSeqToArray( features, *feat, CV_WHOLE_SEQ );
	for( i = 0; i < n; i++ )
	{
		free( feat[i].feature_data );
		feat[i].feature_data = NULL;
	}

	cvReleaseMemStorage( &storage );
	cvReleaseImage( &init_img );
	release_pyr( &gauss_pyr, octvs, intvls + 3 );
	release_pyr( &dog_pyr, octvs, intvls + 2 );
//	return n;
}

IplImage* create_init_img( IplImage* img, int img_dbl, double sigma )
{
	IplImage* gray, * dbl;
	float sig_diff;

	gray = convert_to_gray32( img );
	if( img_dbl )
	{
		sig_diff = sqrt( sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA * 4 );
		dbl = cvCreateImage( cvSize( img->width*2, img->height*2 ),
			IPL_DEPTH_32F, 1 );
		cvResize( gray, dbl, CV_INTER_CUBIC );
		cvSmooth( dbl, dbl, CV_GAUSSIAN, 0, 0, sig_diff, sig_diff );
		cvReleaseImage( &gray );
		return dbl;
	}
	else
	{
		sig_diff = sqrt( sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA );
		cvSmooth( gray, gray, CV_GAUSSIAN, 0, 0, sig_diff, sig_diff );
		return gray;
	}
}

IplImage* convert_to_gray32( IplImage* img )
{
	IplImage* gray8, * gray32;
	int r, c;

	gray8 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
	gray32 = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );

	if( img->nChannels == 1 )
		gray8 = (IplImage*)cvClone( img );
	else
		cvCvtColor( img, gray8, CV_RGB2GRAY );
	cvConvertScale( gray8, gray32, 1.0 / 255.0, 0 );

	cvReleaseImage( &gray8 );
	return gray32;
}

/*
Builds Gaussian scale space pyramid from an image

@param base base image of the pyramid
@param octvs number of octaves of scale space
@param intvls number of intervals per octave
@param sigma amount of Gaussian smoothing per octave

@return Returns a Gaussian scale space pyramid as an octvs x (intvls + 3) array
*/
IplImage*** build_gauss_pyr( IplImage* base, int octvs,
							int intvls, double sigma )
{
	IplImage*** gauss_pyr;
	double* sig = (double*)calloc( intvls + 3, sizeof(double));
	double sig_total, sig_prev, k;
	int i, o;

	gauss_pyr = (IplImage***)calloc( octvs, sizeof( IplImage** ) );
	for( i = 0; i < octvs; i++ )
		gauss_pyr[i] = (IplImage**)calloc( intvls + 3, sizeof( IplImage* ) );

	/*
	precompute Gaussian sigmas using the following formula:

	\sigma_{total}^2 = \sigma_{i}^2 + \sigma_{i-1}^2
	*/
	sig[0] = sigma;
	k = pow( 2.0, 1.0 / intvls );
	for( i = 1; i < intvls + 3; i++ )
	{
		sig_prev = pow( k, i - 1 ) * sigma;
		sig_total = sig_prev * k;
		sig[i] = sqrt( sig_total * sig_total - sig_prev * sig_prev );
	}

	for( o = 0; o < octvs; o++ )
		for( i = 0; i < intvls + 3; i++ )
		{
			if( o == 0  &&  i == 0 )
				gauss_pyr[o][i] = cvCloneImage(base);

			/* base of new octvave is halved image from end of previous octave */
			else if( i == 0 )
				gauss_pyr[o][i] = downsample( gauss_pyr[o-1][intvls] );

			/* blur the current octave's last image to create the next one */
			else
			{
				gauss_pyr[o][i] = cvCreateImage( cvGetSize(gauss_pyr[o][i-1]),
					IPL_DEPTH_32F, 1 );
				cvSmooth( gauss_pyr[o][i-1], gauss_pyr[o][i],
					CV_GAUSSIAN, 0, 0, sig[i], sig[i] );
			}
		}

		free( sig );
		return gauss_pyr;
}

/*
Downsamples an image to a quarter of its size (half in each dimension)
using nearest-neighbor interpolation

@param img an image

@return Returns an image whose dimensions are half those of img
*/
IplImage* downsample( IplImage* img )
{
	IplImage* smaller = cvCreateImage( cvSize(img->width / 2, img->height / 2),
		img->depth, img->nChannels );
	cvResize( img, smaller, CV_INTER_NN );

	return smaller;
}

/*
Builds a difference of Gaussians scale space pyramid by subtracting adjacent
intervals of a Gaussian pyramid

@param gauss_pyr Gaussian scale-space pyramid
@param octvs number of octaves of scale space
@param intvls number of intervals per octave

@return Returns a difference of Gaussians scale space pyramid as an
octvs x (intvls + 2) array
*/
IplImage*** build_dog_pyr( IplImage*** gauss_pyr, int octvs, int intvls )
{
	IplImage*** dog_pyr;
	int i, o;

	dog_pyr = (IplImage***)calloc( octvs, sizeof( IplImage** ) );
	for( i = 0; i < octvs; i++ )
		dog_pyr[i] = (IplImage**)calloc( intvls + 2, sizeof(IplImage*) );

	for( o = 0; o < octvs; o++ )
		for( i = 0; i < intvls + 2; i++ )
		{
			dog_pyr[o][i] = cvCreateImage( cvGetSize(gauss_pyr[o][i]),
				IPL_DEPTH_32F, 1 );
			cvSub( gauss_pyr[o][i+1], gauss_pyr[o][i], dog_pyr[o][i], NULL );
		}

		return dog_pyr;
}

/*
Detects features at extrema in DoG scale space.  Bad features are discarded
based on contrast and ratio of principal curvatures.

@param dog_pyr DoG scale space pyramid
@param octvs octaves of scale space represented by dog_pyr
@param intvls intervals per octave
@param contr_thr low threshold on feature contrast
@param curv_thr high threshold on feature ratio of principal curvatures
@param storage memory storage in which to store detected features

@return Returns an array of detected features whose scales, orientations,
and descriptors are yet to be determined.
*/
void SIFT_feature::scale_space_extrema( IplImage*** dog_pyr, int octvs, int intvls,
						   double contr_thr, int curv_thr,
						   CvMemStorage* storage )
{
	//CvSeq* features;
	double prelim_contr_thr = 0.5 * contr_thr / intvls;
	struct SIFT_feature_unit* feature;
	struct detection_data * ddata;
	int o, i, r, c, w, h;

	//features = cvCreateSeq( 0, sizeof(CvSeq), sizeof(struct feature), storage );
	for( o = 0; o < octvs; o++ )
		for( i = 1; i <= intvls; i++ )
			for(r = SIFT_IMG_BORDER; r < dog_pyr[o][0]->height-SIFT_IMG_BORDER; r++)
				for(c = SIFT_IMG_BORDER; c < dog_pyr[o][0]->width-SIFT_IMG_BORDER; c++)
					/* perform preliminary check on contrast */
					if( ABS( pixval32f( dog_pyr[o][i], r, c ) ) > prelim_contr_thr )
						if( is_extremum( dog_pyr, o, i, r, c ) )
						{
							feature = interp_extremum(dog_pyr, o, i, r, c, intvls, contr_thr);
							if( feature )
							{
								ddata = (detection_data*)feature->feature_data ;
								if( ! is_too_edge_like( dog_pyr[ddata->octv][ddata->intvl],
									ddata->r, ddata->c, curv_thr ) )
								{
									//cvSeqPush( features, feat );
									feat.insert(feat.begin(),*feature);
								}
								else
									free( ddata );
								free( feature );
							}
						}

//	return features;
}

/*
Determines whether a pixel is a scale-space extremum by comparing it to it's
3x3x3 pixel neighborhood.

@param dog_pyr DoG scale space pyramid
@param octv pixel's scale space octave
@param intvl pixel's within-octave interval
@param r pixel's image row
@param c pixel's image col

@return Returns 1 if the specified pixel is an extremum (max or min) among
it's 3x3x3 pixel neighborhood.
*/
int is_extremum( IplImage*** dog_pyr, int octv, int intvl, int r, int c )
{
	float val = pixval32f( dog_pyr[octv][intvl], r, c );
	int i, j, k;

	/* check for maximum */
	if( val > 0 )
	{
		for( i = -1; i <= 1; i++ )
			for( j = -1; j <= 1; j++ )
				for( k = -1; k <= 1; k++ )
					if( val < pixval32f( dog_pyr[octv][intvl+i], r + j, c + k ) )
						return 0;
	}

	/* check for minimum */
	else
	{
		for( i = -1; i <= 1; i++ )
			for( j = -1; j <= 1; j++ )
				for( k = -1; k <= 1; k++ )
					if( val > pixval32f( dog_pyr[octv][intvl+i], r + j, c + k ) )
						return 0;
	}

	return 1;
}

/*
Interpolates a scale-space extremum's location and scale to subpixel
accuracy to form an image feature.  Rejects features with low contrast.
Based on Section 4 of Lowe's paper.  

@param dog_pyr DoG scale space pyramid
@param octv feature's octave of scale space
@param intvl feature's within-octave interval
@param r feature's image row
@param c feature's image column
@param intvls total intervals per octave
@param contr_thr threshold on feature contrast

@return Returns the feature resulting from interpolation of the given
parameters or NULL if the given location could not be interpolated or
if contrast at the interpolated loation was too low.  If a feature is
returned, its scale, orientation, and descriptor are yet to be determined.
*/
struct SIFT_feature_unit* interp_extremum( IplImage*** dog_pyr, int octv, int intvl,
	int r, int c, int intvls, double contr_thr )
{
	struct SIFT_feature_unit* feat;
	struct detection_data* ddata;
	double xi, xr, xc, contr;
	int i = 0;

	while( i < SIFT_MAX_INTERP_STEPS )
	{
		interp_step( dog_pyr, octv, intvl, r, c, &xi, &xr, &xc );
		if( ABS( xi ) < 0.5  &&  ABS( xr ) < 0.5  &&  ABS( xc ) < 0.5 )
			break;

		c += cvRound( xc );
		r += cvRound( xr );
		intvl += cvRound( xi );

		if( intvl < 1  ||
			intvl > intvls  ||
			c < SIFT_IMG_BORDER  ||
			r < SIFT_IMG_BORDER  ||
			c >= dog_pyr[octv][0]->width - SIFT_IMG_BORDER  ||
			r >= dog_pyr[octv][0]->height - SIFT_IMG_BORDER )
		{
			return NULL;
		}

		i++;
	}

	/* ensure convergence of interpolation */
	if( i >= SIFT_MAX_INTERP_STEPS )
		return NULL;

	contr = interp_contr( dog_pyr, octv, intvl, r, c, xi, xr, xc );
	if( ABS( contr ) < contr_thr / intvls )
		return NULL;

	feat = new_feature();
	ddata = feat_detection_data( feat );
	feat->img_pt.x = feat->x = ( c + xc ) * pow( 2.0, octv );
	feat->img_pt.y = feat->y = ( r + xr ) * pow( 2.0, octv );
	ddata->r = r;
	ddata->c = c;
	ddata->octv = octv;
	ddata->intvl = intvl;
	ddata->subintvl = xi;

	return feat;
}

/*
Performs one step of extremum interpolation.  Based on Eqn. (3) in Lowe's
paper.

@param dog_pyr difference of Gaussians scale space pyramid
@param octv octave of scale space
@param intvl interval being interpolated
@param r row being interpolated
@param c column being interpolated
@param xi output as interpolated subpixel increment to interval
@param xr output as interpolated subpixel increment to row
@param xc output as interpolated subpixel increment to col
*/

void interp_step( IplImage*** dog_pyr, int octv, int intvl, int r, int c,
				 double* xi, double* xr, double* xc )
{
	CvMat* dD, * H, * H_inv, X;
	double x[3] = { 0 };

	dD = deriv_3D( dog_pyr, octv, intvl, r, c );
	H = hessian_3D( dog_pyr, octv, intvl, r, c );
	H_inv = cvCreateMat( 3, 3, CV_64FC1 );
	cvInvert( H, H_inv, CV_SVD );
	cvInitMatHeader( &X, 3, 1, CV_64FC1, x, CV_AUTOSTEP );
	cvGEMM( H_inv, dD, -1, NULL, 0, &X, 0 );

	cvReleaseMat( &dD );
	cvReleaseMat( &H );
	cvReleaseMat( &H_inv );

	*xi = x[2];
	*xr = x[1];
	*xc = x[0];
}

/*
Computes the partial derivatives in x, y, and scale of a pixel in the DoG
scale space pyramid.

@param dog_pyr DoG scale space pyramid
@param octv pixel's octave in dog_pyr
@param intvl pixel's interval in octv
@param r pixel's image row
@param c pixel's image col

@return Returns the vector of partial derivatives for pixel I
{ dI/dx, dI/dy, dI/ds }^T as a CvMat*
*/
CvMat* deriv_3D( IplImage*** dog_pyr, int octv, int intvl, int r, int c )
{
	CvMat* dI;
	double dx, dy, ds;

	dx = ( pixval32f( dog_pyr[octv][intvl], r, c+1 ) -
		pixval32f( dog_pyr[octv][intvl], r, c-1 ) ) / 2.0;
	dy = ( pixval32f( dog_pyr[octv][intvl], r+1, c ) -
		pixval32f( dog_pyr[octv][intvl], r-1, c ) ) / 2.0;
	ds = ( pixval32f( dog_pyr[octv][intvl+1], r, c ) -
		pixval32f( dog_pyr[octv][intvl-1], r, c ) ) / 2.0;

	dI = cvCreateMat( 3, 1, CV_64FC1 );
	cvmSet( dI, 0, 0, dx );
	cvmSet( dI, 1, 0, dy );
	cvmSet( dI, 2, 0, ds );

	return dI;
}

/*
Computes the 3D Hessian matrix for a pixel in the DoG scale space pyramid.

@param dog_pyr DoG scale space pyramid
@param octv pixel's octave in dog_pyr
@param intvl pixel's interval in octv
@param r pixel's image row
@param c pixel's image col

@return Returns the Hessian matrix (below) for pixel I as a CvMat*

/ Ixx  Ixy  Ixs \ <BR>
| Ixy  Iyy  Iys | <BR>
\ Ixs  Iys  Iss /
*/
CvMat* hessian_3D( IplImage*** dog_pyr, int octv, int intvl, int r, int c )
{
	CvMat* H;
	double v, dxx, dyy, dss, dxy, dxs, dys;

	v = pixval32f( dog_pyr[octv][intvl], r, c );
	dxx = ( pixval32f( dog_pyr[octv][intvl], r, c+1 ) + 
		pixval32f( dog_pyr[octv][intvl], r, c-1 ) - 2 * v );
	dyy = ( pixval32f( dog_pyr[octv][intvl], r+1, c ) +
		pixval32f( dog_pyr[octv][intvl], r-1, c ) - 2 * v );
	dss = ( pixval32f( dog_pyr[octv][intvl+1], r, c ) +
		pixval32f( dog_pyr[octv][intvl-1], r, c ) - 2 * v );
	dxy = ( pixval32f( dog_pyr[octv][intvl], r+1, c+1 ) -
		pixval32f( dog_pyr[octv][intvl], r+1, c-1 ) -
		pixval32f( dog_pyr[octv][intvl], r-1, c+1 ) +
		pixval32f( dog_pyr[octv][intvl], r-1, c-1 ) ) / 4.0;
	dxs = ( pixval32f( dog_pyr[octv][intvl+1], r, c+1 ) -
		pixval32f( dog_pyr[octv][intvl+1], r, c-1 ) -
		pixval32f( dog_pyr[octv][intvl-1], r, c+1 ) +
		pixval32f( dog_pyr[octv][intvl-1], r, c-1 ) ) / 4.0;
	dys = ( pixval32f( dog_pyr[octv][intvl+1], r+1, c ) -
		pixval32f( dog_pyr[octv][intvl+1], r-1, c ) -
		pixval32f( dog_pyr[octv][intvl-1], r+1, c ) +
		pixval32f( dog_pyr[octv][intvl-1], r-1, c ) ) / 4.0;

	H = cvCreateMat( 3, 3, CV_64FC1 );
	cvmSet( H, 0, 0, dxx );
	cvmSet( H, 0, 1, dxy );
	cvmSet( H, 0, 2, dxs );
	cvmSet( H, 1, 0, dxy );
	cvmSet( H, 1, 1, dyy );
	cvmSet( H, 1, 2, dys );
	cvmSet( H, 2, 0, dxs );
	cvmSet( H, 2, 1, dys );
	cvmSet( H, 2, 2, dss );

	return H;
}

/*
Calculates interpolated pixel contrast.  Based on Eqn. (3) in Lowe's paper.

@param dog_pyr difference of Gaussians scale space pyramid
@param octv octave of scale space
@param intvl within-octave interval
@param r pixel row
@param c pixel column
@param xi interpolated subpixel increment to interval
@param xr interpolated subpixel increment to row
@param xc interpolated subpixel increment to col

@param Returns interpolated contrast.
*/
double interp_contr( IplImage*** dog_pyr, int octv, int intvl, int r,
					int c, double xi, double xr, double xc )
{
	CvMat* dD, X, T;
	double t[1], x[3] = { xc, xr, xi };

	cvInitMatHeader( &X, 3, 1, CV_64FC1, x, CV_AUTOSTEP );
	cvInitMatHeader( &T, 1, 1, CV_64FC1, t, CV_AUTOSTEP );
	dD = deriv_3D( dog_pyr, octv, intvl, r, c );
	cvGEMM( dD, &X, 1, NULL, 0, &T,  CV_GEMM_A_T );
	cvReleaseMat( &dD );

	return pixval32f( dog_pyr[octv][intvl], r, c ) + t[0] * 0.5;
}

/*
Allocates and initializes a new feature

@return Returns a pointer to the new feature
*/
struct SIFT_feature_unit* new_feature( void )
{
	struct SIFT_feature_unit* feature;
	struct detection_data* ddata;

	feature = (struct SIFT_feature_unit*)malloc( sizeof( struct SIFT_feature_unit ) );
	memset( feature, 0, sizeof( struct SIFT_feature_unit ) );
	ddata = (struct detection_data*)malloc( sizeof( struct detection_data ) );
	memset( ddata, 0, sizeof( struct detection_data ) );
	feature->feature_data = ddata;
	feature->type = FEATURE_LOWE;

	return feature;
}

/*
Determines whether a feature is too edge like to be stable by computing the
ratio of principal curvatures at that feature.  Based on Section 4.1 of
Lowe's paper.

@param dog_img image from the DoG pyramid in which feature was detected
@param r feature row
@param c feature col
@param curv_thr high threshold on ratio of principal curvatures

@return Returns 0 if the feature at (r,c) in dog_img is sufficiently
corner-like or 1 otherwise.
*/
int is_too_edge_like( IplImage* dog_img, int r, int c, int curv_thr )
{
	double d, dxx, dyy, dxy, tr, det;

	/* principal curvatures are computed using the trace and det of Hessian */
	d = pixval32f(dog_img, r, c);
	dxx = pixval32f( dog_img, r, c+1 ) + pixval32f( dog_img, r, c-1 ) - 2 * d;
	dyy = pixval32f( dog_img, r+1, c ) + pixval32f( dog_img, r-1, c ) - 2 * d;
	dxy = ( pixval32f(dog_img, r+1, c+1) - pixval32f(dog_img, r+1, c-1) -
		pixval32f(dog_img, r-1, c+1) + pixval32f(dog_img, r-1, c-1) ) / 4.0;
	tr = dxx + dyy;
	det = dxx * dyy - dxy * dxy;

	/* negative determinant -> curvatures have different signs; reject feature */
	if( det <= 0 )
		return 1;

	if( tr * tr / det < ( curv_thr + 1.0 )*( curv_thr + 1.0 ) / curv_thr )
		return 0;
	return 1;
}

/*
Calculates characteristic scale for each feature in an array.

@param features array of features
@param sigma amount of Gaussian smoothing per octave of scale space
@param intvls intervals per octave of scale space
*/
void SIFT_feature::calc_feature_scales( double sigma, int intvls )
{
	struct detection_data* ddata;
	double intvl;
//	int i, n;

	//n = feat[0]->total;
	for( unsigned i = 0; i < feat.size(); i++ )
	{
		//feat = CV_GET_SEQ_ELEM( struct feature, features, i );
		ddata = (detection_data* )feat[i].feature_data;
		intvl = ddata->intvl + ddata->subintvl;
		feat[i].scl = sigma * pow( 2.0, ddata->octv + intvl / intvls );
		ddata->scl_octv = sigma * pow( 2.0, intvl / intvls );
	}
}

/*
Halves feature coordinates and scale in case the input image was doubled
prior to scale space construction.

@param features array of features
*/
void SIFT_feature::adjust_for_img_dbl()
{
	
//	int i, n;

	//n = feat[0]->total;
	for( unsigned i = 0; i < feat.size(); i++ )
	{
		//feat = CV_GET_SEQ_ELEM( struct feature, features, i );
		feat[i].x /= 2.0;
		feat[i].y /= 2.0;
		feat[i].scl /= 2.0;
		feat[i].img_pt.x /= 2.0;
		feat[i].img_pt.y /= 2.0;
	}
}

/*
Computes a canonical orientation for each image feature in an array.  Based
on Section 5 of Lowe's paper.  This function adds features to the array when
there is more than one dominant orientation at a given feature location.

@param features an array of image features
@param gauss_pyr Gaussian scale space pyramid
*/
void SIFT_feature::calc_feature_oris( IplImage*** gauss_pyr )
{
	//struct feature* feat;
	struct SIFT_feature_unit* feature;
	struct detection_data* ddata;
	double* hist;
	double omax;
	//int i, j;

	for( unsigned i = 0; i < feat.size(); i++ )
	{
		feature = (struct SIFT_feature_unit*)malloc( sizeof( struct SIFT_feature_unit ) );
		*feature = feat.back();
		ddata = (detection_data* )feat.back().feature_data;
		feat.pop_back();
		hist = ori_hist( gauss_pyr[ddata->octv][ddata->intvl],
			ddata->r, ddata->c, SIFT_ORI_HIST_BINS,
			cvRound( SIFT_ORI_RADIUS * ddata->scl_octv ),
			SIFT_ORI_SIG_FCTR * ddata->scl_octv );
		for( unsigned j = 0; j < SIFT_ORI_SMOOTH_PASSES; j++ )
			smooth_ori_hist( hist, SIFT_ORI_HIST_BINS );
		omax = dominant_ori( hist, SIFT_ORI_HIST_BINS );
		add_good_ori_features(  hist, SIFT_ORI_HIST_BINS,
			omax * SIFT_ORI_PEAK_RATIO, feature );
		free( ddata );
		free( feature );
		free( hist );
	}
}

/*
Computes a gradient orientation histogram at a specified pixel.

@param img image
@param r pixel row
@param c pixel col
@param n number of histogram bins
@param rad radius of region over which histogram is computed
@param sigma std for Gaussian weighting of histogram entries

@return Returns an n-element array containing an orientation histogram
representing orientations between 0 and 2 PI.
*/
double* ori_hist( IplImage* img, int r, int c, int n, int rad, double sigma)
{
	double* hist;
	double mag, ori, w, exp_denom, PI2 = CV_PI * 2.0;
	int bin, i, j;

	hist = (double*)calloc( n, sizeof( double ) );
	exp_denom = 2.0 * sigma * sigma;
	for( i = -rad; i <= rad; i++ )
		for( j = -rad; j <= rad; j++ )
			if( calc_grad_mag_ori( img, r + i, c + j, &mag, &ori ) )
			{
				w = exp( -( i*i + j*j ) / exp_denom ); // why
				bin = cvRound( n * ( ori + CV_PI ) / PI2 );
				bin = ( bin < n )? bin : 0;
				hist[bin] += w * mag;
			}

			return hist;
}



/*
Calculates the gradient magnitude and orientation at a given pixel.

@param img image
@param r pixel row
@param c pixel col
@param mag output as gradient magnitude at pixel (r,c)
@param ori output as gradient orientation at pixel (r,c)

@return Returns 1 if the specified pixel is a valid one and sets mag and
ori accordingly; otherwise returns 0
*/
int calc_grad_mag_ori( IplImage* img, int r, int c, double* mag, double* ori )
{
	double dx, dy;

	if( r > 0  &&  r < img->height - 1  &&  c > 0  &&  c < img->width - 1 )
	{
		dx = pixval32f( img, r, c+1 ) - pixval32f( img, r, c-1 );
		dy = pixval32f( img, r-1, c ) - pixval32f( img, r+1, c );
		*mag = sqrt( dx*dx + dy*dy );
		*ori = atan2( dy, dx );
		return 1;
	}

	else
		return 0;
}



/*
Gaussian smooths an orientation histogram.

@param hist an orientation histogram
@param n number of bins
*/
void smooth_ori_hist( double* hist, int n )
{
	double prev, tmp, h0 = hist[0];
	int i;

	prev = hist[n-1];
	for( i = 0; i < n; i++ )
	{
		tmp = hist[i];
		hist[i] = 0.25 * prev + 0.5 * hist[i] + 
			0.25 * ( ( i+1 == n )? h0 : hist[i+1] );
		prev = tmp;
	}
}



/*
Finds the magnitude of the dominant orientation in a histogram

@param hist an orientation histogram
@param n number of bins

@return Returns the value of the largest bin in hist
*/
double dominant_ori( double* hist, int n )
{
	double omax;
	int maxbin, i;

	omax = hist[0];
	maxbin = 0;
	for( i = 1; i < n; i++ )
		if( hist[i] > omax )
		{
			omax = hist[i];
			maxbin = i;
		}
		return omax;
}



/*
Interpolates a histogram peak from left, center, and right values
*/
#define interp_hist_peak( l, c, r ) ( 0.5 * ((l)-(r)) / ((l) - 2.0*(c) + (r)) )



/*
Adds features to an array for every orientation in a histogram greater than
a specified threshold.

@param features new features are added to the end of this array
@param hist orientation histogram
@param n number of bins in hist
@param mag_thr new features are added for entries in hist greater than this
@param feat new features are clones of this with different orientations
*/
void SIFT_feature::add_good_ori_features( double* hist, int n,
						   double mag_thr, struct SIFT_feature_unit* feature )
{
	struct SIFT_feature_unit* new_feat;
	double bin, PI2 = CV_PI * 2.0;
	int l, r;
	
	for( unsigned i = 0; i < n; i++ )
	{
		l = ( i == 0 )? n - 1 : i-1;
		r = ( i + 1 ) % n;

		if( hist[i] > hist[l]  &&  hist[i] > hist[r]  &&  hist[i] >= mag_thr )
		{
			bin = i + interp_hist_peak( hist[l], hist[i], hist[r] );
			bin = ( bin < 0 )? n + bin : ( bin >= n )? bin - n : bin;
			new_feat = clone_feature( feature );
			new_feat->ori = ( ( PI2 * bin ) / n ) - CV_PI;
			feat.insert(feat.begin(),*new_feat);
			free( new_feat );
		}
	}
}


/*
Makes a deep copy of a feature

@param feat feature to be cloned

@return Returns a deep copy of feat
*/
struct SIFT_feature_unit* clone_feature( struct SIFT_feature_unit* feature )
{
	struct SIFT_feature_unit* new_feat;
	struct detection_data* ddata;

	new_feat = new_feature();
	ddata = (detection_data* )new_feat->feature_data;
	memcpy( new_feat, feature, sizeof( struct SIFT_feature_unit ) );
	memcpy( ddata, (detection_data*)feature->feature_data, sizeof( struct detection_data ) );
	new_feat->feature_data = ddata;

	return new_feat;
}

/*
Computes feature descriptors for features in an array.  Based on Section 6
of Lowe's paper.

@param features array of features
@param gauss_pyr Gaussian scale space pyramid
@param d width of 2D array of orientation histograms
@param n number of bins per orientation histogram
*/
void SIFT_feature::compute_descriptors( IplImage*** gauss_pyr, int d, int n)
{
	struct SIFT_feature_unit* feature;
	struct detection_data* ddata;
	double*** hist;
//	int i, k = features->total;

	for( unsigned i = 0; i < feat.size(); i++ )
	{
		//feat = CV_GET_SEQ_ELEM( struct feature, features, i );
		ddata = (detection_data* )feat[i].feature_data;
		hist = descr_hist( gauss_pyr[ddata->octv][ddata->intvl], ddata->r,
			ddata->c, feat[i].ori, ddata->scl_octv, d, n );
		hist_to_descr( hist, d, n, &feat[i] );
		release_descr_hist( &hist, d );
	}
}

/*
Computes the 2D array of orientation histograms that form the feature
descriptor.  Based on Section 6.1 of Lowe's paper.

@param img image used in descriptor computation
@param r row coord of center of orientation histogram array
@param c column coord of center of orientation histogram array
@param ori canonical orientation of feature whose descr is being computed
@param scl scale relative to img of feature whose descr is being computed
@param d width of 2d array of orientation histograms
@param n bins per orientation histogram

@return Returns a d x d array of n-bin orientation histograms.
*/
double*** descr_hist( IplImage* img, int r, int c, double ori,
					 double scl, int d, int n )
{
	double*** hist;
	double cos_t, sin_t, hist_width, exp_denom, r_rot, c_rot, grad_mag,
		grad_ori, w, rbin, cbin, obin, bins_per_rad, PI2 = 2.0 * CV_PI;
	int radius, i, j;

	hist = (double***)calloc( d, sizeof( double** ) );
	for( i = 0; i < d; i++ )
	{
		hist[i] = (double**)calloc( d, sizeof( double* ) );
		for( j = 0; j < d; j++ )
			hist[i][j] = (double*)calloc( n, sizeof( double ) );
	}

	cos_t = cos( ori );
	sin_t = sin( ori );
	bins_per_rad = n / PI2;
	exp_denom = d * d * 0.5;
	hist_width = SIFT_DESCR_SCL_FCTR * scl;
	radius = hist_width * sqrt(2.0) * ( d + 1.0 ) * 0.5 + 0.5;
	for( i = -radius; i <= radius; i++ )
		for( j = -radius; j <= radius; j++ )
		{
			/*
			Calculate sample's histogram array coords rotated relative to ori.
			Subtract 0.5 so samples that fall e.g. in the center of row 1 (i.e.
			r_rot = 1.5) have full weight placed in row 1 after interpolation.
			*/
			c_rot = ( j * cos_t - i * sin_t ) / hist_width;
			r_rot = ( j * sin_t + i * cos_t ) / hist_width;
			rbin = r_rot + d / 2 - 0.5;
			cbin = c_rot + d / 2 - 0.5;

			if( rbin > -1.0  &&  rbin < d  &&  cbin > -1.0  &&  cbin < d )
				if( calc_grad_mag_ori( img, r + i, c + j, &grad_mag, &grad_ori ))
				{
					grad_ori -= ori;
					while( grad_ori < 0.0 )
						grad_ori += PI2;
					while( grad_ori >= PI2 )
						grad_ori -= PI2;

					obin = grad_ori * bins_per_rad;
					w = exp( -(c_rot * c_rot + r_rot * r_rot) / exp_denom );
					interp_hist_entry( hist, rbin, cbin, obin, grad_mag * w, d, n );
				}
		}

		return hist;
}

/*
Interpolates an entry into the array of orientation histograms that form
the feature descriptor.

@param hist 2D array of orientation histograms
@param rbin sub-bin row coordinate of entry
@param cbin sub-bin column coordinate of entry
@param obin sub-bin orientation coordinate of entry
@param mag size of entry
@param d width of 2D array of orientation histograms
@param n number of bins per orientation histogram
*/
void interp_hist_entry( double*** hist, double rbin, double cbin,
					   double obin, double mag, int d, int n )
{
	double d_r, d_c, d_o, v_r, v_c, v_o;
	double** row, * h;
	int r0, c0, o0, rb, cb, ob, r, c, o;

	r0 = cvFloor( rbin );
	c0 = cvFloor( cbin );
	o0 = cvFloor( obin );
	d_r = rbin - r0;
	d_c = cbin - c0;
	d_o = obin - o0;

	/*
	The entry is distributed into up to 8 bins.  Each entry into a bin
	is multiplied by a weight of 1 - d for each dimension, where d is the
	distance from the center value of the bin measured in bin units.
	*/
	for( r = 0; r <= 1; r++ )
	{
		rb = r0 + r;
		if( rb >= 0  &&  rb < d )
		{
			v_r = mag * ( ( r == 0 )? 1.0 - d_r : d_r );
			row = hist[rb];
			for( c = 0; c <= 1; c++ )
			{
				cb = c0 + c;
				if( cb >= 0  &&  cb < d )
				{
					v_c = v_r * ( ( c == 0 )? 1.0 - d_c : d_c );
					h = row[cb];
					for( o = 0; o <= 1; o++ )
					{
						ob = ( o0 + o ) % n;
						v_o = v_c * ( ( o == 0 )? 1.0 - d_o : d_o );
						h[ob] += v_o;
					}
				}
			}
		}
	}
}

/*
Converts the 2D array of orientation histograms into a feature's descriptor
vector.

@param hist 2D array of orientation histograms
@param d width of hist
@param n bins per histogram
@param feat feature into which to store descriptor
*/
void hist_to_descr( double*** hist, int d, int n, struct SIFT_feature_unit* feat )
{
	int int_val, i, r, c, o, k = 0;

	for( r = 0; r < d; r++ )
		for( c = 0; c < d; c++ )
			for( o = 0; o < n; o++ )
				feat->descr[k++] = hist[r][c][o];

	feat->d = k;
	normalize_descr( feat );
	for( i = 0; i < k; i++ )
		if( feat->descr[i] > SIFT_DESCR_MAG_THR )
			feat->descr[i] = SIFT_DESCR_MAG_THR;
	normalize_descr( feat );

	/* convert floating-point descriptor to integer valued descriptor */
	for( i = 0; i < k; i++ )
	{
		int_val = SIFT_INT_DESCR_FCTR * feat->descr[i];
		feat->descr[i] = MIN( 255, int_val );
	}
}

/*
Normalizes a feature's descriptor vector to unitl length

@param feat feature
*/
void normalize_descr( struct SIFT_feature_unit* feat )
{
	double cur, len_inv, len_sq = 0.0;
	int i, d = feat->d;

	for( i = 0; i < d; i++ )
	{
		cur = feat->descr[i];
		len_sq += cur*cur;
	}
	len_inv = 1.0 / sqrt( len_sq );
	for( i = 0; i < d; i++ )
		feat->descr[i] *= len_inv;
}

/*
Compares features for a decreasing-scale ordering.  Intended for use with
CvSeqSort

@param feat1 first feature
@param feat2 second feature
@param param unused

@return Returns 1 if feat1's scale is greater than feat2's, -1 if vice versa,
and 0 if their scales are equal
*/
bool feature_cmp( struct SIFT_feature_unit feat1, struct SIFT_feature_unit feat2)
{
	/*struct SIFT_feature_unit* f1 = (struct SIFT_feature_unit*) feat1;
	struct SIFT_feature_unit* f2 = (struct SIFT_feature_unit*) feat2;*/

	/*if( f1->scl < f2->scl )
		return 1;
	if( f1->scl > f2->scl )
		return -1;
	return 0;*/
	return (feat1.scl<feat2.scl);
}

/*
De-allocates memory held by a descriptor histogram

@param hist pointer to a 2D array of orientation histograms
@param d width of hist
*/
void release_descr_hist( double**** hist, int d )
{
	int i, j;

	for( i = 0; i < d; i++)
	{
		for( j = 0; j < d; j++ )
			free( (*hist)[i][j] );
		free( (*hist)[i] );
	}
	free( *hist );
	*hist = NULL;
}

/*
De-allocates memory held by a scale space pyramid

@param pyr scale space pyramid
@param octvs number of octaves of scale space
@param n number of images per octave
*/
void release_pyr( IplImage**** pyr, int octvs, int n )
{
	int i, j;
	for( i = 0; i < octvs; i++ )
	{
		for( j = 0; j < n; j++ )
			cvReleaseImage( &(*pyr)[i][j] );
		free( (*pyr)[i] );
	}
	free( *pyr );
	*pyr = NULL;
}
//////////////////////////////////////////////////////////////////////////
// Convert Image with bounded rect
//////////////////////////////////////////////////////////////////////////
void ConvertImage(IplImage* source,IplImage* target,Rect Roi)
{
	
	if (source->imageSize < target->imageSize)
	{
		for (int nLine = Roi.upper; nLine < Roi.upper + Roi.height; nLine++)
		{
			uchar* ptr_source = (uchar*)(source->imageData + (nLine-Roi.upper) * source->widthStep);
			uchar* ptr_destination = (uchar*)(target->imageData + nLine * target->widthStep);
			for (int nCol = Roi.left; nCol < Roi.left + Roi.width; nCol++)
			{
				ptr_destination[3*(nCol)+0] = ptr_source[3*(nCol-Roi.left)+0];
				ptr_destination[3*(nCol)+1] = ptr_source[3*(nCol-Roi.left)+1];
				ptr_destination[3*(nCol)+2] = ptr_source[3*(nCol-Roi.left)+2];
			}
		}


	} 
	else
	{
		for (int nLine = Roi.upper; nLine < Roi.upper + Roi.height; nLine++)
		{
			uchar* ptr_source = (uchar*)(source->imageData + nLine * source->widthStep);
			uchar* ptr_destination = (uchar*)(target->imageData + (nLine - Roi.upper) * target->widthStep);
			for (int nCol = Roi.left; nCol < Roi.left + Roi.width; nCol++)
			{
				ptr_destination[3*(nCol-Roi.left)+0] = ptr_source[3*nCol+0];
				ptr_destination[3*(nCol-Roi.left)+1] = ptr_source[3*nCol+1];
				ptr_destination[3*(nCol-Roi.left)+2] = ptr_source[3*nCol+2];
			}
		}

	}
	
}