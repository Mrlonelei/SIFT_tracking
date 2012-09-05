// stdafx.h : ��׼ϵͳ�����ļ��İ����ļ���
// ���Ǿ���ʹ�õ��������ĵ�
// �ض�����Ŀ�İ����ļ�
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: �ڴ˴����ó�����Ҫ������ͷ�ļ�
#include "opencv2\opencv.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv\cv.h"
#include "opencv\highgui.h"

#include "Patches.h"
#include "ImageRepresentation.h"
#include "ImageSource.h"
#include "ImageSourceDir.h"
#include "ImageHandler.h"
#include "ImageSourceUSBCam.h"
#include "ImageSourceAVIFile.h"
//#include "imgfeatures.h"
//#include "sift.h"
//#include "utils.h"
#include "Def.h"
#include "SIFTBoostingTracker.h"
#include "SIFT_navie_tracker.h"
#include "SIFT_opt_tracker.h"
#include "kdtree.h"
#include "minpq.h"

#include <iostream>
#include <fstream>
#include <stdio.h>

#include <vector>
#if OS_type==2
#include <conio.h>
#include <io.h>   
#endif

#include <sys/types.h> 
#include <sys/stat.h> 
#include <math.h>
#include <string.h>
#include <time.h>