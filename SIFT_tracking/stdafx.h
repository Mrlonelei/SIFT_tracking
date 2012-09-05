// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: 在此处引用程序需要的其他头文件
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