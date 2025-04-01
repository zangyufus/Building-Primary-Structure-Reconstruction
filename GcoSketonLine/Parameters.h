
//===========================================================================

//===========================================================================

#ifndef PARAM_H_
#define PARAM_H_

//common
#define LABEL_NUM 2  //two labels: 1 means inside, 0 means at the outside.
#define NEAREST_ZERO 0.0001
#define M_PI 3.1415926535897
#define DOUBLE_MAX_VAL 1e15
#define DOUBLE_MIN_VAL -1e15


//MRF&gco
#define VOXEL_SIZE 0.5
#define OPTIM_LAMBDA 1
#define NUMS_NEIGHBOR 4 //  邻域,设置邻域
#define MINPTS 5
#define LINENUMs_NEARBY 6 // 周围的线，2的倍数

//parameter calculate
#define TH_ANGLE 12.5//角度阈值
#define TH_DIST 0.2//线邻域点

//out
#define LINE_SAMPLE_INTERVAL 0.05

#endif