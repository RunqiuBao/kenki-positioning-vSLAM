/**
*
* Runqiu Bao (University of Tokyo)
* For more information see <https://github.com/RunqiuBao/kenki-posi>
*
*/
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#ifndef MYSTRUCT_H
#define MYSTRUCT_H

struct bbox{
    int lux;//左上角的点
    int luy;//左上角的点
    int rlx;//右下角的点
    int rly;//右下角的点
};

struct MaskSet{//runqiu: the masks set for one frame
    std::vector<std::string> labels;
    std::vector<bbox> bboxes;
};

#endif
