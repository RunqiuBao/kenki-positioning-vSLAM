/**
*
* Runqiu Bao (University of Tokyo)
* For more information see <https://github.com/RunqiuBao/kenki-posi>
*
*/
#include "MaskInfo.h"

#ifndef MYSTRUCT_H3
#define MYSTRUCT_H3

struct MaskSetSD{//runqiu: the masks set for one frame, including static or dynamic label for the objects
    std::vector<std::string> labels;
    std::vector<bbox> bboxes;
    std::vector<int> isdynamic;//runqiu:0 is static, 1 is dynamic, 2 is lost tracking or not in ref_frame, 3 is lost tracking background
};

#endif
