/*
 * Copyright (c) 2020.Huawei Technologies Co., Ltd. All rights reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef YOLOV3POST_H
#define YOLOV3POST_H

#include <algorithm>
#include <vector>
#include <memory>
#include "DataType/DataType.h"

const int CLASS_NUM = 80;
const int DEPTH = 255; // (4(box: x, y, h, w) + 1(confidence) + 80(classNum)) * 3(anchorNum)
const int BIASES_NUM = 18; // Yolov3 anchors, generate from train data, coco dataset
const float BIASES[BIASES_NUM] = {10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326};
const float SCORE_THRESH = 0.3; // Threshold of confidence
const float OBJECTNESS_THRESH = 0.3; // Threshold of objectness value
const float IOU_THRESH = 0.45; // Non-Maximum Suppression threshold
const float COORDINATE_PARAM = 2.0;
const int YOLO_TYPE = 3;
const int ANCHOR_DIM = 3;
const int BOX_DIM = 4;

struct OutputLayer {
    int layerIdx;
    int width;
    int height;
    float anchors[6];
};

struct NetInfo {
    int anchorDim;
    int classNum;
    int bboxDim;
    int netWidth;
    int netHeight;
    std::vector<OutputLayer> outputLayers;
};

// Box information
struct DetectBox {
    float prob;
    int classID;
    float x;
    float y;
    float width;
    float height;
};

// Detect Info which could be transformed by DetectBox
struct ObjDetectInfo {
    float leftTopX;
    float leftTopY;
    float rightBotX;
    float rightBotY;
    float confidence;
    float classId;
};

// Realize the Yolo layer to get detiction object info
void Yolov3DetectionOutput(std::vector<std::shared_ptr<void>> featLayerData,
                           std::vector<ObjDetectInfo> &objInfos,
                           YoloImageInfo imgInfo);

#endif
