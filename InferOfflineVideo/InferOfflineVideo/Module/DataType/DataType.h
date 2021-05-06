/*
 * Copyright(C) 2020. Huawei Technologies Co.,Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INC_DATA_TYPE_H
#define INC_DATA_TYPE_H

#include "CommonDataType/CommonDataType.h"
#include "DvppCommon/DvppCommon.h"

struct FrameInfo {
    bool eof;
    uint32_t channelId;
    uint64_t frameId;
    uint32_t width;
    uint32_t height;
    acldvppStreamFormat format;
};

struct FrameData {
    FrameInfo frameInfo;
    StreamData streamData;
};

struct DvppDataInfoT {
    bool eof;
    uint32_t channelId;
    uint32_t frameId = 0;
    uint32_t srcImageWidth = 0;
    uint32_t srcImageHeight = 0;
    std::shared_ptr<DvppDataInfo> dvppData;
    std::shared_ptr<DvppDataInfo> fullFrame;
};

struct YoloImageInfo {
    int modelWidth;
    int modelHeight;
    int imgWidth;
    int imgHeight;
};

struct CommonData {
    bool eof;
    uint32_t channelId = 0;
    uint32_t frameId = 0;
    std::vector<RawData> inferOutput;
    YoloImageInfo yoloImgInfo;
    uint32_t modelType = 0;
    std::shared_ptr<DvppDataInfo> dvppData;
    std::shared_ptr<DvppDataInfo> fullFrame;
};

#endif