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

#ifndef VIDEO_DECODER_H
#define VIDEO_DECODER_H

#include "ModuleManager/ModuleManager.h"
#include "ConfigParser/ConfigParser.h"
#include "DvppCommon/DvppCommon.h"
#include "DataType/DataType.h"

class VideoDecoder : public ascendBaseModule::ModuleBase {
public:
    VideoDecoder();
    ~VideoDecoder();
    APP_ERROR Init(ConfigParser &configParser, ascendBaseModule::ModuleInitArgs &initArgs);
    APP_ERROR DeInit(void);

protected:
    APP_ERROR Process(std::shared_ptr<void> inputData);

private:
    APP_ERROR ParseConfig(ConfigParser &configParser);
    VdecConfig GetVdecConfig();
    static void *DecoderThread(void *arg);
    static void VideoDecoderCallBack(acldvppStreamDesc *input, acldvppPicDesc *output, void *userdata);
    APP_ERROR CreateVdecDvppCommon(acldvppStreamFormat format);

private:
    bool stopDecoderThread_ = false;

    int64_t frameId = 0;
    uint32_t streamWidth_ = 0;
    uint32_t streamHeight_ = 0;
    uint32_t resizeWidth_ = 0;
    uint32_t resizeHeight_ = 0;
    uint32_t skipInterval_ = 1;

    aclrtStream vpcDvppStream_ = nullptr;
    DvppCommon* vpcDvppCommon_ = nullptr;
    DvppCommon* vdecDvppCommon_ = nullptr;
    pthread_t decoderThreadId_;
};

struct DecodeInfo {
    VideoDecoder *videoDecoder = nullptr;
    FrameInfo frameInfo;
};

MODULE_REGIST(VideoDecoder)

#endif
