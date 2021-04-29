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

#ifndef INC_STREAMPULLER_H
#define INC_STREAMPULLER_H

#include "acl/acl.h"
#include "ErrorCode/ErrorCode.h"
#include "ModuleManager/ModuleManager.h"
#include "ConfigParser/ConfigParser.h"
#include "DataType/DataType.h"

extern "C" {
#include "libavformat/avformat.h"
}

class StreamPuller : public ascendBaseModule::ModuleBase {
public:
    StreamPuller();
    ~StreamPuller();
    APP_ERROR Init(ConfigParser &configParser, ascendBaseModule::ModuleInitArgs &initArgs);
    APP_ERROR DeInit(void);

protected:
    APP_ERROR Process(std::shared_ptr<void> inputData);

private:
    APP_ERROR ParseConfig(ConfigParser &configParser);
    APP_ERROR StartStream();
    AVFormatContext *CreateFormatContext();
    APP_ERROR GetStreamInfo();
    void PullStreamDataLoop();

private:
    int videoStream_ = 0;
    FrameInfo frameInfo_;
    std::string streamName_;
    AVFormatContext *pFormatCtx_ = nullptr;
};

MODULE_REGIST(StreamPuller)

#endif
