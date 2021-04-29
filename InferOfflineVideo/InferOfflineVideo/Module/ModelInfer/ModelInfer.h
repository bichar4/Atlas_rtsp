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

#ifndef MODEL_INFER_H
#define MODEL_INFER_H

#include <queue>
#include <sys/time.h>
#include "ModuleManager/ModuleManager.h"
#include "ModelProcess/ModelProcess.h"
#include "ConfigParser/ConfigParser.h"
#include "DvppCommon/DvppCommon.h"
#include "DataType/DataType.h"
#include "acl/acl.h"
#include "StreamPuller/StreamPuller.h"

// Definition of input image info array index
enum {
    MODEL_HEIGHT_INDEX = 0,
    MODEL_WIDTH_INDEX,
    IMAGE_HEIGHT_INDEX,
    IMAGE_WIDTH_INDEX,
    IMAGE_INFO_ARRAY_SIZE // size of input image info array in yolo
};

struct ModelBufferSize {
    static int outputSize_;
    static std::vector<size_t> bufferSize_;
};

class ModelInfer : public ascendBaseModule::ModuleBase {
public:
    ModelInfer();
    ~ModelInfer();
    APP_ERROR Init(ConfigParser &configParser, ascendBaseModule::ModuleInitArgs &initArgs);
    APP_ERROR DeInit(void);

protected:
    APP_ERROR Process(std::shared_ptr<void> inputData);

private:
    APP_ERROR InputBuffMalloc(std::shared_ptr<DvppDataInfo> &vpcData, std::vector<void *> &inputDataBuffers,
        std::vector<size_t> &buffersSize, std::shared_ptr<void>& yoloInfo);
    APP_ERROR ParseConfig(ConfigParser &configParser);

    APP_ERROR YoloProcess(uint32_t channelId, uint32_t frameId, std::shared_ptr<DeviceStreamData> &dataToSend,
        std::shared_ptr<DvppDataInfo> &vpcData, std::vector<void *> &outBuf,
        std::vector<size_t> &outSizes, std::vector<RawData> &modelOutput);
private:
    int deviceId_ = 0;
    uint32_t modelWidth_ = 0;
    uint32_t modelHeight_ = 0;
    uint32_t srcImageWidth_ = 0;
    uint32_t srcImageHeight_ = 0;
    uint32_t modelType_ = 0;

    std::string modelName_ = "";
    std::string modelPath_ = "";
    ModelProcess* modelProcess_ = nullptr;

    std::queue<std::vector<void *>> buffers_;
};

MODULE_REGIST(ModelInfer)

#endif