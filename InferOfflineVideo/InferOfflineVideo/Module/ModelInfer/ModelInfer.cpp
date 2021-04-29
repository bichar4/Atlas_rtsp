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
#include "ModelInfer/ModelInfer.h"
#include "PostProcess/PostProcess.h"
#include "PostProcess/config.h"
#include "PostProcess/PracticalSocket.h"
#include "Singleton.h"
#include <sstream>
#include <atomic>
#include <sys/stat.h>
#include <sys/time.h>
#include "FileManager/FileManager.h"

using namespace ascendBaseModule;

int ModelBufferSize::outputSize_ = {};
std::vector<size_t> ModelBufferSize::bufferSize_ = {};
namespace
{
    const int YOLOV3_CAFFE = 0;
    const int YOLOV3_TF = 1;
    const int BUFFER_SZIE = 5;
}

ModelInfer::ModelInfer()
{
    isStop_ = false;
}

ModelInfer::~ModelInfer() {}

APP_ERROR ModelInfer::ParseConfig(ConfigParser &configParser)
{
    std::string itemCfgStr = moduleName_ + std::string(".modelWidth");
    APP_ERROR ret = configParser.GetUnsignedIntValue(itemCfgStr, modelWidth_);
    if (ret != APP_ERR_OK)
    {
        LogError << "ModelInfer[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }

    itemCfgStr = moduleName_ + std::string(".modelHeight");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, modelHeight_);
    if (ret != APP_ERR_OK)
    {
        LogError << "ModelInfer[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }

    itemCfgStr = moduleName_ + std::string(".modelType");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, modelType_);
    if (ret != APP_ERR_OK)
    {
        LogError << "ModelInfer[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }

    itemCfgStr = moduleName_ + std::string(".modelName");
    ret = configParser.GetStringValue(itemCfgStr, modelName_);
    if (ret != APP_ERR_OK)
    {
        LogError << "ModelInfer[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }

    itemCfgStr = moduleName_ + std::string(".modelPath");
    ret = configParser.GetStringValue(itemCfgStr, modelPath_);
    if (ret != APP_ERR_OK)
    {
        LogError << "ModelInfer[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }

    itemCfgStr = std::string("SystemConfig.deviceId");
    ret = configParser.GetIntValue(itemCfgStr, deviceId_);
    if (ret != APP_ERR_OK)
    {
        LogError << "ModelInfer[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }

    return ret;
}

APP_ERROR ModelInfer::Init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    LogDebug << "Begin to init instance " << initArgs.instanceId;
    AssignInitArgs(initArgs);

    int ret = ParseConfig(configParser);
    if (ret != APP_ERR_OK)
    {
        LogError << "ModelInfer[" << instanceId_ << "]: Fail to parse config params." << GetAppErrCodeInfo(ret) << ".";
        return ret;
    }

    modelProcess_ = new ModelProcess(deviceId_, modelName_);
    LogDebug << "modelPath_ = " << modelPath_;
    ret = modelProcess_->Init(modelPath_);
    if (ret != APP_ERR_OK)
    {
        LogError << "ModelInfer[" << instanceId_ << "]: Fail to init ModelProcess." << GetAppErrCodeInfo(ret) << ".";
        return ret;
    }

    aclmdlDesc *modelDesc = modelProcess_->GetModelDesc();
    size_t outputSize = aclmdlGetNumOutputs(modelDesc);
    ModelBufferSize::outputSize_ = outputSize;
    for (size_t i = 0; i < outputSize; i++)
    {
        size_t bufferSize = aclmdlGetOutputSizeByIndex(modelDesc, i);
        ModelBufferSize::bufferSize_.push_back(bufferSize);
    }

    for (size_t i = 0; i < BUFFER_SZIE; ++i)
    {
        std::vector<void *> temp;
        for (size_t j = 0; j < outputSize; ++j)
        {
            void *outputBuffer = nullptr;
            APP_ERROR ret = aclrtMalloc(&outputBuffer, ModelBufferSize::bufferSize_[j], ACL_MEM_MALLOC_NORMAL_ONLY);
            if (ret != APP_ERR_OK)
            {
                LogError << "Failed to malloc buffer, size is " << ModelBufferSize::bufferSize_[j];
                return ret;
            }
            temp.push_back(outputBuffer);
        }
        buffers_.push(temp);
    }
    return APP_ERR_OK;
}

APP_ERROR ModelInfer::Process(std::shared_ptr<void> inputData)
{
    std::shared_ptr<DvppDataInfoT> vpcData = std::static_pointer_cast<DvppDataInfoT>(inputData);
    if (vpcData->eof)
    {
        std::shared_ptr<CommonData> data = std::make_shared<CommonData>();
        data->channelId = vpcData->channelId;
        data->eof = true;
        SendToNextModule(MT_PostProcess, data, data->channelId);
        return APP_ERR_OK;
    }
    srcImageWidth_ = vpcData->srcImageWidth;
    srcImageHeight_ = vpcData->srcImageHeight;

    

    std::shared_ptr<DeviceStreamData> dataToSend = std::make_shared<DeviceStreamData>();
    std::vector<void *> outBuf;
    std::vector<size_t> outSizes;
    std::vector<RawData> modelOutput;

    APP_ERROR ret = YoloProcess(vpcData->channelId, vpcData->frameId,
                                dataToSend, vpcData->dvppData, outBuf, outSizes, modelOutput);
    if (ret != APP_ERR_OK)
    {
        acldvppFree(vpcData->dvppData->data);
        LogError << "Failed to YoloProcess, ret=" << ret;
        return ret;
    }


    //=======================
    //did thid so that image frame goes to another pipeline
    //acldvppFree(vpcData->dvppData->data);

    std::shared_ptr<CommonData> data = std::make_shared<CommonData>();
    data->eof = false;
    data->inferOutput = std::move(modelOutput);
    data->yoloImgInfo.modelWidth = modelWidth_;
    data->yoloImgInfo.modelHeight = modelHeight_;
    data->yoloImgInfo.imgWidth = vpcData->srcImageWidth;
    data->yoloImgInfo.imgHeight = vpcData->srcImageHeight;
    data->modelType = modelType_;
    data->channelId = vpcData->channelId;
    data->frameId = vpcData->frameId;
    data->dvppData = vpcData->dvppData;
    SendToNextModule(MT_PostProcess, data, data->channelId);
    return APP_ERR_OK;
}

APP_ERROR ModelInfer::DeInit(void)
{
    LogInfo << "ModelInfer[" << instanceId_ << "]: ModelInfer::begin to deinit.";

    // Release objects resource
    modelProcess_->DeInit();
    delete modelProcess_;

    while (!buffers_.empty())
    {
        std::vector<void *> buffer = buffers_.front();
        buffers_.pop();
        for (auto &j : buffer)
        {
            aclrtFree(j);
        }
    }
    LogInfo << "ModelInfer[" << instanceId_ << "]: ModelInfer deinit success.";
    return APP_ERR_OK;
}

/*
 * @description: YoloV3 inference process
 * @param channelId Channel Id of the video input stream
 * @param dataToSend The output data
 * @param vpcData The input data
 * @param outBuf Temporarily save YoloV3 model inference result
 * @param outSizes Temporarily save YoloV3 model inference result sizes
 * @param modelOutput Get output data from inference
 */
APP_ERROR ModelInfer::YoloProcess(uint32_t channelId, uint32_t frameId, std::shared_ptr<DeviceStreamData> &dataToSend,
                                  std::shared_ptr<DvppDataInfo> &vpcData, std::vector<void *> &outBuf,
                                  std::vector<size_t> &outSizes, std::vector<RawData> &modelOutput)
{
    std::vector<void *> inputDataBuffers;
    std::vector<size_t> buffersSize;
    std::shared_ptr<void> yoloInfo;

    APP_ERROR ret = InputBuffMalloc(vpcData, inputDataBuffers, buffersSize, yoloInfo);
    if (ret != APP_ERR_OK)
    {
        LogError << "Failed to execute InputBuffMalloc, ret = " << ret;
        return ret;
    }

    outBuf = buffers_.front();
    buffers_.pop();
    buffers_.push(outBuf);
    outSizes = ModelBufferSize::bufferSize_;

    dataToSend->channelId = channelId;
    dataToSend->framId = frameId;
    ret = modelProcess_->ModelInference(inputDataBuffers, buffersSize, outBuf, outSizes);
    if (ret != APP_ERR_OK)
    {
        LogError << "Failed to execute ModelInference, ret = " << ret;
        return ret;
    }
    for (size_t i = 0; i < outBuf.size(); i++)
    {
        RawData rawDevData = RawData();
        rawDevData.data.reset(outBuf[i], [](void *) {});
        rawDevData.lenOfByte = outSizes[i];
        modelOutput.push_back(std::move(rawDevData));
    }

    return APP_ERR_OK;
}

/*
 * @description: Apply memory for model inference input Data
 * @param: vpcData The resize result to inference
 * @param: inputDataBuffers Input buffer
 * @param: buffersSize Input buffer sizes
 * @param yoloInfo The second input of the model, the memory needs to be released
 */
APP_ERROR ModelInfer::InputBuffMalloc(std::shared_ptr<DvppDataInfo> &vpcData, std::vector<void *> &inputDataBuffers,
                                      std::vector<size_t> &buffersSize, std::shared_ptr<void> &yoloInfo)
{
    inputDataBuffers.push_back(vpcData->data);
    buffersSize.push_back(vpcData->dataSize);
    if (modelType_ == YOLOV3_CAFFE)
    {
        float imgInfo[IMAGE_INFO_ARRAY_SIZE] = {0};
        imgInfo[MODEL_HEIGHT_INDEX] = modelHeight_;
        imgInfo[MODEL_WIDTH_INDEX] = modelWidth_;
        imgInfo[IMAGE_HEIGHT_INDEX] = srcImageHeight_;
        imgInfo[IMAGE_WIDTH_INDEX] = srcImageWidth_;
        void *yoloImgInfo = nullptr;
        uint32_t imgInfoInputSize = sizeof(float) * IMAGE_INFO_ARRAY_SIZE;
        APP_ERROR ret = acldvppMalloc(&yoloImgInfo, imgInfoInputSize);
        if (ret != APP_ERR_OK)
        {
            LogError << "Failed to malloc buffer, size is " << imgInfoInputSize << ", ret = " << ret;
            return ret;
        }
        yoloInfo.reset(yoloImgInfo, acldvppFree);
        ret = aclrtMemcpy((uint8_t *)yoloImgInfo, imgInfoInputSize, imgInfo, imgInfoInputSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != APP_ERR_OK)
        {
            LogError << "Failed to execute aclrtMemcpy, ret = " << ret;
            return ret;
        }
        inputDataBuffers.push_back(yoloImgInfo);
        buffersSize.push_back(sizeof(float) * IMAGE_INFO_ARRAY_SIZE);
    }
    return APP_ERR_OK;
}
