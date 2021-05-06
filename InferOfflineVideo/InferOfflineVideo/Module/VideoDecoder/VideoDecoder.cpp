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

#include <iostream>

#include "ErrorCode/ErrorCode.h"
#include "VideoDecoder/VideoDecoder.h"
#include "Log/Log.h"
#include "FileManager/FileManager.h"
#include "ModelInfer/ModelInfer.h"
#include <sys/time.h>

using namespace ascendBaseModule;

namespace {
    const int CALLBACK_TRIGGER_TIME = 1000;
}

VideoDecoder::VideoDecoder()
{
    isStop_ = false;
}

VideoDecoder::~VideoDecoder() {}

void VideoDecoder::VideoDecoderCallBack(acldvppStreamDesc *input, acldvppPicDesc *output, void *userdata)
{
    void *dataDev = acldvppGetStreamDescData(input);
    APP_ERROR ret = (APP_ERROR)acldvppFree(dataDev);
    if (ret != APP_ERR_OK) {
        LogError << "fail to free input stream desc dataDev";
    }
    ret = (APP_ERROR)acldvppDestroyStreamDesc(input);
    if (ret != APP_ERR_OK) {
        LogError << "fail to destroy input stream desc";
    }

    DecodeInfo *decodeInfo = (DecodeInfo *)userdata;
    if (decodeInfo == nullptr) {
        LogError << "VideoDecoder: user data is nullptr";
        return;
    }
    VideoDecoder* videoDecoder = decodeInfo->videoDecoder;
    if (videoDecoder->frameId % videoDecoder->skipInterval_ == 0) {
        std::shared_ptr<DvppDataInfo> temp = std::make_shared<DvppDataInfo>();
        temp->height = decodeInfo->frameInfo.height;
        temp->width = decodeInfo->frameInfo.width;
        temp->heightStride = DVPP_ALIGN_UP(decodeInfo->frameInfo.height, VPC_STRIDE_HEIGHT);
        temp->widthStride = DVPP_ALIGN_UP(decodeInfo->frameInfo.width, VPC_STRIDE_WIDTH);
        temp->dataSize = (uint32_t)acldvppGetPicDescSize(output);
        temp->data = (uint8_t*)acldvppGetPicDescData(output);

        DvppDataInfo out;
        out.height = videoDecoder->resizeHeight_;
        out.width = videoDecoder->resizeWidth_;
        videoDecoder->vpcDvppCommon_->CombineResizeProcess(*temp, out, true, VPC_PT_FIT);
        temp = videoDecoder->vpcDvppCommon_->GetResizedImage();

        std::shared_ptr<DvppDataInfoT> toNext = std::make_shared<DvppDataInfoT>();
        toNext->eof = false;
        toNext->channelId = decodeInfo->frameInfo.channelId;
        toNext->srcImageWidth = decodeInfo->frameInfo.width;
        toNext->srcImageHeight = decodeInfo->frameInfo.height;
        toNext->frameId = videoDecoder->frameId;
        toNext->dvppData = std::move(temp);
        videoDecoder->SendToNextModule(MT_ModelInfer, toNext, toNext->channelId);
    }
    videoDecoder->frameId++;
    acldvppFree(acldvppGetPicDescData(output));
    ret = (APP_ERROR)acldvppDestroyPicDesc(output);
    if (ret != APP_ERR_OK) {
        LogError << "Fail to destroy pic desc";
    }
    delete decodeInfo;
}

void *VideoDecoder::DecoderThread(void *arg)
{
    VideoDecoder *videoDecoder = (VideoDecoder *)arg;
    if (videoDecoder == nullptr) {
        LogError << "arg is nullptr";
        return ((void *)(-1));
    }

    aclError ret = aclrtSetCurrentContext(videoDecoder->aclContext_);
    if (ret != APP_ERR_OK) {
        LogError << "Failed to set context, ret = " << ret;
        return ((void *)(-1));
    }

    LogInfo << "DecoderThread start";
    while (!videoDecoder->stopDecoderThread_) {
        (void)aclrtProcessReport(CALLBACK_TRIGGER_TIME);
    }
    return nullptr;
}

VdecConfig VideoDecoder::GetVdecConfig()
{
    VdecConfig vdecConfig;
    vdecConfig.inputWidth = streamWidth_;
    vdecConfig.inputHeight = streamHeight_;
    vdecConfig.outFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    vdecConfig.channelId = instanceId_;
    vdecConfig.threadId = decoderThreadId_;
    vdecConfig.callback = &VideoDecoder::VideoDecoderCallBack;
    return vdecConfig;
}

APP_ERROR VideoDecoder::Init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    LogDebug << "Begin to init instance " << initArgs.instanceId;

    AssignInitArgs(initArgs);

    int ret = ParseConfig(configParser);
    if (ret != APP_ERR_OK) {
        LogError << "VideoDecoder[" << instanceId_ << "]: Fail to parse config params." << GetAppErrCodeInfo(ret) <<
            ".";
        return ret;
    }

    int createThreadErr = pthread_create(&decoderThreadId_, nullptr, &VideoDecoder::DecoderThread, (void *)this);
    if (createThreadErr != 0) {
        LogError << "Failed to create thread, err = " << createThreadErr;
        return APP_ERR_ACL_FAILURE;
    }
    LogInfo <<"thread create ID = " << decoderThreadId_;

    ret = aclrtCreateStream(&vpcDvppStream_);
    if (ret != APP_ERR_OK) {
        LogError << "VideoDecoder[" << instanceId_ << "]: aclrtCreateStream failed, ret=" << ret << ".";
        return ret;
    }

    vpcDvppCommon_ = new DvppCommon(vpcDvppStream_);
    if (vpcDvppCommon_ == nullptr) {
        LogError << "create vpcDvppCommon_ Failed";
        return APP_ERR_COMM_ALLOC_MEM;
    }

    ret = vpcDvppCommon_->Init();
    if (ret != APP_ERR_OK) {
        delete vpcDvppCommon_;
        vpcDvppCommon_ = nullptr;
        LogError << "pDvpp_ Init Failed";
        return ret;
    }

    LogDebug << "VideoDecoder [" << instanceId_ << "] Init success";
    return APP_ERR_OK;
}

APP_ERROR VideoDecoder::ParseConfig(ConfigParser &configParser)
{
    std::string itemCfgStr = moduleName_ + std::string(".resizeWidth");
    APP_ERROR ret = configParser.GetUnsignedIntValue(itemCfgStr, resizeWidth_);
    if (ret != APP_ERR_OK) {
        LogError << "VideoDecoder[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }

    itemCfgStr = moduleName_ + std::string(".resizeWidth");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, resizeHeight_);
    if (ret != APP_ERR_OK) {
        LogError << "VideoDecoder[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }

    itemCfgStr = std::string("SystemConfig.deviceId");
    ret = configParser.GetIntValue(itemCfgStr, deviceId_);
    if (ret != APP_ERR_OK) {
        LogError << "VideoDecoder[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }

    itemCfgStr = std::string("skipInterval");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, skipInterval_);
    if (ret != APP_ERR_OK) {
        LogError << "VideoDecoder[" << instanceId_ << "]: Fail to get config variable named " << itemCfgStr << ".";
        return ret;
    }
    if (skipInterval_ == 0) {
        LogError << "The value of skipInterval_ must be greater than 0";
        return APP_ERR_ACL_FAILURE;
    }

    return ret;
}

APP_ERROR VideoDecoder::CreateVdecDvppCommon(acldvppStreamFormat format)
{
    auto vdecConfig = GetVdecConfig();
    vdecConfig.inFormat = format;
    vdecDvppCommon_ = new DvppCommon(vdecConfig);
    if (vdecDvppCommon_ == nullptr) {
        LogError << "create vdecDvppCommon_ Failed";
        return APP_ERR_COMM_ALLOC_MEM;
    }

    APP_ERROR ret = vdecDvppCommon_->InitVdec();
    if (ret != APP_ERR_OK) {
        delete vdecDvppCommon_;
        LogError << "vdecDvppCommon_ InitVdec Failed";
        return ret;
    }
    return ret;
}

APP_ERROR VideoDecoder::Process(std::shared_ptr<void> inputData)
{
    std::shared_ptr<FrameData> frameData = std::static_pointer_cast<FrameData>(inputData);
    if (frameData->frameInfo.eof) {
        APP_ERROR ret = vdecDvppCommon_->VdecSendEosFrame();
        if (ret != APP_ERR_OK) {
            LogError << "Failed to send eos frame, ret = " << ret;
            return ret;
        }
        std::shared_ptr<DvppDataInfoT> toNext = std::make_shared<DvppDataInfoT>();
        toNext->eof = true;
        toNext->channelId = frameData->frameInfo.channelId;
        SendToNextModule(MT_ModelInfer, toNext, toNext->channelId);
        return APP_ERR_OK;
    }
    streamWidth_ = frameData->frameInfo.width;
    streamHeight_ = frameData->frameInfo.height;

    if (vdecDvppCommon_ == nullptr) {
        APP_ERROR ret = CreateVdecDvppCommon(frameData->frameInfo.format);
        if (ret != APP_ERR_OK) {
            LogError << "CreateVdecDvppCommon Failed";
            return ret;
        }
    }

    std::shared_ptr<DvppDataInfo> vdecData = std::make_shared<DvppDataInfo>();
    vdecData->dataSize = frameData->streamData.size;
    vdecData->data = (uint8_t *)frameData->streamData.data.get();

    DecodeInfo *decodeInfo = new DecodeInfo();
    decodeInfo->frameInfo = frameData->frameInfo;
    decodeInfo->videoDecoder = this;

    APP_ERROR ret = vdecDvppCommon_->CombineVdecProcess(vdecData, decodeInfo);
    if (ret != APP_ERR_OK) {
        LogError << "Failed to do VdecProcess, ret = " << ret;
        return ret;
    }

    return APP_ERR_OK;
}

APP_ERROR VideoDecoder::DeInit(void)
{
    LogDebug << "VideoDecoder [" << instanceId_ << "] begin to deinit";

    if (vdecDvppCommon_) {
        APP_ERROR ret = vdecDvppCommon_->DeInit();
        if (ret != APP_ERR_OK) {
            LogError << "Failed to deinitialize vdecDvppCommon, ret = " << ret;
            return ret;
        }
        delete vdecDvppCommon_;
    }

    stopDecoderThread_ = true;
    pthread_join(decoderThreadId_, NULL);

    if (vpcDvppCommon_) {
        APP_ERROR ret = vpcDvppCommon_->DeInit();
        if (ret != APP_ERR_OK) {
            LogError << "Failed to deinitialize vpcDvppCommon, ret = " << ret;
            return ret;
        }
        delete vpcDvppCommon_;
    }

    if (vpcDvppStream_) {
        APP_ERROR ret = aclrtDestroyStream(vpcDvppStream_);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to destroy stream, ret = " << ret;
            return ret;
        }
        vpcDvppStream_ = nullptr;
    }

    LogDebug << "VideoDecoder [" << instanceId_ << "] deinit success.";
    return APP_ERR_OK;
}
