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

#include "StreamPuller/StreamPuller.h"

#include <chrono>
#include <iostream>
#include <atomic>
#include "Log/Log.h"
#include <unistd.h>
#include "VideoDecoder/VideoDecoder.h"
#include "Singleton.h"

using namespace ascendBaseModule;

namespace {
const int LOW_THRESHOLD = 128;
const int MAX_THRESHOLD = 4096;
}

using Time = std::chrono::high_resolution_clock;

StreamPuller::StreamPuller()
{
    withoutInputQueue_ = true;
}

StreamPuller::~StreamPuller() {}

APP_ERROR StreamPuller::ParseConfig(ConfigParser &configParser)
{
    LogDebug << "StreamPuller [" << instanceId_ << "]: begin to parse config values.";
    std::string itemCfgStr = std::string("stream.ch") + std::to_string(instanceId_);
    return configParser.GetStringValue(itemCfgStr, streamName_);
}

APP_ERROR StreamPuller::Init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    LogDebug << "Begin to init instance " << initArgs.instanceId;

    AssignInitArgs(initArgs);

    int ret = ParseConfig(configParser);
    if (ret != APP_ERR_OK) {
        LogError << "StreamPuller[" << instanceId_ << "]: Fail to parse config params." << GetAppErrCodeInfo(ret);
        return ret;
    }

    isStop_ = false;
    pFormatCtx_ = nullptr;

    LogDebug << "StreamPuller [" << instanceId_ << "] Init success.";
    return APP_ERR_OK;
}

APP_ERROR StreamPuller::DeInit(void)
{
    LogDebug << "StreamPuller [" << instanceId_ << "]: Deinit start.";

    // clear th cache of the queue
    avformat_close_input(&pFormatCtx_);

    isStop_ = true;
    pFormatCtx_ = nullptr;
    LogDebug << "StreamPuller [" << instanceId_ << "]: Deinit success.";
    return APP_ERR_OK;
}

APP_ERROR StreamPuller::Process(std::shared_ptr<void> inputData)
{
    int failureNum = 0;
    while (failureNum < 1) {
        StartStream();
        failureNum++;
    }
    return APP_ERR_OK;
}

APP_ERROR StreamPuller::StartStream()
{
    avformat_network_init(); // init network
    pFormatCtx_ = CreateFormatContext(); // create context
    if (pFormatCtx_ == nullptr) {
        LogError << "pFormatCtx_ null!";
        return APP_ERR_COMM_FAILURE;
    }
    // for debug dump
    av_dump_format(pFormatCtx_, 0, streamName_.c_str(), 0);

    // get stream infomation
    APP_ERROR ret = GetStreamInfo();
    if (ret != APP_ERR_OK) {
        LogError << "Stream Info Check failed, ret = " << ret;
        return APP_ERR_COMM_FAILURE;
    }

    LogInfo << "Start the stream......";
    PullStreamDataLoop(); // Cyclic stream pull

    return APP_ERR_OK;
}

APP_ERROR StreamPuller::GetStreamInfo()
{
    if (pFormatCtx_ != nullptr) {
        videoStream_ = -1;
        frameInfo_.frameId = 0;
        frameInfo_.channelId = instanceId_;
        AVCodecID codecId = pFormatCtx_->streams[0]->codecpar->codec_id;
        if (codecId == AV_CODEC_ID_H264) {
            frameInfo_.format = H264_MAIN_LEVEL;
        } else if (codecId == AV_CODEC_ID_H265) {
            frameInfo_.format = H265_MAIN_LEVEL;
        } else {
            LogError << "\033[0;31mError unsupported format \033[0m" << codecId;
            return APP_ERR_COMM_FAILURE;
        }

        for (unsigned int i = 0; i < pFormatCtx_->nb_streams; i++) {
            AVStream *inStream = pFormatCtx_->streams[i];
            if (inStream->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                videoStream_ = i;
                frameInfo_.height = inStream->codecpar->height;
                frameInfo_.width = inStream->codecpar->width;
                break;
            }
        }

        if (videoStream_ == -1) {
            LogError << "Didn't find a video stream!";
            return APP_ERR_COMM_FAILURE;
        }

        if (frameInfo_.height < LOW_THRESHOLD || frameInfo_.width < LOW_THRESHOLD ||
            frameInfo_.height > MAX_THRESHOLD || frameInfo_.width > MAX_THRESHOLD) {
            LogError << "Size of frame is not supported in DVPP Video Decode!";
            return APP_ERR_COMM_FAILURE;
        }
    }
    return APP_ERR_OK;
}

AVFormatContext *StreamPuller::CreateFormatContext()
{
    // create message for stream pull
    AVFormatContext *formatContext = nullptr;
    AVDictionary *options = nullptr;
    av_dict_set(&options, "rtsp_transport", "tcp", 0);
    av_dict_set(&options, "stimeout", "3000000", 0);
    int ret = avformat_open_input(&formatContext, streamName_.c_str(), nullptr, &options);
    if (options != nullptr) {
        av_dict_free(&options);
    }
    if (ret != 0) {
        LogError << "Couldn't open input stream" << streamName_.c_str() << ", ret=" << ret;
        return nullptr;
    }
    ret = avformat_find_stream_info(formatContext, nullptr);
    if (ret != 0) {
        LogError << "Couldn't find stream information, ret = " << ret;
        return nullptr;
    }
    return formatContext;
}

void StreamPuller::PullStreamDataLoop()
{
    // Pull data cyclically
    AVPacket pkt;
    auto startTime = Time::now();
    while (1) {
        if (isStop_ || pFormatCtx_ == nullptr) {
            break;
        }
        av_init_packet(&pkt);
        int ret = av_read_frame(pFormatCtx_, &pkt);
        if (ret != 0) {
            if (ret == AVERROR_EOF) {
                LogInfo << "StreamPuller [" << instanceId_ << "]: channel StreamPuller is EOF, exit";
                std::shared_ptr<FrameData> frameData = std::make_shared<FrameData>();
                frameData->frameInfo = frameInfo_;
                frameData->frameInfo.eof = true;
                SendToNextModule(MT_VideoDecoder, frameData, frameData->frameInfo.channelId);
                break;
            }
            LogInfo << "StreamPuller [" << instanceId_ << "]: channel Read frame failed, continue";
            av_packet_unref(&pkt);
            continue;
        } else if (pkt.stream_index == videoStream_) {
            if (pkt.size <= 0) {
                LogError << "Invalid pkt.size: " << pkt.size;
                av_packet_unref(&pkt);
                continue;
            }

            uint8_t* dataBuffer = (uint8_t *)malloc(pkt.size);
            std::copy(pkt.data, pkt.data + pkt.size, dataBuffer);
            std::shared_ptr<FrameData> frameData = std::make_shared<FrameData>();
            frameData->frameInfo = frameInfo_;
            frameData->frameInfo.eof = false;
            frameData->streamData.data.reset(dataBuffer, free);
            frameData->streamData.size = pkt.size;
            SendToNextModule(MT_VideoDecoder, frameData, frameData->frameInfo.channelId);
            frameInfo_.frameId++;
        }
        av_packet_unref(&pkt);
    }
}
