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

#include "PostProcess/PostProcess.h"
#include "PostProcess/PracticalSocket.h"
#include "PostProcess/config.h"
#include <sstream>
#include <atomic>
#include <sys/stat.h>
#include <sys/time.h>
#include "Singleton.h"
#include "FileManager/FileManager.h"


using namespace ascendBaseModule;

namespace {
    const int YOLOV3_CAFFE = 0;
    const int YOLOV3_TF = 1;
    const int BUFFER_SIZE = 5;
}

PostProcess::PostProcess()
{
    isStop_ = false;
}

PostProcess::~PostProcess() {}

APP_ERROR PostProcess::Init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    LogDebug << "Begin to init instance " << initArgs.instanceId;

    AssignInitArgs(initArgs);

    for (int i = 0; i < BUFFER_SIZE; ++i) {
        std::vector<void *> temp;
        for (size_t j = 0; j < ModelBufferSize::outputSize_; j++) {
            void *hostPtrBuffer = nullptr;
            APP_ERROR ret = (APP_ERROR)aclrtMallocHost(&hostPtrBuffer, ModelBufferSize::bufferSize_[j]);
            if (ret != APP_ERR_OK) {
                LogError << "Failed to malloc output buffer of model on host, ret = " << ret;
                return ret;
            }
            temp.push_back(hostPtrBuffer);
        }
        buffers_.push(temp);
    }

    return APP_ERR_OK;
}

void PostProcess::ConstructData(std::vector<ObjDetectInfo> &objInfos, std::shared_ptr<DeviceStreamData> &dataToSend)
{
    for (int k = 0; k < objInfos.size(); ++k) {
        ObjectDetectInfo detectInfo;
        detectInfo.location.leftTopX = objInfos[k].leftTopX;
        detectInfo.location.leftTopY = objInfos[k].leftTopY;
        detectInfo.location.rightBottomX = objInfos[k].rightBotX;
        detectInfo.location.rightBottomY = objInfos[k].rightBotY;
        detectInfo.confidence = objInfos[k].confidence;
        detectInfo.classId = objInfos[k].classId;
        dataToSend->detectResult.push_back(detectInfo);
    }
}

APP_ERROR PostProcess::WriteResult(const std::vector<ObjDetectInfo> &objInfos, uint32_t channelId, uint32_t frameId)
{
    std::string resultPathName = "result";
    uint32_t objNum = objInfos.size();
    // Create result directory when it does not exist
    if (access(resultPathName.c_str(), 0) != 0) {
        int ret = mkdir(resultPathName.c_str(), S_IRUSR | S_IWUSR | S_IXUSR); // for linux
        if (ret != 0) {
            LogError << "Failed to create result directory: " << resultPathName << ", ret = " << ret;
            return ret;
        }
    }
    // Result file name use the time stamp as a suffix
    struct timeval time = {0, 0};
    gettimeofday(&time, nullptr);
    const int TIME_DIFF_S = 28800; // 8 hour time difference
    const int TIME_STRING_SIZE = 32;
    char timeString[TIME_STRING_SIZE] = {0};
    time_t timeVal = time.tv_sec + TIME_DIFF_S;
    struct tm *ptm = gmtime(&timeVal);
    if (ptm != nullptr) {
        strftime(timeString, sizeof(timeString), "%Y%m%d%H%M%S", ptm);
    }


    // Create result file under result directory
    std::stringstream formatStr;
    formatStr << resultPathName << "/result_" << channelId << "_" << frameId << "_" << timeString << ".txt";
    resultPathName = formatStr.str();
    SetFileDefaultUmask();
    std::ofstream tfile(resultPathName);

    // Check result file validity
    if (tfile.fail()) {
        LogError << "Failed to open result file: " << resultPathName;
        return APP_ERR_COMM_OPEN_FAIL;
    }
    tfile << "[Channel" << channelId << "-Frame" << frameId << "] Object detected number is " << objNum << std::endl;
    // Write inference result into file
    for (uint32_t i = 0; i < objNum; i++) {
        tfile << "#Obj" << i << ", " << "box(" << objInfos[i].leftTopX << ", " << objInfos[i].leftTopY << ", "
              << objInfos[i].rightBotX << ", " << objInfos[i].rightBotY << ") "
              << " confidence: " << objInfos[i].confidence << "  lable: " << objInfos[i].classId << std::endl;
    }
    tfile.close();
    return APP_ERR_OK;
}

APP_ERROR PostProcess::YoloPostProcess(std::vector<RawData> &modelOutput, std::shared_ptr<DeviceStreamData> &dataToSend,std::vector<ObjDetectInfo> &objInfos)
{
    const size_t outputLen = modelOutput.size();
    if (outputLen <= 0) {
        LogError << "Failed to get model output data";
        return APP_ERR_INFER_GET_OUTPUT_FAIL;
    }

    
    APP_ERROR ret;
    if (modelType_ == YOLOV3_CAFFE) {
        ret = GetObjectInfoCaffe(modelOutput, objInfos);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to get Caffe model output, ret = " << ret;
            return ret;
        }
    } else {
        ret = GetObjectInfoTensorflow(modelOutput, objInfos);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to get Caffe model output, ret = " << ret;
            return ret;
        }
    }

    ConstructData(objInfos, dataToSend);
    // Write object info to result file
    ret = WriteResult(objInfos, dataToSend->channelId, dataToSend->framId);
    if (ret != APP_ERR_OK) {
        LogError << "Failed to write result, ret = " << ret;
    }
    return APP_ERR_OK;
}

APP_ERROR PostProcess::GetObjectInfoCaffe(std::vector<RawData> &modelOutput,
    std::vector<ObjDetectInfo> &objInfos)
{
    std::vector<std::shared_ptr<void>> hostPtr;
    std::vector<void *> buffer = buffers_.front();
    buffers_.pop();
    buffers_.push(buffer);
    for (size_t j = 0; j < modelOutput.size(); j++) {
        void *hostPtrBuffer = buffer[j];
        std::shared_ptr<void> hostPtrBufferManager(hostPtrBuffer, [](void *) {});
        APP_ERROR ret = (APP_ERROR)aclrtMemcpy(hostPtrBuffer, modelOutput[j].lenOfByte, modelOutput[j].data.get(),
            modelOutput[j].lenOfByte, ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to copy output buffer of model from device to host, ret = " << ret;
            return ret;
        }
        hostPtr.push_back(hostPtrBufferManager);
    }
    uint32_t objNum = ((uint32_t *)(hostPtr[1].get()))[0];
    for (uint32_t k = 0; k < objNum; k++) {
        int pos = 0;
        ObjDetectInfo objInfo;
        objInfo.leftTopX = ((float *)hostPtr[0].get())[objNum * (pos++) + k];
        objInfo.leftTopY = ((float *)hostPtr[0].get())[objNum * (pos++) + k];
        objInfo.rightBotX = ((float *)hostPtr[0].get())[objNum * (pos++) + k];
        objInfo.rightBotY = ((float *)hostPtr[0].get())[objNum * (pos++) + k];
        objInfo.confidence = ((float *)hostPtr[0].get())[objNum * (pos++) + k];
        objInfo.classId = ((float *)hostPtr[0].get())[objNum * (pos++) + k];
        objInfos.push_back(objInfo);
    }
    return APP_ERR_OK;
}

APP_ERROR PostProcess::GetObjectInfoTensorflow(std::vector<RawData> &modelOutput,
    std::vector<ObjDetectInfo> &objInfos)
{
    std::vector<std::shared_ptr<void>> hostPtr;
    std::vector<void *> buffer = buffers_.front();
    buffers_.pop();
    buffers_.push(buffer);
    for (size_t j = 0; j < modelOutput.size(); j++) {
        void *hostPtrBuffer = buffer[j];
        std::shared_ptr<void> hostPtrBufferManager(hostPtrBuffer, [](void *) {});
        APP_ERROR ret = (APP_ERROR)aclrtMemcpy(hostPtrBuffer, modelOutput[j].lenOfByte, modelOutput[j].data.get(),
            modelOutput[j].lenOfByte, ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to copy output buffer of model from device to host, ret = " << ret;
            return ret;
        }
        hostPtr.push_back(hostPtrBufferManager);
    }
    Yolov3DetectionOutput(hostPtr, objInfos, yoloImageInfo_);
    return APP_ERR_OK;
}

APP_ERROR PostProcess::Process(std::shared_ptr<void> inputData)
{
    std::shared_ptr<CommonData> data = std::static_pointer_cast<CommonData>(inputData);
    if (data->eof) {
        Singleton::GetInstance().GetStopedStreamNum()++;
        if (Singleton::GetInstance().GetStopedStreamNum() == Singleton::GetInstance().GetStreamPullerNum()) {
            Singleton::GetInstance().SetSignalRecieved(true);
        }
        return APP_ERR_OK;
    }
    std::vector<RawData> &modelOutput = data->inferOutput;
    yoloImageInfo_ = data->yoloImgInfo;
    modelType_ = data->modelType;

    std::shared_ptr<DeviceStreamData> detectInfo = std::make_shared<DeviceStreamData>();
    detectInfo->framId = data->frameId;
    detectInfo->channelId = data->channelId;

    std::vector<ObjDetectInfo> objInfos;
    APP_ERROR ret = YoloPostProcess(modelOutput, detectInfo,objInfos);
    if (ret != APP_ERR_OK) {
        acldvppFree(data->dvppData->data);
        LogError << "Failed to run YoloPostProcess, ret = " << ret;
        return ret;
    }
    //test for streaming of data 
    uint32_t objNum = objInfos.size();
    std::cout << "Detected Obj:" << objNum <<std::endl;
    std::shared_ptr<void> vdecOutBufferDev(data->dvppData->data, acldvppFree);
    void *dataHost = malloc(data->dvppData->dataSize);
    if (dataHost == nullptr)
    {
        LogError << "malloc host data buffer failed. dataSize= " << data->dvppData->dataSize << "\n";
    }
    // copy output to host memory
    auto aclRet = aclrtMemcpy(dataHost, data->dvppData->dataSize, vdecOutBufferDev.get(), data->dvppData->dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
    if (aclRet != ACL_ERROR_NONE)
    {
        LogError << "acl memcpy data to host failed, dataSize= " << data->dvppData->dataSize << "ret= " << aclRet << "\n";
        free(dataHost);
    }
    try
    {
        //create a udp socket
        UDPSocket sock;
        string servAddress = "192.168.5.255";
        unsigned short servPort = 8888;
        std::cout << "datasize: " << data->dvppData->dataSize << std::endl;
        int total_pack = 1 + (data->dvppData->dataSize - 1) / PACK_SIZE;
        total_pack += 1;
        int ibuf[1];
        ibuf[0] = total_pack;
        sock.sendTo(ibuf, sizeof(int), servAddress, servPort);
        
        string payloadData;
        std::stringstream sendingDataStream;
        sendingDataStream << "Chnl" << detectInfo->channelId << "-Frme " 
        << detectInfo->framId << " ObjDetNum" << "["<< objNum <<"]"<< "\n";
        // Write inference result into file
        for (uint32_t i = 0; i < 2; i++) {
        sendingDataStream << "#Obj" << i << ", " << "b[" << objInfos[i].leftTopX << "]" 
                <<"["<< objInfos[i].leftTopY << "]"
              <<"["<< objInfos[i].rightBotX <<"]"<< "[" << objInfos[i].rightBotY << "] "
              << " c: [" << objInfos[i].confidence <<"]lbl:[" << objInfos[i].classId <<"]"<< "\n";
        }
        payloadData = sendingDataStream.str();

        //send the image data
         int index =0; 
         for (int i = 0; i < total_pack - 2; i++){
            sock.sendTo(static_cast<char*>(dataHost)+index, PACK_SIZE, servAddress, servPort);
            index+=PACK_SIZE;
         }
         int remainingByte = data->dvppData->dataSize-index;
         sock.sendTo(static_cast<char*>(dataHost)+index, remainingByte, servAddress, servPort);
        sock.sendTo(payloadData.c_str(), payloadData.size(), servAddress, servPort);
        std::cout << "streaming done" << std::endl;

    }
    catch (SocketException &e)
    {
        cerr << e.what() << endl;
        LogError << "THis is a streaming error ";
    }

    free(dataHost);
    acldvppFree(data->dvppData->data);
    return APP_ERR_OK;
}

APP_ERROR PostProcess::DeInit(void)
{
    while (!buffers_.empty()) {
        std::vector<void *> buffer = buffers_.front();
        buffers_.pop();
        for (auto& j : buffer) {
            aclrtFreeHost(j);
        }
    }
    return APP_ERR_OK;
}
