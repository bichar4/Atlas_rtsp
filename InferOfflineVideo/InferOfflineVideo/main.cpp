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

#include <cstring>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <atomic>
#include "CommandLine.h"
#include "Singleton.h"
#include "ConfigParser/ConfigParser.h"
#include "Log/Log.h"
#include "ModuleManager/ModuleManager.h"

#include "StreamPuller/StreamPuller.h"
#include "VideoDecoder/VideoDecoder.h"
#include "ModelInfer/ModelInfer.h"
#include "PostProcess/PostProcess.h"

using namespace ascendBaseModule;

namespace {
    const uint8_t MODULE_TYPE_COUNT = 4;
    const int MODULE_CONNECT_COUNT = 3;
}

ModuleDesc g_moduleDesc[MODULE_TYPE_COUNT] = {
    {MT_StreamPuller, -1},
    {MT_VideoDecoder, -1},
    {MT_ModelInfer, -1},
    {MT_PostProcess, -1},
};

ModuleConnectDesc g_connectDesc[MODULE_CONNECT_COUNT] = {
    {MT_StreamPuller, MT_VideoDecoder, MODULE_CONNECT_CHANNEL},
    {MT_VideoDecoder, MT_ModelInfer, MODULE_CONNECT_CHANNEL},
    {MT_ModelInfer, MT_PostProcess, MODULE_CONNECT_CHANNEL},
};

void SigHandler(int signo)
{
    if (signo == SIGINT) {
        Singleton::GetInstance().SetSignalRecieved(true);
    }
}

APP_ERROR InitModuleManager(ModuleManager &moduleManager, std::string &configPath, std::string &aclConfigPath)
{
    LogInfo << "InitModuleManager begin";
    ConfigParser configParser;
    // 1. create queues of receivers and senders
    APP_ERROR ret = configParser.ParseConfig(configPath);
    if (ret != APP_ERR_OK) {
        LogError << "Cannot parse file, ret = " << ret;
        return ret;
    }
    std::string itemCfgStr = std::string("SystemConfig.channelCount");
    int channelCount = 0;
    ret = configParser.GetIntValue(itemCfgStr, channelCount);
    if (ret != APP_ERR_OK) {
        return ret;
    }
    if (channelCount <= 0) {
        LogError << "Invalid channel count, ret = " << ret;
        return APP_ERR_COMM_INVALID_PARAM;
    }
    LogInfo << "ModuleManager: begin to init";
    ret = moduleManager.Init(configPath, aclConfigPath);
    if (ret != APP_ERR_OK) {
        LogError << "Fail to init system manager, ret = " << ret;
        return APP_ERR_COMM_FAILURE;
    }

    Singleton::GetInstance().SetStreamPullerNum((g_moduleDesc[0].moduleCount == -1) ?
        channelCount : g_moduleDesc[0].moduleCount);
    ret = moduleManager.RegisterModules(PIPELINE_DEFAULT, g_moduleDesc, MODULE_TYPE_COUNT, channelCount);
    if (ret != APP_ERR_OK) {
        return APP_ERR_COMM_FAILURE;
    }

    ret = moduleManager.RegisterModuleConnects(PIPELINE_DEFAULT, g_connectDesc, MODULE_CONNECT_COUNT);
    if (ret != APP_ERR_OK) {
        LogError << "Fail to connect module, ret = " << ret;
        return APP_ERR_COMM_FAILURE;
    }

    return APP_ERR_OK;
}

APP_ERROR DeInitModuleManager(ModuleManager &moduleManager)
{
    APP_ERROR ret = moduleManager.DeInit();
    if (ret != APP_ERR_OK) {
        LogError << "Fail to deinit system manager, ret = " << ret;
        return APP_ERR_COMM_FAILURE;
    }

    return APP_ERR_OK;
}

inline void MainAssert(int exp)
{
    if (exp != APP_ERR_OK) {
        exit(exp);
    }
}

int main(int argc, const char *argv[])
{
    CmdParams cmdParams;
    ParseACommandLine(argc, argv, cmdParams);
    SetLogLevel(cmdParams.debugLevel);

    ModuleManager moduleManager;
    MainAssert(InitModuleManager(moduleManager, cmdParams.Config, cmdParams.aclConfig));
    MainAssert(moduleManager.RunPipeline());

    LogInfo << "wait for exit signal";
    if (signal(SIGINT, SigHandler) == SIG_ERR) {
        LogInfo << "cannot catch SIGINT.";
    }
    const uint16_t signalCheckInterval = 1000;
    while (!Singleton::GetInstance().GetSignalRecieved()) {
        usleep(signalCheckInterval);
    }

    MainAssert(DeInitModuleManager(moduleManager));

    LogInfo << "program End.";
    return 0;
}