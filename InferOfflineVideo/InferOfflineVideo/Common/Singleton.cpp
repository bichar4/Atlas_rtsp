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

#include "Singleton.h"

static std::mutex g_mtx = {};

Singleton& Singleton::GetInstance()
{
    std::unique_lock<std::mutex> lock(g_mtx);
    static Singleton singleton;
    return singleton;
}

bool Singleton::GetSignalRecieved() const
{
    return signalRecieved;
}

void Singleton::SetSignalRecieved(bool val)
{
    signalRecieved = val;
}

int Singleton::GetStreamPullerNum() const
{
    return streamPullerNum;
}

void Singleton::SetStreamPullerNum(int num)
{
    streamPullerNum = num;
}

std::atomic_int& Singleton::GetStopedStreamNum()
{
    return stopedStreamNum;
}