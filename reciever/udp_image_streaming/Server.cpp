
/*  C++ UDP socket server for live image upstreaming
 *   Modified from http://cs.ecs.baylor.edu/~donahoo/practical/CSockets/practical/UDPEchoServer.cpp
 *   Copyright (C) 2015
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "PracticalSocket.h" // For UDPSocket and SocketException
#include <iostream>          // For cout and cerr
#include <cstdlib>           // For atoi()
#include <sstream>
#include <atomic>
#include <cmath>
#include <sys/stat.h>
#include <sys/time.h>
#include <bits/stdc++.h>

string labels[100] = {
    "plate",
"bicycle",
"car",
"motorbike",
"aeroplane",
"bus",
"train",
"truck",
"boat",
"traffic light",
"fire hydrant",
"stop sign",
"parking meter",
"bench",
"bird",
"cat",
"dog",
"horse",
"sheep",
"cow",
"elephant",
"bear",
"zebra",
"giraffe",
"backpack",
"umbrella",
"handbag",
"tie",
"suitcase",
"frisbee",
"skis",
"snowboard",
"sports ball",
"kite",
"baseball bat",
"baseball glove",
"skateboard",
"surfboard",
"tennis racket",
"bottle",
"wine glass",
"cup",
"fork",
"knife",
"spoon",
"bowl",
"banana",
"apple",
"sandwich",
"orange",
"broccoli",
"carrot",
"hot dog",
"pizza",
"donut",
"cake",
"chair",
"sofa",
"pottedplant",
"bed",
"diningtable",
"toilet",
"tvmonitor",
"laptop",
"mouse",
"remote",
"keyboard",
"cell phone",
"microwave",
"oven",
"toaster",
"sink",
"refrigerator",
"book",
"clock",
"vase",
"scissors",
"teddy bear",
"hair drier",
"toothbrush"
};

#define BUF_LEN 65540 // Larger than maximum UDP packet size

#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
#include "config.h"

const int ECHOMAX = 255;
RNG rng(12345);

void writeToFile(char *longbuf, int size)
{

    std::string resultPathName = "DestinationTest";
    struct timeval time = {0, 0};
    gettimeofday(&time, nullptr);
    const int TIME_DIFF_S = 28800; // 8 hour time difference
    const int TIME_STRING_SIZE = 32;
    char timeString[TIME_STRING_SIZE] = {0};
    time_t timeVal = time.tv_sec + TIME_DIFF_S;
    struct tm *ptm = gmtime(&timeVal);
    if (ptm != nullptr)
    {
        strftime(timeString, sizeof(timeString), "%Y%m%d%H%M%S", ptm);
    }
    // Create result file under result directory
    std::stringstream formatStr;
    formatStr << resultPathName << "resultNew_"
              << "_" << timeString;
    resultPathName = formatStr.str();
    std::cout << resultPathName << std::endl;
    FILE *outFileFp = fopen(resultPathName.c_str(), "wb+");
    if (outFileFp == nullptr)
    {
        cout << "the filepointer is empty" << endl;
        exit(1);
    }

    size_t writeRet = fwrite(longbuf, sizeof(char), size, outFileFp);
    fflush(outFileFp);
    fclose(outFileFp);
}

// Function to print strings present
// between any pair of delimeters
void printSubsInDelimeters(string str, vector<float> &metadata)
{
    // Stores the indices of
    stack<int> dels;
    for (int i = 0; i < str.size(); i++)
    {
        // If opening delimeter
        // is encountered
        if (str[i] == '[')
        {
            dels.push(i);
        }

        // If closing delimeter
        // is encountered
        else if (str[i] == ']' && !dels.empty())
        {

            // Extract the position
            // of opening delimeter
            int pos = dels.top();

            dels.pop();

            // Length of substring
            int len = i - 1 - pos;

            // Extract the substring
            string ans = str.substr(
                pos + 1, len);

            metadata.push_back(std::stod(ans));
        }
    }
}
int main(int argc, char *argv[])
{

    int height = 416;
    int width = 416;
    if (argc != 2)
    { // Test for correct number of parameters
        cerr << "Usage: " << argv[0] << " <Server Port>" << endl;
        exit(1);
    }

    unsigned short servPort = atoi(argv[1]); // First arg:  local port
    namedWindow("recv", WINDOW_AUTOSIZE);
    try
    {
        UDPSocket sock(servPort);

        char echoBuffer[ECHOMAX];
        char buffer[BUF_LEN];      // Buffer for echo string
        int recvMsgSize;           // Size of received message
        string sourceAddress;      // Address of datagram source
        unsigned short sourcePort; // Port of datagram source

        clock_t last_cycle = clock();

        while (1)
        {

            // Block until receive message from a client
            do
            {
                recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
            } while (recvMsgSize > sizeof(int));
            int total_pack = ((int *)buffer)[0];

            //cout << "expecting length of packs:" << total_pack << endl;
            int imageSize = 0;
            char *longbuf = new char[PACK_SIZE * total_pack - 1]; //decreatsing 1 to accomodate the string data
            for (int i = 0; i < total_pack - 1; i++)
            {
                recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
                if (recvMsgSize != PACK_SIZE)
                {
                    cerr << "Received unexpected size pack:" << recvMsgSize << endl;
                    continue;
                }
                imageSize += recvMsgSize;
                memcpy(&longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
            }

            //cout << "Image Size" << imageSize << endl;

            // //recive the string data as well
            recvMsgSize = sock.recvFrom(echoBuffer, PACK_SIZE, sourceAddress, sourcePort);

            //writeToFile(longbuf, PACK_SIZE * total_pack - 1);

            echoBuffer[recvMsgSize] = '\0';
            //cout << "The message recieved is " << echoBuffer << endl;

            cout << "Received packet from " << sourceAddress << ":" << sourcePort << endl;
            cv::Mat mat_src = cv::Mat(height * 1.5, width, CV_8UC1, longbuf);
            cv::Mat mat_dst = cv::Mat(height, width, CV_8UC3);
            cv::cvtColor(mat_src, mat_dst, cv::COLOR_YUV2BGR_NV21);
            cv::cvtColor(mat_dst, mat_dst, cv::COLOR_BGR2RGB);

            vector<float> metadata;
            printSubsInDelimeters(echoBuffer, metadata);

            int totaObjectDetected = metadata[0];
            int dataSize = metadata.size();
            int featureNum = 6; //4 bounding box and one confidence one label;
            int recievedFeature = (dataSize - 1) / featureNum;

            int orig_height = 1080;
            int orig_width = 1920;
            int frame_height = 416;
            int frame_width = 416;

            float height_ratio = 1.0 * frame_height / orig_height;
            float width_ratio = 1.0 * frame_width / orig_width;

            for (int i = 0; i < recievedFeature; i++)
            {
                Scalar color = Scalar(255, 0, 0);
                int x0 = floor(metadata[featureNum * i + 1] * width_ratio);
                int y0 = floor(metadata[featureNum * i + 2] * height_ratio);
                int x1 = floor(metadata[featureNum * i + 3] * width_ratio);
                int y1 = floor(metadata[featureNum * i + 4] * height_ratio);

                float confidence = metadata[featureNum * i + 5];
                int labelIndex = metadata[featureNum*i+6];
                
                if (confidence > 0.4)
                {
                    rectangle(mat_dst, Point(x0, y0), Point(x1, y1), color, 3);
                    putText(mat_dst, labels[labelIndex], Point(x0, y0), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
                }
            }

            imshow("recv", mat_dst);
            free(longbuf);

            waitKey(1);
            clock_t next_cycle = clock();
            double duration = (next_cycle - last_cycle) / (double)CLOCKS_PER_SEC;
            cout << "\teffective FPS:" << (1 / duration) << " \tkbps:" << (PACK_SIZE * total_pack / duration / 1024 * 8) << endl;

            cout << next_cycle - last_cycle;
            last_cycle = next_cycle;
        }
    }
    catch (SocketException &e)
    {
        cerr << e.what() << endl;
        exit(1);
    }

    return 0;
}
