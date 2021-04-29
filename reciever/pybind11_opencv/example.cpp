#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "opencv2/opencv.hpp"
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <bits/stdc++.h>
#include <iostream>
#include <string>

#include "PracticalSocket.h" // For UDPSocket and SocketException
#include "config.h"
#include "ndarray_converter.h"

#define BUF_LEN 65540 // Larger than maximum UDP packet size

using namespace cv;
using namespace std;

namespace py = pybind11;

const int ECHOMAX = 255;

string labels[100] = {
    "Plate",
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
    "toothbrush"};

void printSubsInDelimeters(string str, vector<float> &metadata) {
  // Stores the indices of
  stack<int> dels;
  for (int i = 0; i < str.size(); i++) {
    // If opening delimeter
    // is encountered
    if (str[i] == '[') {
      dels.push(i);
    }

    // If closing delimeter
    // is encountered
    else if (str[i] == ']' && !dels.empty()) {

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

std::tuple<cv::Mat, std::string> getUdpFrame(unsigned short servPort) {
	
  int height = 416;
  int width = 416;
  try {
    std::cout <<"checkpoint 1"<<std::endl;
    std::cout <<"I am here "<<std::endl;
    UDPSocket sock(servPort);
    std::cout <<"checkpoint 2 "<<std::endl;
    char echoBuffer[ECHOMAX];
    char buffer[BUF_LEN];      // Buffer for echo string
    int recvMsgSize;           // Size of received message
    string sourceAddress = "192.168.5.255";      // Address of datagram source
    unsigned short sourcePort; // Port of datagram source
    std::cout <<"checkpoint 2.1 "<<std::endl;     
    do {
      recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
	std::cout << "recv" << recvMsgSize <<std::endl;
    } while (recvMsgSize > sizeof(int));
    std::cout <<"checkpoint 3 "<<std::endl;
    int total_pack = ((int *)buffer)[0];

    //cout << "expecting length of packs:" << total_pack << endl;
    int imageSize = 0;
    char *longbuf = new char[PACK_SIZE * total_pack - 1]; //decreatsing 1 to accomodate the string data
    for (int i = 0; i < total_pack - 1; i++) {
      recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
      if (recvMsgSize != PACK_SIZE) {
        cerr << "Received unexpected size pack:" << recvMsgSize << endl;
        //continue;
      }
      imageSize += recvMsgSize;
      memcpy(&longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
    }
    // //recive the string data as well
    recvMsgSize = sock.recvFrom(echoBuffer, PACK_SIZE, sourceAddress, sourcePort);
    //writeToFile(longbuf, PACK_SIZE * total_pack - 1);

    echoBuffer[recvMsgSize] = '\0';
    //cout << "The message recieved is " << echoBuffer << endl;
    //cout << "Received packet from " << sourceAddress << ":" << sourcePort << endl;
    cv::Mat mat_src;

    mat_src = cv::Mat(height * 1.5, width, CV_8UC1, longbuf);

    cv::Mat mat_dst = cv::Mat(height, width, CV_8UC3);
    cv::cvtColor(mat_src, mat_dst, cv::COLOR_YUV2BGR_NV21);
    cv::cvtColor(mat_dst, mat_dst, cv::COLOR_BGR2RGB);
    
  
    vector<float> metadata;
    boost::property_tree::ptree pt;
    boost::property_tree::ptree children; //for creating json
    printSubsInDelimeters(echoBuffer, metadata);
    
    if(metadata.size() <1 ){
      
      cv::Mat error_frame = cv:: Mat::zeros(Size(height,width),CV_8UC3);
      return std::make_tuple(error_frame, "Error frame");
    }
    int totaObjectDetected = metadata[0];
    pt.put("expected_objects", totaObjectDetected);
    int dataSize = metadata.size();
    int featureNum = 6; //4 bounding box and one confidence one label;
    int recievedFeature = (dataSize - 1) / featureNum;

    int orig_height = 1080;
    int orig_width = 1920;
    int frame_height = 416;
    int frame_width = 416;

    float height_ratio = 1.0 * frame_height / orig_height;
    float width_ratio = 1.0 * frame_width / orig_width;

    for (int i = 0; i < recievedFeature; i++) {
      boost::property_tree::ptree child;
      Scalar color = Scalar(255, 0, 0);

      int x0 = floor(metadata[featureNum * i + 1] * width_ratio);
      child.put("bounding_x0", x0);
      int y0 = floor(metadata[featureNum * i + 2] * height_ratio);
      child.put("bounding_y0", y0);
      int x1 = floor(metadata[featureNum * i + 3] * width_ratio);
      child.put("bounding_x1", x1);
      int y1 = floor(metadata[featureNum * i + 4] * height_ratio);
      child.put("bounding_y1", y1);
      float confidence = metadata[featureNum * i + 5];
      child.put("confidence", confidence);
      int labelIndex = metadata[featureNum * i + 6];
      child.put("object_name", labels[labelIndex]);

      if (confidence > 0.4) {
        rectangle(mat_dst, Point(x0, y0), Point(x1, y1), color, 3);
        putText(mat_dst, labels[labelIndex], Point(x0, y0), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
      }
      children.push_back(std::make_pair("", child));
    }
    pt.add_child("detected_objects", children);
    std::stringstream ss;
    boost::property_tree::json_parser::write_json(ss, pt);

    free(longbuf);
    return std::make_tuple(mat_dst, ss.str());
    

  } catch (SocketException &e) {
    std::cout << "Exception occured" <<e.what() << std::endl;
    exit(1);
  }
}

void show_image(cv::Mat image) {
  cv::imshow("image_from_Cpp", image);
  cv::waitKey(1);
}

cv::Mat read_image(std::string image_name) {
#if CV_MAJOR_VERSION < 4
  cv::Mat image = cv::imread(image_name, CV_LOAD_IMAGE_COLOR);
#else
  cv::Mat image = cv::imread(image_name, cv::IMREAD_COLOR);
#endif
  return image;
}

cv::Mat passthru(cv::Mat image) {
  return image;
}

cv::Mat cloneimg(cv::Mat image) {
  return image.clone();
}

class AddClass {
public:
  AddClass(int value)
      : value(value) {
  }

  cv::Mat add(cv::Mat input) {
    return input + this->value;
  }

private:
  int value;
};

PYBIND11_MODULE(example, m) {

  NDArrayConverter::init_numpy();

  m.def("read_image", &read_image, "A function that read an image",
        py::arg("image"));

  m.def("show_image", &show_image, "A function that show an image",
        py::arg("image"));

  m.def("passthru", &passthru, "Passthru function", py::arg("image"));
  m.def("clone", &cloneimg, "Clone function", py::arg("image"));

  m.def("getUdpFrame", &getUdpFrame, "A function that returns a udp image frame",
        py::arg("servPort"));

  py::class_<AddClass>(m, "AddClass")
      .def(py::init<int>())
      .def("add", &AddClass::add);
}
