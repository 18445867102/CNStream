#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#ifdef HAVE_OPENCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#if (CV_MAJOR_VERSION >= 3)
#include "opencv2/imgcodecs/imgcodecs.hpp"
#endif
#endif

#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#define SOCK_BUFSIZE (20 * 1024)  // buffer size
#define MAX_PLANES 6

static std::string g_socket_address = "/tmp/test_ipc";

enum CNPkgType {
  CNPKG_INVALID = -1,     ///< invalid package type
  CNPKG_DATA = 0,         ///< data package
  CNPKG_RELEASE_MEM = 1,  ///< package with release shared memory info
  CNPKG_EXIT = 2,         ///< package with exit info
  CNPKG_ERROR = 3         ///< package with error info
};

typedef enum {
  PIXEL_FORMAT_YUV420_NV21 = 0,  ///< This frame is in the YUV420SP(NV21) format.
} DataFormat;

typedef struct {
  float x, y, w, h;
} InferBoundingBox;

typedef struct {
  InferBoundingBox bbox;  ///< position info for object
  float score;            ///< score
  std::string label_id;   ///< label id
} InferObjInfo;

typedef struct {
  CNPkgType pkg_type;                     ///< package type
  uint32_t channel_idx = 0;               ///< The index of the channel, stream_index
  std::string stream_id;                  ///< The data stream aliases where this frame is located to.
  size_t flags = 0;                       ///< The mask for this frame, ``CNFrameFlag``.
  uint64_t frame_id;                      ///< The frame index that incremented from 0.
  uint64_t timestamp;                     ///< The time stamp of this frame.
  DataFormat fmt;                         ///< The format of the frame.
  int width;                              ///< The width of the frame.
  int height;                             ///< The height of the frame.
  int stride[MAX_PLANES];                 ///< The strides of the frame.
  std::vector<InferObjInfo> detect_objs;  ///< detection objects.
} InferFramePackage;

bool parseJsonStrToDataPackage(const std::string& str, InferFramePackage* pkg) {
  if (!pkg) return false;

  rapidjson::Document doc;
  if (doc.Parse<rapidjson::kParseCommentsFlag>(str.c_str()).HasParseError()) {
    std::cout << "SerializeFromString failed. Error code [" << std::to_string(doc.GetParseError()) << "]"
              << " Offset [" << std::to_string(doc.GetErrorOffset()) << "]. JSON:" << str << std::endl;
    return false;
  }

  // get members
  const auto end = doc.MemberEnd();

  // pkg_type
  if (end == doc.FindMember("pkg_type") || !doc["pkg_type"].IsInt()) {
    return false;
  } else {
    pkg->pkg_type = CNPkgType(doc["pkg_type"].GetInt());
  }

  if (CNPKG_RELEASE_MEM == pkg->pkg_type || CNPKG_DATA == pkg->pkg_type) {
    if (end == doc.FindMember("stream_id") || !doc["stream_id"].IsString()) {
      std::cout << "parse stream_id error.\n";
      return false;
    } else {
      pkg->stream_id = doc["stream_id"].GetString();
    }

    if (end == doc.FindMember("channel_idx") || !doc["channel_idx"].IsUint()) {
      std::cout << "parse channel_idx error.\n";
      return false;
    } else {
      pkg->channel_idx = doc["channel_idx"].GetUint();
    }

    if (end == doc.FindMember("frame_id") || !doc["frame_id"].IsInt64()) {
      std::cout << "parse frame_id error.\n";
      return false;
    } else {
      pkg->frame_id = doc["frame_id"].GetInt64();
    }
  }

  if (CNPKG_DATA == pkg->pkg_type) {
    if (end == doc.FindMember("flags") || !doc["flags"].IsUint()) {
      std::cout << "parse flags error.\n";
      return false;
    } else {
      pkg->flags = doc["flags"].GetUint();
    }

    if (end == doc.FindMember("timestamp") || !doc["timestamp"].IsInt64()) {
      std::cout << "parse timestamp error.\n";
      return false;
    } else {
      pkg->timestamp = doc["timestamp"].GetInt64();
    }

    if (end == doc.FindMember("data_fmt") || !doc["data_fmt"].IsInt()) {
      std::cout << "parse data fmt error.\n";
      return false;
    } else {
      pkg->fmt = DataFormat(doc["data_fmt"].GetInt());
    }

    if (end == doc.FindMember("width") || !doc["width"].IsInt()) {
      std::cout << "parse width error.\n";
      return false;
    } else {
      pkg->width = doc["width"].GetInt();
    }

    if (end == doc.FindMember("height") || !doc["height"].IsInt()) {
      std::cout << "parse height error.\n";
      return false;
    } else {
      pkg->height = doc["height"].GetInt();
    }

    if (end == doc.FindMember("strides") || !doc["strides"].IsArray()) {
      std::cout << "parse strides error.\n";
      return false;
    } else {
      auto values = doc["strides"].GetArray();
      int i = 0;
      for (auto iter = values.begin(); iter != values.end(); ++iter) {
        if (!iter->IsInt()) {
          std::cout << "parse strides type error.\n";
          return false;
        }
        pkg->stride[i] = iter->GetInt();
        i++;
      }
    }

    if (end != doc.FindMember("detect_objs") && doc["detect_objs"].IsArray()) {
      const rapidjson::Value& objs = doc["detect_objs"];
      for (size_t i = 0; i < objs.Size(); ++i) {
        const rapidjson::Value& obj = objs[i];
        InferObjInfo obj_info;
        obj_info.bbox.x = obj["x"].GetDouble();
        obj_info.bbox.y = obj["y"].GetDouble();
        obj_info.bbox.w = obj["w"].GetDouble();
        obj_info.bbox.h = obj["h"].GetDouble();
        obj_info.score = obj["score"].GetDouble();
        obj_info.label_id = obj["label_id"].GetString();
        pkg->detect_objs.push_back(obj_info);
      }
    }
  }
  return true;
}

bool prepareSendPackage(const CNPkgType& pkg_type, const int& channel_idx, const std::string& stream_id,
                        const uint64_t& frame_id, std::string* str) {
  if (!str) return false;
  rapidjson::StringBuffer strBuf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
  writer.StartObject();

  writer.Key("pkg_type");
  writer.Int(static_cast<int>(pkg_type));

  if (CNPKG_RELEASE_MEM == pkg_type) {
    writer.Key("channel_idx");
    writer.Uint(channel_idx);

    writer.Key("stream_id");
    writer.String(stream_id.c_str());

    writer.Key("frame_id");
    writer.Int64(frame_id);
  }

  writer.EndObject();
  *str = strBuf.GetString();
  return true;
}

void readImgFromSharedMem(const InferFramePackage& frame_pkg, char* img_buffer) {
  if (nullptr == img_buffer) return;
  // open shared memoey
  size_t nbytes = frame_pkg.width * frame_pkg.height * 3 / 2;
  size_t boundary = 1 << 16;
  size_t map_mem_size = (nbytes + boundary - 1) & ~(boundary - 1);
  const std::string key = "stream_id_" + frame_pkg.stream_id + "_frame_id_" + std::to_string(frame_pkg.frame_id);
  int map_mem_fd = shm_open(key.c_str(), O_RDWR, S_IRUSR | S_IWUSR);
  if (map_mem_fd < 0) {
    std::cout << "Shered memory open failed, fd: " << map_mem_fd << ", error code: " << errno << std::endl;
  }

  void* map_mem_ptr = mmap(NULL, map_mem_size, PROT_READ | PROT_WRITE, MAP_SHARED, map_mem_fd, 0);
  if (map_mem_ptr == MAP_FAILED) {
    std::cout << "Mmap error" << std::endl;
  }

  if (ftruncate(map_mem_fd, map_mem_size) == -1) {
    std::cout << "truncate shared memory size failed" << std::endl;
  }

  // copy image  data out and convert to bgr
  char* src_frame_ptr = reinterpret_cast<char*>(map_mem_ptr);
  memcpy(img_buffer, src_frame_ptr, frame_pkg.height * frame_pkg.width);
  memcpy(img_buffer + frame_pkg.height * frame_pkg.width, src_frame_ptr + frame_pkg.height * frame_pkg.stride[0],
         (frame_pkg.height * frame_pkg.stride[1]) / 2);
  cv::Mat bgr(frame_pkg.height, frame_pkg.stride[0], CV_8UC3);
  cv::Mat src = cv::Mat(frame_pkg.height * 3 / 2, frame_pkg.stride[0], CV_8UC1, img_buffer);
  cv::cvtColor(src, bgr, cv::COLOR_YUV2BGR_NV21);
  cv::imwrite("stream_" + std::to_string(frame_pkg.frame_id) + "_.jpg", bgr);
}

int main(int argc, char** argv) {
  std::string socket_address = g_socket_address;
  int listen_fd_ = -1;
  int socket_fd_ = -1;
  char recv_buf[SOCK_BUFSIZE];
  char send_buf[SOCK_BUFSIZE];
  void* frame_buffer = nullptr;

  /*********  create socket  **********/
  unlink(socket_address.c_str());
  listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (-1 == listen_fd_) {
    std::cout << "create listen_fd for server failed, errno: " << errno << std::endl;
    return false;
  }

  sockaddr_un un;
  memset(&un, 0, sizeof(un));
  un.sun_family = AF_UNIX;
  memcpy(un.sun_path, socket_address.c_str(), socket_address.length());
  unsigned int length = strlen(un.sun_path) + sizeof(un.sun_family);

  /*********  bind socket  **********/
  if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&un), length) < 0) {
    std::cout << "bind server listen_fd failed, errno: " << errno << std::endl;
    return false;
  }

  /*********  listen  **********/
  if (listen(listen_fd_, 1) < 0) {
    std::cout << "start server listen failed, errno: " << errno << std::endl;
    return false;
  } else {
    std::cout << "ipcserver--- server listening connection " << std::endl;
  }

  /*********  listen connection loop **********/
  while (1) {
    unsigned int length = 0;
    // accept connection
    socket_fd_ = accept(listen_fd_, reinterpret_cast<sockaddr*>(&un), &length);
    if (-1 == socket_fd_) {
      std::cout << "server accept failed, errno: " << errno << std::endl;
      ;
      return false;
    }

    // recv data package loop
    while (recv(socket_fd_, recv_buf, sizeof(recv_buf), 0)) {
      // parse data package
      InferFramePackage recv_pkg;
      std::string recv_str(recv_buf);
      parseJsonStrToDataPackage(recv_str, &recv_pkg);

      std::cout << "testcns--- recv string: " << recv_str << std::endl;
      // get image data(nv21 format) from shared memory and dump
      if (nullptr == frame_buffer) {
        frame_buffer = malloc(recv_pkg.width * recv_pkg.height * 3 / 2);
      }

      if (!recv_pkg.flags) {  // normal data
        readImgFromSharedMem(recv_pkg, reinterpret_cast<char*>(frame_buffer));
      }

      // prepare and send release_mem package to cnstream client
      std::string send_str;
      if (!recv_pkg.flags) {  // normal data
        prepareSendPackage(CNPKG_RELEASE_MEM, recv_pkg.channel_idx, recv_pkg.stream_id, recv_pkg.frame_id, &send_str);
      } else {  // eos
        prepareSendPackage(CNPKG_EXIT, recv_pkg.channel_idx, "0", 0, &send_str);
      }

      std::cout << "ipcserver--- send string: " << send_str << std::endl;
      memset(send_buf, 0, sizeof(send_buf));
      memcpy(send_buf, send_str.c_str(), send_str.length());
      if (sizeof(send_buf) != send(socket_fd_, send_buf, sizeof(send_buf), 0)) {
        std::cout << "send data to client failed.\n";
        break;
      }
    }

    break;
  }

  std::cout << "ipcserver end------------\n";
  close(socket_fd_);
  socket_fd_ = -1;
  close(listen_fd_);
  listen_fd_ = -1;
  if (frame_buffer) free(frame_buffer);

  return 1;
}
