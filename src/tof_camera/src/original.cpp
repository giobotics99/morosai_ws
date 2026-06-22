#include <chronoptics/tof.hpp>
#include <iostream>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

using namespace chronoptics;

static std::vector<sensor_msgs::msg::PointField> create_point_fields(
    tof::FrameType frame_type) {
  size_t nr_fields = [frame_type]() {
    switch (frame_type) {
      case tof::FrameType::XYZ:
        return 3;
      case tof::FrameType::XYZ_BGR:
        return 4;
      case tof::FrameType::XYZ_BGR_I:
        return 5;
      default:
        throw std::runtime_error("Type not supported for point cloud");
    }
  }();

  std::vector<sensor_msgs::msg::PointField> fields(nr_fields);
  fields[0].name = "x";
  fields[0].offset = 0;
  fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[0].count = 1;

  fields[1].name = "y";
  fields[1].offset = 4;
  fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[1].count = 1;

  fields[2].name = "z";
  fields[2].offset = 8;
  fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  fields[2].count = 1;

  if (frame_type != tof::FrameType::XYZ) {
    fields[3].name = "rgb";
    fields[3].offset = 12;
    fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
    fields[3].count = 1;

    if (frame_type == tof::FrameType::XYZ_BGR_I) {
      fields[4].name = "intensity";
      fields[4].offset = 15;
      fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
      fields[4].count = 1;
    }
  }

  return fields;
}

template <typename T>
struct PubStreaming {
  std::shared_ptr<rclcpp::Publisher<T>> pub;
  bool streaming;
};

struct FramePublishers {
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info;
  PubStreaming<sensor_msgs::msg::Image> radial;
  PubStreaming<sensor_msgs::msg::Image> intensity;
  PubStreaming<sensor_msgs::msg::Image> reflectivity;
  PubStreaming<sensor_msgs::msg::Image> z;

  PubStreaming<sensor_msgs::msg::PointCloud2> xyz;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_rgb;
  PubStreaming<sensor_msgs::msg::Image> bgr;
  PubStreaming<sensor_msgs::msg::Image> bgr_projected;

  PubStreaming<sensor_msgs::msg::PointCloud2> xyzbgr;
  PubStreaming<sensor_msgs::msg::PointCloud2> xyzbgri;
};

template <typename T>
static bool stream_changed(PubStreaming<T>& publisher,
                           rclcpp::Logger const& logger) {
  bool now_streaming = publisher.pub->get_subscription_count() > 0;

  if (now_streaming != publisher.streaming) {
    RCLCPP_INFO(logger, "%s %sstreaming", publisher.pub->get_topic_name(),
                (now_streaming ? "is now " : "stopped "));
    publisher.streaming = now_streaming;
    return true;
  }

  return false;
}

static void update_streams(tof::Camera& cam, FramePublishers& publishers,
                           std::vector<tof::FrameType>& frame_types,
                           rclcpp::Logger const& logger) {
  bool update_stream_list{false};

  auto update_stream = [&](auto& publisher, tof::FrameType frame_type) {
    if (!stream_changed(publisher, logger)) return;

    update_stream_list = true;

    if (publisher.streaming) {
      // Add to frame_types
      frame_types.push_back(frame_type);
    } else {
      // Remove from frame_types
      auto it = std::find(frame_types.begin(), frame_types.end(), frame_type);
      if (it == frame_types.end()) {
        // We should never get here
        assert(false);
        return;
      }

      frame_types.erase(it);
    }
  };

  update_stream(publishers.radial, tof::FrameType::RADIAL);
  update_stream(publishers.intensity, tof::FrameType::INTENSITY);
  update_stream(publishers.reflectivity, tof::FrameType::REFLECTIVITY);
  update_stream(publishers.z, tof::FrameType::Z);

  update_stream(publishers.xyz, tof::FrameType::XYZ);

  if (publishers.bgr.pub) {
    update_stream(publishers.bgr, tof::FrameType::BGR);
  }

  if (publishers.bgr_projected.pub) {
    update_stream(publishers.bgr_projected, tof::FrameType::BGR_PROJECTED);
  }

  if (publishers.xyzbgr.pub) {
    update_stream(publishers.xyzbgr, tof::FrameType::XYZ_BGR);
  }

  if (publishers.xyzbgri.pub) {
    update_stream(publishers.xyzbgri, tof::FrameType::XYZ_BGR_I);
  }

  if (!update_stream_list) return;

  // Start/stop camera when required
  if (frame_types.empty()) {
    RCLCPP_INFO(logger, "Stopping camera");
    cam.stop();
    return;
  }

  cam.set_output_frame_types(frame_types);

  if (!cam.is_streaming()) {
    RCLCPP_INFO(logger, "Starting camera");
    cam.start();
  }
}

static bool has_camera_rgb(tof::Camera& cam) {
  const auto streams = cam.get_stream_list();

  auto it = std::find_if(streams.begin(), streams.end(),
                         [](const tof::Stream& stream) {
                           return stream.frame_type() == tof::FrameType::BGR;
                         });

  return it != streams.end();
}

static FramePublishers create_publishers(rclcpp::Node::SharedPtr& node,
                                         const bool has_rgb) {
  FramePublishers publishers{
      node->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info",
                                                           1),
      {node->create_publisher<sensor_msgs::msg::Image>("depth/image", 1),
       false},
      {node->create_publisher<sensor_msgs::msg::Image>("depth/intensity", 1),
       false},
      {node->create_publisher<sensor_msgs::msg::Image>("depth/reflectivity", 1),
       false},
      {node->create_publisher<sensor_msgs::msg::Image>("depth/image_rect", 1),
       false},
      {node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud/xyz",
                                                             1),
       false}};

  if (has_rgb) {
    publishers.camera_info_rgb =
        node->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info",
                                                             1);
    publishers.bgr = {
        node->create_publisher<sensor_msgs::msg::Image>("rgb/image_color", 1),
        false};
    publishers.bgr_projected = {node->create_publisher<sensor_msgs::msg::Image>(
                                    "depth/image_color_rect", 1),
                                false};

    publishers.xyzbgr = {node->create_publisher<sensor_msgs::msg::PointCloud2>(
                             "point_cloud/xyzrgb", 1),
                         false};
    publishers.xyzbgri = {node->create_publisher<sensor_msgs::msg::PointCloud2>(
                              "point_cloud/xyzrgbi", 1),
                          false};
  }

  return publishers;
}

struct CallbackImage {
  CallbackImage(tof::Camera& cam_, tof::FrameType frame_type_, uint32_t width,
                uint32_t height, const std::string& encoding)
      : cam(cam_), frame_type(frame_type_) {
    uint32_t byte_depth = sensor_msgs::image_encodings::bitDepth(encoding) / 8;
    uint32_t nr_columns = sensor_msgs::image_encodings::numChannels(encoding);

    image.width = width;
    image.height = height;
    image.step = width * byte_depth * nr_columns;
    image.is_bigendian = false;
    image.encoding = encoding;

    image.data.resize(height * image.step);

    add_user_pointer();
  }

  tof::Camera& cam;
  tof::FrameType frame_type;

  sensor_msgs::msg::Image image;

  tof_user_pointer_destructed_t cb = [](uint8_t*, size_t, void* user_data) {
    auto data = static_cast<CallbackImage*>(user_data);
    try {
      data->add_user_pointer();
    } catch (std::exception& e) {
      delete data;
    }
  };

  void add_user_pointer() {
    cam.add_user_pointer(image.data.data(), image.data.size(), cb, this,
                         frame_type);
  }
};

struct CallbackPointCloud {
  CallbackPointCloud(tof::Camera& cam_, tof::FrameType frame_type_,
                     uint32_t width, uint32_t height)
      : cam(cam_), frame_type(frame_type_) {
    point_cloud.fields = create_point_fields(frame_type);
    point_cloud.width = width;
    point_cloud.height = height;
    point_cloud.is_bigendian = false;
    point_cloud.is_dense = true;
    point_cloud.point_step = 16;
    point_cloud.row_step = width * 16;

    point_cloud.data.resize(width * height * 16);
    add_user_pointer();
  }

  tof::Camera& cam;
  tof::FrameType frame_type;

  sensor_msgs::msg::PointCloud2 point_cloud;

  tof_user_pointer_destructed_t cb = [](uint8_t*, size_t, void* user_data) {
    auto data = static_cast<CallbackPointCloud*>(user_data);
    try {
      data->add_user_pointer();
    } catch (std::exception& e) {
      delete data;
    }
  };

  void add_user_pointer() {
    cam.add_user_pointer(point_cloud.data.data(), point_cloud.data.size(), cb,
                         this, frame_type);
  }
};

void queue_user_pointers(tof::Camera& cam, const size_t queue_size,
                         const bool has_rgb) {
  cam.set_user_pointer_capacity(queue_size);
  auto roi = cam.get_camera_config().get_roi(0);

  uint32_t width = roi.get_img_cols();
  uint32_t height = roi.get_img_rows();

  auto queue_image = [&](tof::FrameType frame_type,
                         const std::string& encoding) {
    for (size_t i = 0; i < queue_size; i++) {
      new CallbackImage(cam, frame_type, width, height, encoding);
    }
  };

  queue_image(tof::FrameType::RADIAL, sensor_msgs::image_encodings::TYPE_16UC1);
  queue_image(tof::FrameType::INTENSITY,
              sensor_msgs::image_encodings::TYPE_8UC1);
  queue_image(tof::FrameType::REFLECTIVITY,
              sensor_msgs::image_encodings::TYPE_8UC1);
  queue_image(tof::FrameType::Z, sensor_msgs::image_encodings::TYPE_32FC1);

  if (has_rgb) {
    auto cam_config = cam.get_camera_config();
    uint32_t rgb_width = cam_config.get_rgb_width();
    uint32_t rgb_height = cam_config.get_rgb_height();

    for (size_t i = 0; i < queue_size; i++) {
      new CallbackImage(cam, tof::FrameType::BGR, rgb_width, rgb_height,
                        sensor_msgs::image_encodings::BGR8);
    }

    queue_image(tof::FrameType::BGR_PROJECTED,
                sensor_msgs::image_encodings::BGR8);
  }

  auto queue_point_cloud = [&](tof::FrameType frame_type) {
    for (size_t i = 0; i < queue_size; i++) {
      new CallbackPointCloud(cam, frame_type, width, height);
    }
  };

  queue_point_cloud(tof::FrameType::XYZ);

  if (has_rgb) {
    queue_point_cloud(tof::FrameType::XYZ_BGR);
    queue_point_cloud(tof::FrameType::XYZ_BGR_I);
  }
}

void update_header(tof::Data& frame, std_msgs::msg::Header& hdr) {
  hdr.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  hdr.frame_id = std::to_string(frame.frame_id());
}

void publish_image(
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher,
    tof::Data& frame, rclcpp::Logger const& logger,
    const std_msgs::msg::Header& hdr) {
  if (!publisher) {
    return;
  } else if (!frame.user_allocated()) {
    RCLCPP_ERROR(logger, "Frame not allocated dropping data");
    // TODO: Copy data
  } else {
    auto cb = static_cast<CallbackImage*>(frame.user_pointer());
    cb->image.header = hdr;
    publisher->publish(cb->image);
  }
}

void publish_pointcloud(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
    tof::Data& frame, rclcpp::Logger const& logger,
    const std_msgs::msg::Header& hdr) {
  if (!frame.user_allocated()) {
    RCLCPP_ERROR(logger, "Frame not allocated dropping data");
    // TODO: Copy data
  } else {
    auto cb = static_cast<CallbackPointCloud*>(frame.user_pointer());
    cb->point_cloud.header = hdr;

    // Turn xyz data into meters
    float* data = reinterpret_cast<float*>(cb->point_cloud.data.data());
    const size_t size = frame.rows() * frame.cols();
    for (size_t i = 0; i < size; i++) {
      data[i * 4] /= 1000.f;      // x
      data[i * 4 + 1] /= 1000.f;  // y
      data[i * 4 + 2] /= 1000.f;  // z
      // Don't divide the fourth piece of data since that won't be x, y or z
    }

    publisher->publish(cb->point_cloud);
  }
}

void publish_frames(FramePublishers& publishers, std::vector<tof::Data>& frames,
                    rclcpp::Logger const& logger,
                    const std_msgs::msg::Header& hdr) {
  for (auto& frame : frames) {
    switch (frame.frame_type()) {
      case tof::FrameType::RADIAL:
        publish_image(publishers.radial.pub, frame, logger, hdr);
        break;
      case tof::FrameType::INTENSITY:
        publish_image(publishers.intensity.pub, frame, logger, hdr);
        break;
      case tof::FrameType::REFLECTIVITY:
        publish_image(publishers.reflectivity.pub, frame, logger, hdr);
        break;
      case tof::FrameType::Z:
        publish_image(publishers.z.pub, frame, logger, hdr);
        break;
      case tof::FrameType::BGR:
        publish_image(publishers.bgr.pub, frame, logger, hdr);
        break;
      case tof::FrameType::BGR_PROJECTED:
        publish_image(publishers.bgr_projected.pub, frame, logger, hdr);
        break;

      case tof::FrameType::XYZ:
        publish_pointcloud(publishers.xyz.pub, frame, logger, hdr);
        break;
      case tof::FrameType::XYZ_BGR:
        publish_pointcloud(publishers.xyzbgr.pub, frame, logger, hdr);
        break;
      case tof::FrameType::XYZ_BGR_I:
        publish_pointcloud(publishers.xyzbgri.pub, frame, logger, hdr);
        break;

      default:
        RCLCPP_ERROR(logger, "Unexpected tof frame type");
        break;
    }
  }
}

static sensor_msgs::msg::CameraInfo create_depth_camera_info(tof::Camera& cam) {
  sensor_msgs::msg::CameraInfo cam_info;

  auto camera_config = cam.get_camera_config();
  auto calibration = cam.get_calibration();
  auto roi = camera_config.get_roi(0);

  cam_info.width = roi.sensor_cols();
  cam_info.height = roi.sensor_rows();

  cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  auto d = calibration.get_depth_distortion_coefficients();
  cam_info.d.insert(cam_info.d.end(), d.begin(), d.end());

  auto k = calibration.get_depth_camera_matrix();
  std::copy(k.begin(), k.end(), cam_info.k.begin());
  cam_info.k[8] = 1.;

  // p can use same params as k
  cam_info.p[0] = cam_info.k[0];
  cam_info.p[2] = cam_info.k[2];
  cam_info.p[5] = cam_info.k[4];
  cam_info.p[6] = cam_info.k[5];
  cam_info.p[10] = cam_info.k[8];

  cam_info.binning_x = camera_config.get_binning(0) + 1;
  cam_info.binning_y = cam_info.binning_x;

  // The roi should be in the original image dimensions
  cam_info.roi.width = roi.sensor_cols();
  cam_info.roi.height = roi.sensor_rows();
  cam_info.roi.x_offset = roi.get_col_offset();
  cam_info.roi.y_offset = roi.get_row_offset();

  return cam_info;
}

static sensor_msgs::msg::CameraInfo create_rgb_camera_info(tof::Camera& cam) {
  sensor_msgs::msg::CameraInfo cam_info;

  auto camera_config = cam.get_camera_config();
  auto calibration = cam.get_calibration();

  cam_info.width = camera_config.get_rgb_width();
  cam_info.height = camera_config.get_rgb_height();

  cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  auto d = calibration.get_rgb_distortion_coefficients();
  cam_info.d.insert(cam_info.d.end(), d.begin(), d.end());

  auto k = calibration.get_rgb_camera_matrix();
  std::copy(k.begin(), k.end(), cam_info.k.begin());

  return cam_info;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("original");

  std::thread spin{[node]() { rclcpp::spin(node); }};

  // Serial can be empty
  std::string serial;
  node->get_parameter_or("serial", serial, std::string());

  bool log;
  node->get_parameter_or("log", log, true);

  int queue_size;
  node->get_parameter_or("queue_size", queue_size, 10);

  // log_fun needs to stay in existence
  auto logger = node->get_logger();
  std::function<void(int16_t, const char*, const char*)> log_fun =
      [logger](int16_t level, const char* log_name, const char* message) {
        if (level == 2) {
          RCLCPP_INFO(logger, "%s: %s", log_name, message);
        } else if (level == 3) {
          RCLCPP_WARN(logger, "%s: %s", log_name, message);
        } else if (level >= 4) {
          RCLCPP_ERROR(logger, "%s: %s", log_name, message);
        }
      };
  if (log) tof::log_callback_separated(log_fun);

  tof::TuiCamera cam(serial);

  RCLCPP_INFO(logger, "Connected to camera %s", cam.get_serial());

  const bool has_rgb = has_camera_rgb(cam);
  auto publishers = create_publishers(node, has_rgb);

  auto depth_cam_info = create_depth_camera_info(cam);
  sensor_msgs::msg::CameraInfo rgb_cam_info;
  if (has_rgb) rgb_cam_info = create_rgb_camera_info(cam);

  queue_user_pointers(cam, queue_size, has_rgb);

  std::vector<tof::FrameType> current_frame_types;
  rclcpp::Rate rate(100.0);
  while (rclcpp::ok() && cam.is_connected()) {
    update_streams(cam, publishers, current_frame_types, logger);

    if (cam.is_streaming() && cam.has_frames()) {
      auto frames = cam.get_frames();

      if (!frames.empty()) {
        update_header(frames[0], depth_cam_info.header);
        publishers.camera_info->publish(depth_cam_info);
        if (has_rgb) {
          rgb_cam_info.header = depth_cam_info.header;
          publishers.camera_info_rgb->publish(rgb_cam_info);
        }

        publish_frames(publishers, frames, logger, depth_cam_info.header);
      }
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  spin.join();
}