// camera_aravis
//
// This is a ROS node that operates GenICam-based cameras via the Aravis library.
// Commonly available camera features are supported through the dynamic_reconfigure user-interface and GUI,
// and for those features not in the GUI but that are specific to a camera, they can be set in the
// camera by setting the appropriate parameter at startup.  This code reads those parameters, and
// if any of them match a camera feature, then the camera is written to.
//
// For example, if a camera has a feature called "IRFormat" that is an integer 0, 1, or 2, you can do
// rosparam set camnode/IRFormat 2
// and this driver will write it to the camera at startup.  Note that the datatype of the parameter
// must be correct for the camera feature (e.g. bool, int, double, string, etc), so for example you should use
// rosparam set camnode/GainAuto true
// and NOT
// rosparam set camnode/GainAuto 1
//

#include <arv.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int64.hpp>

#include <chrono>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <glib.h>

using namespace std::chrono_literals;

//#define TUNING	// Allows tuning the gains for the timestamp controller.  Publishes output on topic /dt, and receives gains on params /kp, /ki, /kd

#define CLIP(x, lo, hi)   MIN(MAX((lo), (x)), (hi))
#define THROW_ERROR(m) throw std::string((m))

#define TRIGGERSOURCE_SOFTWARE  0
#define TRIGGERSOURCE_LINE1             1
#define TRIGGERSOURCE_LINE2             2

#define ARV_PIXEL_FORMAT_BIT_PER_PIXEL(pixel_format)  (((pixel_format) >> 16) & 0xff)
#define ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(pixel_format) ((((pixel_format) >> 16) & 0xff) >> 3)

// TODO FIXME Dynamic Reconfigure
#if 0
static gboolean SoftwareTrigger_callback(void *);
#endif

std::string kNodeName = "camera";

typedef struct
{
  const char * szName;
  const char * szTag;
  ArvDomNode * pNode;
  ArvDomNode * pNodeSibling;
} NODEEX;

struct Config
{
  bool acquire;
  std::string device_id;
  std::string exposure_auto;
  std::string gain_auto;
  double exposure_time_abs;
  double gain;
  std::string acquisition_mode;
  double acquisition_frame_rate;
  std::string trigger_mode;
  std::string trigger_source;
  double trigger_rate;
  int focus_pos;
  std::string frame_id;
  int mtu;
  std::string pixel_format;
  int target_brightness;
};

// Global variables -------------------
struct global_s
{
  gboolean bCancel;
  std::shared_ptr<image_transport::CameraPublisher> publisher;
  std::shared_ptr<camera_info_manager::CameraInfoManager> pCameraInfoManager;
  std::shared_ptr<image_transport::ImageTransport> pImageTransport;
  sensor_msgs::msg::CameraInfo camerainfo;
  gint width, height; // buffer->width and buffer->height not working, so I used a global.
  Config config;
  Config configMin;
  Config configMax;
  int idSoftwareTriggerTimer;

  int isImplementedAcquisitionFrameRate;
  int isImplementedAcquisitionFrameRateEnable;
  int isImplementedGain;
  int isImplementedExposureTimeAbs;
  int isImplementedExposureAuto;
  int isImplementedGainAuto;
  int isImplementedFocusPos;
  int isImplementedTriggerSelector;
  int isImplementedTriggerSource;
  int isImplementedTriggerMode;
  int isImplementedAcquisitionMode;
  int isImplementedMtu;

  int xRoi;
  int yRoi;
  int widthRoi;
  int widthRoiMin;
  int widthRoiMax;
  int heightRoi;
  int heightRoiMin;
  int heightRoiMax;

  int widthSensor;
  int heightSensor;

  const char * pszPixelformat;
  unsigned nBytesPixel;
  std::shared_ptr<rclcpp::Node> pNode;
  ArvCamera * pCamera;
  ArvDevice * pDevice;
  int mtu;
  int acquire;
  const char * keyAcquisitionFrameRate;
#ifdef TUNING
  rclcpp::Publisher::SharedPtr ppubInt64;
#endif

} global;

typedef struct
{
  GMainLoop * main_loop;
  int nBuffers;                 // Counter for Hz calculation.
} ApplicationData;


// ------------------------------------

// Conversions from integers to Arv types.
const char * szBufferStatusFromInt[] = {
  "ARV_BUFFER_STATUS_SUCCESS",
  "ARV_BUFFER_STATUS_CLEARED",
  "ARV_BUFFER_STATUS_TIMEOUT",
  "ARV_BUFFER_STATUS_MISSING_PACKETS",
  "ARV_BUFFER_STATUS_WRONG_PACKET_ID",
  "ARV_BUFFER_STATUS_SIZE_MISMATCH",
  "ARV_BUFFER_STATUS_FILLING",
  "ARV_BUFFER_STATUS_ABORTED"
};

// New Parameter Stuff
void declareParameters()
{
  RCLCPP_INFO(
    rclcpp::get_logger(""),
    "DECLARE CAM NODE PARAMETERS");
  // Declare the Test Parameters
  global.config.device_id = global.pNode->declare_parameter("device_id", "");
  global.config.acquire = global.pNode->declare_parameter("acquire", false);
  global.config.exposure_auto = global.pNode->declare_parameter("exposure_auto", "");
  global.config.gain_auto = global.pNode->declare_parameter("gain_auto", "");
  global.config.exposure_time_abs = global.pNode->declare_parameter("exposure_time_abs", 0.0);
  global.config.gain = global.pNode->declare_parameter("gain", 0.0);
  global.config.acquisition_mode = global.pNode->declare_parameter("acquisition_mode", "");
  global.config.acquisition_frame_rate = global.pNode->declare_parameter("acquisition_frame_rate", 0.0);
  global.config.trigger_mode = global.pNode->declare_parameter("trigger_mode", "");
  global.config.trigger_source = global.pNode->declare_parameter("trigger_source", "");
  global.config.trigger_rate = global.pNode->declare_parameter("trigger_rate", 0.0);
  global.config.focus_pos = global.pNode->declare_parameter("focus_pos", 0);
  global.config.frame_id = global.pNode->declare_parameter("frame_id", "");
  global.config.mtu = global.pNode->declare_parameter("gev_scps_packet_size", 0);
  global.config.pixel_format = global.pNode->declare_parameter("pixel_format", "");
  global.config.target_brightness = global.pNode->declare_parameter("target_brightness", 0);
}

void setLocalParameters()
{
  RCLCPP_INFO(
    rclcpp::get_logger(""),
    "SETTING LOCAL CAM NODE PARAMETERS");
  // Declare the Test Parameters
  global.pNode->get_parameter("device_id", global.config.device_id);
  global.pNode->get_parameter("acquire", global.config.acquire);
  global.pNode->get_parameter("exposure_auto", global.config.exposure_auto);
  global.pNode->get_parameter("gain_auto", global.config.gain_auto);
  global.pNode->get_parameter("exposure_time_abs", global.config.exposure_time_abs);
  global.pNode->get_parameter("gain", global.config.gain);
  global.pNode->get_parameter("acquisition_mode", global.config.acquisition_mode);
  global.pNode->get_parameter("acuqisition_frame_rate", global.config.acquisition_frame_rate);
  global.pNode->get_parameter("trigger_mode", global.config.trigger_mode);
  global.pNode->get_parameter("trigger_source", global.config.trigger_source);
  global.pNode->get_parameter("trigger_rate", global.config.trigger_rate);
  global.pNode->get_parameter("focus_pos", global.config.focus_pos);
  global.pNode->get_parameter("frame_id", global.config.frame_id);
  global.pNode->get_parameter("gev_scps_packet_size", global.config.mtu);
  global.pNode->get_parameter("pixel_format", global.config.pixel_format);
  global.pNode->get_parameter("target_brightness", global.config.target_brightness);
}

void updateParameterValue(const rcl_interfaces::msg::Parameter param)
{
  if (param.name == "acquire") {
    global.config.acquire = param.value.bool_value;
  } else if (param.name == "exposure_auto") {
    global.config.exposure_auto = param.value.string_value;
  } else if (param.name == "gain_auto") {
    global.config.gain_auto = param.value.string_value;
  } else if (param.name == "exposure_time_abs") {
    global.config.exposure_time_abs = param.value.double_value;
  } else if (param.name == "gain") {
    global.config.gain = param.value.double_value;
  } else if (param.name == "acquisition_mode") {
    global.config.acquisition_mode = param.value.string_value;
  } else if (param.name == "acquisition_frame_rate") {
    global.config.acquisition_frame_rate = param.value.double_value;
  } else if (param.name == "trigger_mode") {
    global.config.trigger_mode = param.value.string_value;
  } else if (param.name == "trigger_source") {
    global.config.trigger_source = param.value.string_value;
  } else if (param.name == "trigger_rate") {
    global.config.trigger_rate = param.value.double_value;
  } else if (param.name == "focus_pos") {
    global.config.focus_pos = param.value.integer_value;
  } else if (param.name == "frame_id") {
    global.config.frame_id = param.value.string_value;
  } else if (param.name == "gev_scps_packet_size") {
    global.config.mtu = param.value.integer_value;
  } else if (param.name == "pixel_format") {
    global.config.pixel_format = param.value.string_value;
  } else if (param.name == "target_brightness") {
    global.config.target_brightness = param.value.integer_value;
  }
}

static void set_cancel(int signal)
{
  (void)signal;
  global.bCancel = TRUE;
}

ArvGvStream * CreateStream(void)
{
  gboolean bAutoBuffer = FALSE;
  gboolean bPacketResend = TRUE;
  unsigned int timeoutPacket = 40;          // milliseconds
  unsigned int timeoutFrameRetention = 200;

  ArvGvStream * pStream = (ArvGvStream *)arv_device_create_stream(global.pDevice, NULL, NULL);
  if (pStream) {
    ArvBuffer * pBuffer;
    gint nbytesPayload;

    if (!ARV_IS_GV_STREAM(pStream)) {
      RCLCPP_WARN(global.pNode->get_logger(), "Stream is not a GV_STREAM");
    }

    if (bAutoBuffer) {
      g_object_set(
        pStream,
        "socket-buffer",
        ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
        "socket-buffer-size", 0,
        NULL);
    }
    if (!bPacketResend) {
      g_object_set(
        pStream,
        "packet-resend",
        bPacketResend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS : ARV_GV_STREAM_PACKET_RESEND_NEVER,
        NULL);
    }
    g_object_set(
      pStream,
      "packet-timeout",
      (unsigned) timeoutPacket * 1000,
      "frame-retention", (unsigned) timeoutFrameRetention * 1000,
      NULL);

    // Load up some buffers.
    nbytesPayload = arv_camera_get_payload(global.pCamera);
    for (int i = 0; i < 50; i++) {
      pBuffer = arv_buffer_new(nbytesPayload, NULL);
      arv_stream_push_buffer((ArvStream *)pStream, pBuffer);
    }
  }
  return pStream;
}

// TODO FIXME Dynamic Reconfigure TBD
#if 0
void RosReconfigure_callback(Config & config, uint32_t level)
{
  int changedAcquire;
  int changedAcquisitionFrameRate;
  int changedExposureAuto;
  int changedGainAuto;
  int changedExposureTimeAbs;
  int changedGain;
  int changedAcquisitionMode;
  int changedTriggerMode;
  int changedTriggerSource;
  int changedSoftwarerate;
  int changedFrameid;
  int changedFocusPos;
  int changedMtu;

  if (config.frame_id == "") {
    config.frame_id = "camera";
  }

  // Find what the user changed.
  changedAcquire = (global.config.acquire != config.acquire);
  changedAcquisitionFrameRate =
    (global.config.acquisition_frame_rate != config.acquisition_frame_rate);
  changedExposureAuto = (global.config.exposure_auto != config.exposure_auto);
  changedExposureTimeAbs = (global.config.exposure_time_abs != config.exposure_time_abs);
  changedGainAuto = (global.config.gain_auto != config.gain_auto);
  changedGain = (global.config.gain != config.gain);
  changedAcquisitionMode = (global.config.acquisition_mode != config.acquisition_mode);
  changedTriggerMode = (global.config.trigger_mode != config.trigger_mode);
  changedTriggerSource = (global.config.trigger_source != config.trigger_source);
  changedSoftwarerate = (global.config.trigger_rate != config.trigger_rate);
  changedFrameid = (global.config.frame_id != config.frame_id);
  changedFocusPos = (global.config.focus_pos != config.focus_pos);
  changedMtu = (global.config.mtu != config.mtu);

  // Limit params to legal values.
  config.acquisition_frame_rate = CLIP(
    config.acquisition_frame_rate,
    global.configMin.acquisition_frame_rate,
    global.configMax.acquisition_frame_rate);
  config.exposure_time_abs = CLIP(
    config.exposure_time_abs, global.configMin.exposure_time_abs,
    global.configMax.exposure_time_abs);
  config.gain = CLIP(config.gain, global.configMin.gain, global.configMax.gain);
  config.focus_pos = CLIP(config.focus_pos, global.configMin.focus_pos, global.configMax.focus_pos);

  // Adjust other controls dependent on what the user changed.
  if (changedExposureTimeAbs || changedGainAuto ||
    ((changedAcquisitionFrameRate || changedGain || changedFrameid ||
    changedAcquisitionMode || changedTriggerSource || changedSoftwarerate) &&
    config.exposure_auto == "Once"))
  {
    config.exposure_auto = "Off";
  }

  if (changedGain || changedExposureAuto ||
    ((changedAcquisitionFrameRate || changedExposureTimeAbs || changedFrameid ||
    changedAcquisitionMode || changedTriggerSource || changedSoftwarerate) &&
    config.gain_auto == "Once"))
  {
    config.gain_auto = "Off";
  }

  if (changedAcquisitionFrameRate) {
    config.trigger_mode = "Off";
  }

  // Find what changed for any reason.
  changedAcquire = (global.config.acquire != config.acquire);
  changedAcquisitionFrameRate =
    (global.config.acquisition_frame_rate != config.acquisition_frame_rate);
  changedExposureAuto = (global.config.exposure_auto != config.exposure_auto);
  changedExposureTimeAbs = (global.config.exposure_time_abs != config.exposure_time_abs);
  changedGainAuto = (global.config.gain_auto != config.gain_auto);
  changedGain = (global.config.gain != config.gain);
  changedAcquisitionMode = (global.config.acquisition_mode != config.acquisition_mode);
  changedTriggerMode = (global.config.trigger_mode != config.trigger_mode);
  changedTriggerSource = (global.config.trigger_source != config.trigger_source);
  changedSoftwarerate = (global.config.trigger_rate != config.trigger_rate);
  changedFrameid = (global.config.frame_id != config.frame_id);
  changedFocusPos = (global.config.focus_pos != config.focus_pos);
  changedMtu = (global.config.mtu != config.mtu);

  // Set params into the camera.
  if (changedExposureTimeAbs) {
    if (global.isImplementedExposureTimeAbs) {
      RCLCPP_INFO(global.pNode->get_logger(), "Set ExposureTimeAbs = %f", config.exposure_time_abs);
      arv_device_set_float_feature_value(
        global.pDevice, "ExposureTimeAbs",
        config.exposure_time_abs);
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "Camera does not support ExposureTimeAbs.");
    }
  }

  if (changedGain) {
    if (global.isImplementedGain) {
      RCLCPP_INFO(global.pNode->get_logger(), "Set gain = %f", config.gain);
      //arv_device_set_integer_feature_value (global.pDevice, "GainRaw", config.GainRaw);
      arv_camera_set_gain(global.pCamera, config.gain);
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "Camera does not support Gain or GainRaw.");
    }
  }

  rclcpp::Rate timeout(1);
  if (changedExposureAuto) {
    if (global.isImplementedExposureAuto && global.isImplementedExposureTimeAbs) {
      RCLCPP_INFO(
        global.pNode->get_logger(), "Set ExposureAuto = %s",
        config.exposure_auto.c_str());
      arv_device_set_string_feature_value(
        global.pDevice, "ExposureAuto",
        config.exposure_auto.c_str());
      if (config.exposure_auto == "Once") {
        timeout.sleep();
        config.exposure_time_abs = arv_device_get_float_feature_value(
          global.pDevice,
          "ExposureTimeAbs");
        RCLCPP_INFO(
          global.pNode->get_logger(), "Get ExposureTimeAbs = %f",
          config.exposure_time_abs);
        config.exposure_auto = "Off";
      }
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "Camera does not support ExposureAuto.");
    }
  }
  if (changedGainAuto) {
    if (global.isImplementedGainAuto && global.isImplementedGain) {
      RCLCPP_INFO(global.pNode->get_logger(), "Set GainAuto = %s", config.gain_auto.c_str());
      arv_device_set_string_feature_value(global.pDevice, "GainAuto", config.gain_auto.c_str());
      if (config.gain_auto == "Once") {
        timeout.sleep();
        //config.GainRaw = arv_device_get_integer_feature_value (global.pDevice, "GainRaw");
        config.gain = arv_camera_get_gain(global.pCamera);
        RCLCPP_INFO(global.pNode->get_logger(), "Get Gain = %f", config.gain);
        config.gain_auto = "Off";
      }
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "Camera does not support GainAuto.");
    }
  }

  if (changedAcquisitionFrameRate) {
    if (global.isImplementedAcquisitionFrameRate) {
      RCLCPP_INFO(
        global.pNode->get_logger(), "Set %s = %f", global.keyAcquisitionFrameRate,
        config.acquisition_frame_rate);
      arv_device_set_float_feature_value(
        global.pDevice, global.keyAcquisitionFrameRate,
        config.acquisition_frame_rate);
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "Camera does not support AcquisitionFrameRate");
    }
  }

  if (changedTriggerMode) {
    if (global.isImplementedTriggerMode) {
      RCLCPP_INFO(global.pNode->get_logger(), "Set TriggerMode = %s", config.trigger_mode.c_str());
      arv_device_set_string_feature_value(
        global.pDevice, "TriggerMode",
        config.trigger_mode.c_str());
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "Camera does not support TriggerMode.");
    }
  }

  if (changedTriggerSource) {
    if (global.isImplementedTriggerSource) {
      RCLCPP_INFO(
        global.pNode->get_logger(), "Set TriggerSource = %s",
        config.trigger_source.c_str());
      arv_device_set_string_feature_value(
        global.pDevice, "TriggerSource",
        config.trigger_source.c_str());
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "Camera does not support TriggerSource.");
    }
  }

  if ((changedTriggerMode || changedTriggerSource || changedSoftwarerate) &&
    config.trigger_mode == "On" && config.trigger_source == "Software")
  {
    if (global.isImplementedAcquisitionFrameRate) {
      // The software rate is limited by the camera's internal framerate.  Bump up the camera's internal framerate if necessary.
      config.acquisition_frame_rate = global.configMax.acquisition_frame_rate;
      RCLCPP_INFO(
        global.pNode->get_logger(), "Set %s = %f", global.keyAcquisitionFrameRate,
        config.acquisition_frame_rate);
      arv_device_set_float_feature_value(
        global.pDevice, global.keyAcquisitionFrameRate,
        config.acquisition_frame_rate);
    }
  }

  if (changedTriggerSource || changedSoftwarerate) {
    // Recreate the software trigger callback.
    if (global.idSoftwareTriggerTimer) {
      g_source_remove(global.idSoftwareTriggerTimer);
      global.idSoftwareTriggerTimer = 0;
    }
    if (!strcmp(config.trigger_source.c_str(), "Software")) {
      RCLCPP_INFO(
        global.pNode->get_logger(),
        "Set softwaretriggerrate = %f",
        1000.0 / ceil(1000.0 / config.trigger_rate));

      // Turn on software timer callback.
      global.idSoftwareTriggerTimer = g_timeout_add(
        (guint)ceil(
          1000.0 / config.trigger_rate), SoftwareTrigger_callback, global.pCamera);
    }
  }
  if (changedFocusPos) {
    if (global.isImplementedFocusPos) {
      RCLCPP_INFO(global.pNode->get_logger(), "Set FocusPos = %d", config.focus_pos);
      arv_device_set_integer_feature_value(global.pDevice, "FocusPos", config.focus_pos);
      timeout.sleep();
      config.focus_pos = arv_device_get_integer_feature_value(global.pDevice, "FocusPos");
      RCLCPP_INFO(global.pNode->get_logger(), "Get FocusPos = %d", config.focus_pos);
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "Camera does not support FocusPos.");
    }
  }
  if (changedMtu) {
    if (global.isImplementedMtu) {
      RCLCPP_INFO(global.pNode->get_logger(), "Set mtu = %d", config.mtu);
      arv_device_set_integer_feature_value(global.pDevice, "GevSCPSPacketSize", config.mtu);
      timeout.sleep();
      config.mtu = arv_device_get_integer_feature_value(global.pDevice, "GevSCPSPacketSize");
      RCLCPP_INFO(global.pNode->get_logger(), "Get mtu = %d", config.mtu);
    } else {
      RCLCPP_INFO(
        global.pNode->get_logger(), "Camera does not support mtu (i.e. GevSCPSPacketSize).");
    }
  }

  if (changedAcquisitionMode) {
    if (global.isImplementedAcquisitionMode) {
      RCLCPP_INFO(
        global.pNode->get_logger(), "Set AcquisitionMode = %s", config.acquisition_mode.c_str());
      arv_device_set_string_feature_value(
        global.pDevice, "AcquisitionMode",
        config.acquisition_mode.c_str());

      RCLCPP_INFO(global.pNode->get_logger(), "AcquisitionStop");
      arv_device_execute_command(global.pDevice, "AcquisitionStop");
      RCLCPP_INFO(global.pNode->get_logger(), "AcquisitionStart");
      arv_device_execute_command(global.pDevice, "AcquisitionStart");
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "Camera does not support AcquisitionMode.");
    }
  }

  if (changedAcquire) {
    if (config.acquire) {
      RCLCPP_INFO(global.pNode->get_logger(), "AcquisitionStart");
      arv_device_execute_command(global.pDevice, "AcquisitionStart");
    } else {
      RCLCPP_INFO(global.pNode->get_logger(), "AcquisitionStop");
      arv_device_execute_command(global.pDevice, "AcquisitionStop");
    }
  }

  global.config = config;
} // RosReconfigure_callback()
#endif

static void NewBuffer_callback(ArvStream * pStream, ApplicationData * pApplicationdata)
{
  static uint64_t cm = 0L;            // Camera time prev
  uint64_t cn = 0L;                   // Camera time now

#ifdef TUNING
  static uint64_t rm = 0L;            // ROS time prev
#endif
  uint64_t rn = 0L;                   // ROS time now

  static uint64_t tm = 0L;            // Calculated image time prev
  uint64_t tn = 0L;                   // Calculated image time now

  static int64_t em = 0L;             // Error prev.
  int64_t en = 0L;                    // Error now between calculated image time and ROS time.
  int64_t de = 0L;                    // derivative.
  int64_t ie = 0L;                    // integral.
  int64_t u = 0L;                     // Output of controller.

  int64_t kp1 = 0L;                   // Fractional gains in integer form.
  int64_t kp2 = 1024L;
  int64_t kd1 = 0L;
  int64_t kd2 = 1024L;
  int64_t ki1 = -1L;                  // A gentle pull toward zero.
  int64_t ki2 = 1024L;

  static uint32_t iFrame = 0;         // Frame counter.

  ArvBuffer * pBuffer;

#ifdef TUNING
  std_msgs::msg::Int64 msgInt64;
  int kp = 0;
  int kd = 0;
  int ki = 0;

  if (global.pNode->has_parameter(kNodeName + "/kp")) {
    global.pNode->get_parameter(kNodeName + "/kp", kp);
    kp1 = kp;
  }

  if (global.pNode->has_parameter(kNodeName + "/kd")) {
    global.pNode->get_parameter(kNodeName + "/kd", kd);
    kd1 = kd;
  }

  if (global.pNode->has_parameter(kNodeName + "/ki")) {
    global.pNode->get_parameter(kNodeName + "/ki", ki);
    ki1 = ki;
  }
#endif

  pBuffer = arv_stream_try_pop_buffer(pStream);
  if (pBuffer != NULL) {
    if (arv_buffer_get_status(pBuffer) == ARV_BUFFER_STATUS_SUCCESS) {
      sensor_msgs::msg::Image msg;

      pApplicationdata->nBuffers++;
      size_t size;
      const void * ptr = arv_buffer_get_data(pBuffer, &size);
      std::vector<uint8_t> this_data(size);
      memcpy(&this_data[0], ptr, size);

      // Camera/ROS Timestamp coordination.
      cn = (uint64_t)arv_buffer_get_timestamp(pBuffer); // Camera now
      rclcpp::Clock clock;
      rn = clock.now().nanoseconds();

      if (iFrame < 10) {
        cm = cn;
        tm = rn;
      }

      // Control the error between the computed image timestamp and the ROS timestamp.
      en = (int64_t)tm + (int64_t)cn - (int64_t)cm - (int64_t)rn; // i.e. tn-rn, but calced from prior values.
      de = en - em;
      ie += en;
      u = kp1 * (en / kp2) + ki1 * (ie / ki2) + kd1 * (de / kd2); // kp<0, ki<0, kd>0

      // Compute the new timestamp.
      tn = (uint64_t)((int64_t)tm + (int64_t)cn - (int64_t)cm + u);

#ifdef TUNING
      RCLCPP_WARN(
        global.pNode->get_logger(),
        "en=%16ld, ie=%16ld, de=%16ld, u=%16ld + %16ld + %16ld = %16ld", en, ie, de,
        kp1 * (en / kp2), ki1 * (ie / ki2), kd1 * (de / kd2), u);
      RCLCPP_WARN(
        global.pNode->get_logger(),
        "cn=%16lu, rn=%16lu, cn-cm=%8ld, rn-rm=%8ld, tn-tm=%8ld, tn-rn=%ld", cn, rn, cn - cm,
        rn - rm, (int64_t)tn - (int64_t)tm, tn - rn);
      msgInt64.data = tn - rn;               //cn-cm+tn-tm; //
      global.ppubInt64->publish(msgInt64);
      rm = rn;
#endif

      // Save prior values.
      cm = cn;
      tm = tn;
      em = en;

      // Construct the image message.
      msg.header.stamp = clock.now();
      msg.header.frame_id = global.config.frame_id;
      msg.width = global.widthRoi;
      msg.height = global.heightRoi;
      msg.encoding = global.pszPixelformat;
      msg.step = msg.width * global.nBytesPixel;
      msg.data = this_data;

      // get current CameraInfo data
      global.camerainfo = global.pCameraInfoManager->getCameraInfo();
      global.camerainfo.header.stamp = msg.header.stamp;
      global.camerainfo.header.frame_id = msg.header.frame_id;
      global.camerainfo.width = global.widthRoi;
      global.camerainfo.height = global.heightRoi;

      global.publisher->publish(msg, global.camerainfo);

    } else {
      RCLCPP_WARN(
        global.pNode->get_logger(), "Frame error: %s",
        szBufferStatusFromInt[arv_buffer_get_status(pBuffer)]);
    }

    arv_stream_push_buffer(pStream, pBuffer);
    iFrame++;
  }
} // NewBuffer_callback()

static void ControlLost_callback(ArvGvDevice * pGvDevice)
{
  (void)pGvDevice;
  RCLCPP_ERROR(global.pNode->get_logger(), "Control lost.");

  global.bCancel = TRUE;
}

// TODO FIXME Dynamic Reconfigure TBD
#if 0
static gboolean SoftwareTrigger_callback(void * pCamera)
{
  (void)pCamera;
  arv_device_execute_command(global.pDevice, "TriggerSoftware");

  return TRUE;
}
#endif

// PeriodicTask_callback()
// Check for termination, and spin for ROS.
static gboolean PeriodicTask_callback(void * applicationdata)
{
  ApplicationData * pData = (ApplicationData *)applicationdata;

  //  RCLCPP_INFO (global.pNode->get_logger(), "Frame rate = %d Hz", pData->nBuffers);
  pData->nBuffers = 0;

  if (global.bCancel) {
    g_main_loop_quit(pData->main_loop);
    return FALSE;
  }

  if (rclcpp::ok()) {
    rclcpp::spin_some(global.pNode);
  } else {
    global.bCancel = TRUE;
    return FALSE;
  }
  return TRUE;
} // PeriodicTask_callback()

// Get the child and the child's sibling, where <p___> indicates an indirection.
NODEEX GetGcFirstChild(ArvGc * pGenicam, NODEEX nodeex)
{
  const char * szName = 0;

  if (nodeex.pNode) {
    nodeex.pNode = arv_dom_node_get_first_child(nodeex.pNode);
    if (nodeex.pNode) {
      nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
      nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

      // Do the indirection.
      if (nodeex.szName[0] == 'p' && strcmp("pInvalidator", nodeex.szName)) {
        szName = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.pNode));
        nodeex.pNode = (ArvDomNode *)arv_gc_get_node(pGenicam, szName);
        nodeex.szTag = arv_dom_node_get_node_name(nodeex.pNode);
      } else {
        nodeex.szTag = nodeex.szName;
      }
    } else {
      nodeex.pNodeSibling = NULL;
    }
  } else {
    nodeex.szName = NULL;
    nodeex.szTag = NULL;
    nodeex.pNodeSibling = NULL;
  }

  //RCLCPP_INFO(pNode->get_logger(), "GFC name=%s, node=%p, sib=%p", szNameChild, nodeex.pNode, nodeex.pNodeSibling);

  return nodeex;
} // GetGcFirstChild()

// Get the sibling and the sibling's sibling, where <p___> indicates an indirection.
NODEEX GetGcNextSibling(ArvGc * pGenicam, NODEEX nodeex)
{
  const char * szName = 0;

  // Go to the sibling.
  nodeex.pNode = nodeex.pNodeSibling;
  if (nodeex.pNode) {
    nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
    nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

    // Do the indirection.
    if (nodeex.szName[0] == 'p' && strcmp("pInvalidator", nodeex.szName)) {
      szName = arv_dom_node_get_node_value(arv_dom_node_get_first_child(nodeex.pNode));
      nodeex.pNode = (ArvDomNode *)arv_gc_get_node(pGenicam, szName);
      nodeex.szTag = nodeex.pNode ? arv_dom_node_get_node_name(nodeex.pNode) : NULL;
    } else {
      nodeex.szTag = nodeex.szName;
    }
  } else {
    nodeex.szName = NULL;
    nodeex.szTag = NULL;
    nodeex.pNodeSibling = NULL;
  }

  //RCLCPP_INFO(pNode->get_logger(), "GNS name=%s, node=%p, sib=%p", nodeex.szName, nodeex.pNode, nodeex.pNodeSibling);

  return nodeex;
} // GetGcNextSibling()

// Walk the DOM tree, i.e. the tree represented by the XML file in the camera, and that contains all the various features, parameters, etc.
void PrintDOMTree(ArvGc * pGenicam, NODEEX nodeex, int nIndent)
{
  char * szIndent = 0;
  const char * szFeature = 0;
  const char * szDomName = 0;
  const char * szFeatureValue = 0;

  szIndent = new char[nIndent + 1];
  memset(szIndent, ' ', nIndent);
  szIndent[nIndent] = 0;

  nodeex = GetGcFirstChild(pGenicam, nodeex);
  if (nodeex.pNode) {
    do {
      if (ARV_IS_GC_FEATURE_NODE((ArvGcFeatureNode *)nodeex.pNode)) {
        szDomName = arv_dom_node_get_node_name(nodeex.pNode);
        szFeature = arv_gc_feature_node_get_name((ArvGcFeatureNode *)nodeex.pNode);
        szFeatureValue = arv_gc_feature_node_get_value_as_string(
          (ArvGcFeatureNode *)nodeex.pNode,
          NULL);
        if (szFeature && szFeatureValue && szFeatureValue[0]) {
          RCLCPP_INFO(
            global.pNode->get_logger(), "FeatureName: %s%s, %s=%s", szIndent, szDomName, szFeature,
            szFeatureValue);
        }
      }
      PrintDOMTree(pGenicam, nodeex, nIndent + 4);

      // Go to the next sibling.
      nodeex = GetGcNextSibling(pGenicam, nodeex);

    } while (nodeex.pNode && nodeex.pNodeSibling);
  }
} //PrintDOMTree()

// WriteCameraFeaturesFromRosparam()
// Read ROS parameters from this node's namespace, and see if each parameter has a similarly named & typed feature in the camera.  Then set the
// camera feature to that value.  For example, if the parameter camnode/Gain is set to 123.0, then we'll write 123.0 to the Gain feature
// in the camera.
//
// Note that the datatype of the parameter *must* match the datatype of the camera feature, and this can be determined by
// looking at the camera's XML file.  Camera enum's are string parameters, camera bools are false/true parameters (not 0/1),
// integers are integers, doubles are doubles, etc.
//
void WriteCameraFeaturesFromRosparam(void)
{
    const char * key = "Acquire";
    arv_device_set_integer_feature_value(global.pDevice, key, global.config.acquire);

    key = "ExposureAuto";
    arv_device_set_string_feature_value(global.pDevice, key, global.config.exposure_auto.c_str());

    key = "GainAuto";
    arv_device_set_string_feature_value(global.pDevice, key, global.config.gain_auto.c_str());

    key = "ExposureTimeAbs";
    arv_device_set_float_feature_value(
      global.pDevice, key,
      global.config.exposure_time_abs);

    key = "Gain";
    arv_device_set_float_feature_value(global.pDevice, key, global.config.gain);

    key = "AcquisitionMode";
    arv_device_set_string_feature_value(
      global.pDevice, key,
      global.config.acquisition_mode.c_str());

    key = "AcquisitionFrameRate";
    arv_device_set_float_feature_value(
      global.pDevice, key,
      global.config.acquisition_frame_rate);
    RCLCPP_INFO_STREAM(global.pNode->get_logger(), "SETTING FRAME RATE ALLOWED: " << global.isImplementedAcquisitionFrameRate);
    RCLCPP_INFO_STREAM(global.pNode->get_logger(), "TRYING TO SET ACQUISITION FRAME RATE TO: " << global.config.acquisition_frame_rate);

    key = "TriggerMode";
    arv_device_set_string_feature_value(global.pDevice, key, global.config.trigger_mode.c_str());

    key = "TriggerSource";
    arv_device_set_string_feature_value(global.pDevice, key, global.config.trigger_source.c_str());

    key = "TriggerRate";
    arv_device_set_float_feature_value(global.pDevice, key, global.config.trigger_rate);

    key = "FocusPos";
    arv_device_set_integer_feature_value(global.pDevice, key, global.config.focus_pos);

    key = "GevSCPSPacketSize";
    arv_device_set_integer_feature_value(global.pDevice, key, global.config.mtu);

    key = "PixelFormat";
    arv_device_set_string_feature_value(global.pDevice, key, global.config.pixel_format.c_str());

    key = "TargetBrightness";
    arv_device_set_integer_feature_value(global.pDevice, key, global.config.target_brightness);
} // WriteCameraFeaturesFromRosparam()

void onParameterEvent(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event, rclcpp::Logger logger)
{
  for (auto & changed_parameter : event->changed_parameters) {
    RCLCPP_INFO(global.pNode->get_logger(), "Changed Parameter: %s", changed_parameter.name);
    updateParameterValue(changed_parameter);
  }
}

int main(int argc, char ** argv)
{
  char * pszGuid = NULL;
  char szGuid[512];
  int nInterfaces = 0;
  int nDevices = 0;
  int i = 0;
  const char * pkeyAcquisitionFrameRate[2] = {"AcquisitionFrameRate", "AcquisitionFrameRateAbs"};
  ArvGcNode * pGcNode;
  GError * error = NULL;

  global.bCancel = FALSE;
  global.idSoftwareTriggerTimer = 0;

  rclcpp::init(argc, argv);
  global.pNode = rclcpp::Node::make_shared(kNodeName);
  kNodeName = global.pNode->get_name();

  // Setup the parameter client
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(global.pNode);
  while (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        global.pNode->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(
      global.pNode->get_logger(),
      "service not available, waiting again...");
  }

  auto sub = parameters_client->on_parameter_event(
    [global](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      onParameterEvent(event, global.pNode->get_logger());
    });

  rclcpp::spin_some(global.pNode);
  rclcpp::sleep_for(5s);
  rclcpp::spin_some(global.pNode);
  
  declareParameters();
  //setLocalParameters();

  // Print out some useful info.
  RCLCPP_INFO(global.pNode->get_logger(), "Attached cameras:");
  arv_update_device_list();
  nInterfaces = arv_get_n_interfaces();
  RCLCPP_INFO(global.pNode->get_logger(), "# Interfaces: %d", nInterfaces);

  nDevices = arv_get_n_devices();
  RCLCPP_INFO(global.pNode->get_logger(), "# Devices: %d", nDevices);
  std::string stGuid;
  for (i = 0; i < nDevices; i++) {
    if(arv_get_device_id(i) == global.config.device_id)
    {
      strcpy(szGuid, global.config.device_id.c_str());
    }
    RCLCPP_ERROR(global.pNode->get_logger(), "Device%d: %s", i, arv_get_device_id(i));
  }
  pszGuid = szGuid;

  rclcpp::Rate timeout(1);
  if (nDevices > 0) {
    // Open the camera, and set it up.
    RCLCPP_ERROR(global.pNode->get_logger(), "Opening: %s", pszGuid ? pszGuid : "(any)");
    while (TRUE) {
      global.pCamera = arv_camera_new(pszGuid);
      if (global.pCamera) {
        break;
      } else {
        RCLCPP_WARN(global.pNode->get_logger(), "Could not open camera %s.  Retrying...", pszGuid);
        timeout.sleep();
        rclcpp::spin_some(global.pNode);
      }
    }

    global.pDevice = arv_camera_get_device(global.pCamera);
    RCLCPP_ERROR(
      global.pNode->get_logger(),
      "Opened: %s | %s", arv_device_get_string_feature_value(
        global.pDevice,
        "DeviceVendorName"
      ), arv_device_get_string_feature_value(
        global.pDevice,
        "DeviceID"
      )
    );

    // See if some basic camera features exist.
    pGcNode = arv_device_get_feature(global.pDevice, "AcquisitionMode");
    global.isImplementedAcquisitionMode =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "GainRaw");
    global.isImplementedGain = ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;
    pGcNode = arv_device_get_feature(global.pDevice, "Gain");
    global.isImplementedGain |= ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "ExposureTimeAbs");
    global.isImplementedExposureTimeAbs =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "ExposureAuto");
    global.isImplementedExposureAuto =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "GainAuto");
    global.isImplementedGainAuto =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "TriggerSelector");
    global.isImplementedTriggerSelector =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "TriggerSource");
    global.isImplementedTriggerSource =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "TriggerMode");
    global.isImplementedTriggerMode =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "FocusPos");
    global.isImplementedFocusPos =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "GevSCPSPacketSize");
    global.isImplementedMtu = ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    pGcNode = arv_device_get_feature(global.pDevice, "AcquisitionFrameRateEnable");
    global.isImplementedAcquisitionFrameRateEnable =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
      ARV_GC_FEATURE_NODE(
        pGcNode), &error) : FALSE;

    // Find the key name for framerate.
    global.keyAcquisitionFrameRate = NULL;
    for (i = 0; i < 2; i++) {
      pGcNode = arv_device_get_feature(global.pDevice, pkeyAcquisitionFrameRate[i]);
      global.isImplementedAcquisitionFrameRate = pGcNode ? arv_gc_feature_node_is_implemented(
        ARV_GC_FEATURE_NODE(
          pGcNode), &error) : FALSE;
      if (global.isImplementedAcquisitionFrameRate) {
        global.keyAcquisitionFrameRate = pkeyAcquisitionFrameRate[i];
        break;
      }
    }

    // Get parameter bounds.
    arv_camera_get_exposure_time_bounds(
      global.pCamera, &global.configMin.exposure_time_abs,
      &global.configMax.exposure_time_abs);
    arv_camera_get_gain_bounds(global.pCamera, &global.configMin.gain, &global.configMax.gain);
    arv_camera_get_sensor_size(global.pCamera, &global.widthSensor, &global.heightSensor);
    arv_camera_get_width_bounds(global.pCamera, &global.widthRoiMin, &global.widthRoiMax);
    arv_camera_get_height_bounds(global.pCamera, &global.heightRoiMin, &global.heightRoiMax);

    if (global.isImplementedFocusPos) {
      gint64 focusMin64, focusMax64;
      arv_device_get_integer_feature_bounds(global.pDevice, "FocusPos", &focusMin64, &focusMax64);
      global.configMin.focus_pos = focusMin64;
      global.configMax.focus_pos = focusMax64;
    } else {
      global.configMin.focus_pos = 0;
      global.configMax.focus_pos = 0;
    }

    global.configMin.acquisition_frame_rate = 0.0;
    global.configMax.acquisition_frame_rate = 1000.0;

    // Initial camera settings.
    if (global.isImplementedExposureTimeAbs) {
      arv_device_set_float_feature_value(
        global.pDevice, "ExposureTimeAbs",
        global.config.exposure_time_abs);
    }
    if (global.isImplementedGain) {
      arv_camera_set_gain(global.pCamera, global.config.gain);
    }
    //arv_device_set_integer_feature_value(global.pDevice, "GainRaw", global.config.GainRaw);
    if (global.isImplementedAcquisitionFrameRateEnable) {
      arv_device_set_integer_feature_value(global.pDevice, "AcquisitionFrameRateEnable", 1);
    }
    if (global.isImplementedAcquisitionFrameRate) {
      arv_device_set_float_feature_value(
        global.pDevice, global.keyAcquisitionFrameRate,
        global.config.acquisition_frame_rate);
    }

    // Set up the triggering.
    if (global.isImplementedTriggerMode) {
      if (global.isImplementedTriggerSelector && global.isImplementedTriggerMode) {
        arv_device_set_string_feature_value(
          global.pDevice, "TriggerSelector",
          "AcquisitionStart");
        arv_device_set_string_feature_value(global.pDevice, "TriggerMode", "Off");
        arv_device_set_string_feature_value(global.pDevice, "TriggerSelector", "FrameStart");
        arv_device_set_string_feature_value(global.pDevice, "TriggerMode", "Off");
      }
    }

    WriteCameraFeaturesFromRosparam();

#ifdef TUNING
    global.ppubInt64 = &pubInt64;
    global.ppubInt64 = global.pNode->create_publisher<std_msgs::msg::Int64>(
      kNodeName + "/dt", 100);
#endif

    // Start the camerainfo manager.
     std::string url = "file://" + ament_index_cpp::get_package_share_directory("camera_aravis") + "/" + global.pNode->get_name() + "/" + global.pNode->get_name() + ".yaml";
    //std::string url = std::string("file://${ROS_HOME}/camera_info/") + arv_device_get_string_feature_value(
    //  global.pDevice, "DeviceID") + std::string(".yaml");
    global.pCameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
      global.pNode.get(), kNodeName, url);

    // TODO FIXME Dynamic Reconfigure TBD
#if 0
    global.pNode->set_on_parameters_set_callback(
      std::bind(&RosReconfigure_callback, std::placeholders::_1));
    timeout.sleep();
#endif

    // Get parameter current values.
    global.xRoi = 0; global.yRoi = 0; global.widthRoi = 0; global.heightRoi = 0;
    arv_camera_get_region(
      global.pCamera, &global.xRoi, &global.yRoi, &global.widthRoi,
      &global.heightRoi);
    global.config.exposure_time_abs =
      global.isImplementedExposureTimeAbs ? arv_device_get_float_feature_value(
      global.pDevice,
      "ExposureTimeAbs") :
      0;
    global.config.gain = global.isImplementedGain ? arv_camera_get_gain(global.pCamera) : 0.0;
    global.pszPixelformat =
      g_string_ascii_down(
      g_string_new(
        arv_device_get_string_feature_value(
          global.pDevice,
          "PixelFormat")))->str;
    global.nBytesPixel =
      ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(
      arv_device_get_integer_feature_value(
        global.pDevice,
        "PixelFormat"));
    global.config.focus_pos = global.isImplementedFocusPos ? arv_device_get_integer_feature_value(
      global.pDevice, "FocusPos") : 0;

    // Print information.
    RCLCPP_INFO(global.pNode->get_logger(), "    Using Camera Configuration:");
    RCLCPP_INFO(global.pNode->get_logger(), "    ---------------------------");
    RCLCPP_INFO(
      global.pNode->get_logger(),
      "    Vendor name          = %s",
      arv_device_get_string_feature_value(global.pDevice, "DeviceVendorName"));
    RCLCPP_INFO(
      global.pNode->get_logger(),
      "    Model name           = %s",
      arv_device_get_string_feature_value(global.pDevice, "DeviceModelName"));
    RCLCPP_INFO(
      global.pNode->get_logger(),
      "    Device id            = %s",
      arv_device_get_string_feature_value(global.pDevice, "DeviceID"));
    RCLCPP_INFO(global.pNode->get_logger(), "    Sensor width         = %d", global.widthSensor);
    RCLCPP_INFO(global.pNode->get_logger(), "    Sensor height        = %d", global.heightSensor);
    RCLCPP_INFO(
      global.pNode->get_logger(),
      "    ROI x,y,w,h          = %d, %d, %d, %d", global.xRoi, global.yRoi, global.widthRoi,
      global.heightRoi);
    RCLCPP_INFO(
      global.pNode->get_logger(), "    Pixel format         = %s",
      global.pszPixelformat);
    RCLCPP_INFO(global.pNode->get_logger(), "    BytesPerPixel        = %d", global.nBytesPixel);
    RCLCPP_INFO(
      global.pNode->get_logger(),
      "    Acquisition Mode     = %s", global.isImplementedAcquisitionMode ? arv_device_get_string_feature_value(
        global.pDevice,
        "AcquisitionMode") : "(not implemented in camera)");
    RCLCPP_INFO(
      global.pNode->get_logger(),
      "    Trigger Mode         = %s", global.isImplementedTriggerMode ? arv_device_get_string_feature_value(
        global.pDevice,
        "TriggerMode") : "(not implemented in camera)");
    RCLCPP_INFO(
      global.pNode->get_logger(),
      "    Trigger Source       = %s", global.isImplementedTriggerSource ? arv_device_get_string_feature_value(
        global.pDevice,
        "TriggerSource") : "(not implemented in camera)");
    RCLCPP_INFO(
      global.pNode->get_logger(),
      "    Can set FrameRate:     %s",
      global.isImplementedAcquisitionFrameRate ? "True" : "False");
    if (global.isImplementedAcquisitionFrameRate) {
      global.config.acquisition_frame_rate = arv_device_get_float_feature_value(
        global.pDevice,
        global.keyAcquisitionFrameRate);
      RCLCPP_INFO(
        global.pNode->get_logger(), "    AcquisitionFrameRate = %g hz",
        global.config.acquisition_frame_rate);
    }

    RCLCPP_INFO(
      global.pNode->get_logger(),
      "    Can set Exposure:      %s",
      global.isImplementedExposureTimeAbs ? "True" : "False");
    if (global.isImplementedExposureTimeAbs) {
      RCLCPP_INFO(
        global.pNode->get_logger(),
        "    Can set ExposureAuto:  %s",
        global.isImplementedExposureAuto ? "True" : "False");
      RCLCPP_INFO(
        global.pNode->get_logger(),
        "    Exposure             = %g us in range [%g,%g]", global.config.exposure_time_abs,
        global.configMin.exposure_time_abs, global.configMax.exposure_time_abs);
    }

    RCLCPP_INFO(
      global.pNode->get_logger(), "    Can set Gain:          %s",
      global.isImplementedGain ? "True" : "False");
    if (global.isImplementedGain) {
      RCLCPP_INFO(
        global.pNode->get_logger(),
        "    Can set GainAuto:      %s",
        global.isImplementedGainAuto ? "True" : "False");
      RCLCPP_INFO(
        global.pNode->get_logger(),
        "    Gain                 = %f %% in range [%f,%f]", global.config.gain,
        global.configMin.gain, global.configMax.gain);
    }

    RCLCPP_INFO(
      global.pNode->get_logger(), "    Can set FocusPos:      %s",
      global.isImplementedFocusPos ? "True" : "False");

    if (global.isImplementedMtu) {
      RCLCPP_INFO(
        global.pNode->get_logger(),
        "    Network mtu          = %lu",
        arv_device_get_integer_feature_value(global.pDevice, "GevSCPSPacketSize"));
    }

    RCLCPP_INFO(global.pNode->get_logger(), "    ---------------------------");
    ArvGvStream * pStream = NULL;
    while (TRUE) {
      pStream = CreateStream();
      if (pStream) {
        break;
      } else {
        RCLCPP_WARN(
          global.pNode->get_logger(), "Could not create image stream for %s.  Retrying...",
          pszGuid);
        timeout.sleep();
        rclcpp::spin_some(global.pNode);
      }
    }

    ApplicationData applicationdata;
    applicationdata.nBuffers = 0;
    applicationdata.main_loop = 0;

    // Set up image_raw.
    global.pImageTransport = std::make_shared<image_transport::ImageTransport>(global.pNode);
    std::string topic(global.pNode->get_name());
    topic = "/" + topic + "/image";
    RCLCPP_ERROR(global.pNode->get_logger(), "PUBLISH ON TOPIC: %s", topic.c_str());
    global.publisher =
      std::make_shared<image_transport::CameraPublisher>(
      global.pImageTransport->advertiseCamera(topic, 1));

    // Connect signals with callbacks.
    g_signal_connect(pStream, "new-buffer", G_CALLBACK(NewBuffer_callback), &applicationdata);
    g_signal_connect(global.pDevice, "control-lost", G_CALLBACK(ControlLost_callback), NULL);
    g_timeout_add_seconds(1, PeriodicTask_callback, &applicationdata);
    arv_stream_set_emit_signals((ArvStream *)pStream, TRUE);

    void (* pSigintHandlerOld)(int);
    pSigintHandlerOld = signal(SIGINT, set_cancel);

    arv_device_execute_command(global.pDevice, "AcquisitionStart");

    applicationdata.main_loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(applicationdata.main_loop);

    if (global.idSoftwareTriggerTimer) {
      g_source_remove(global.idSoftwareTriggerTimer);
      global.idSoftwareTriggerTimer = 0;
    }

    signal(SIGINT, pSigintHandlerOld);

    g_main_loop_unref(applicationdata.main_loop);

    guint64 n_completed_buffers;
    guint64 n_failures;
    guint64 n_underruns;
    guint64 n_resent;
    guint64 n_missing;
    arv_stream_get_statistics(
      (ArvStream *)pStream, &n_completed_buffers, &n_failures,
      &n_underruns);
    RCLCPP_INFO(
      global.pNode->get_logger(), "Completed buffers = %Lu",
      (unsigned long long) n_completed_buffers);
    RCLCPP_INFO(
      global.pNode->get_logger(), "Failures          = %Lu", (unsigned long long) n_failures);
    RCLCPP_INFO(
      global.pNode->get_logger(), "Underruns         = %Lu", (unsigned long long) n_underruns);
    arv_gv_stream_get_statistics(pStream, &n_resent, &n_missing);
    RCLCPP_INFO(
      global.pNode->get_logger(), "Resent buffers    = %Lu", (unsigned long long) n_resent);
    RCLCPP_INFO(
      global.pNode->get_logger(), "Missing           = %Lu", (unsigned long long) n_missing);

    arv_device_execute_command(global.pDevice, "AcquisitionStop");

    g_object_unref(pStream);

  } else {
    RCLCPP_ERROR(global.pNode->get_logger(), "No cameras detected.");
  }

  return 0;
} // main()
