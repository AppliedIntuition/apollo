#pragma once

/*
 * A customer's AV stack gets integrated into Simian by inheriting from the
 * CustomerStackBase class and implementing the necessary methods.
 * This class gets instantiated inside a grpc server that runs inside the
 * customer docker container. The Simian simulation engine, which runs
 * in the Applied docker, calls that grpc server as appropriate during
 * a simulation run. The class can override a number of methods to
 * extend or customize some of the generic behavior.
 */

#include <cstdint>
#include <string>

#include "applied/simian/public/proto/common.pb.h"
#include "applied/simian/public/proto/drawing.pb.h"
#include "applied/simian/public/proto/motion_model.pb.h"
#include "applied/simian/public/proto/perception.pb.h"
#include "applied/simian/public/proto/route.pb.h"
#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/proto/sim_command.pb.h"
#include "applied/simian/public/proto/spatial.pb.h"
#include "applied/simian/public/proto/stack_logs.pb.h"
#include "applied/simian/public/proto/v2/io.pb.h"

namespace simian_public {

class CustomerStackBase {
 public:
  explicit CustomerStackBase(const std::string& name);
  virtual ~CustomerStackBase() = default;

  CustomerStackBase(const CustomerStackBase&) = delete;
  CustomerStackBase& operator=(const CustomerStackBase&) = delete;

  /********************************************************
   * INITIALIZATION
   * These functions are called in the order provided here.
   ********************************************************/

  /**
   * @brief Returns a version string for your customer stack.
   * @return A version identifier which will make it clear what customer
   *         stack a simulation was running with.
   */
  virtual const char* GetStackVersion() { return nullptr; }

  /**
   * @brief Bring up your middleware (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t MiddlewareSetup();

  /**
   * @brief Set up your AV stack (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t StackSetup();

  /**
   * @brief Bring up customer log recording (optional).
   *        Note that if you do not intend to implement recording, you should
   *        start Simian with the sim flag --no_customer_record.
   * @param recording_path path to write logs to. Note that you need to create
   *        this directory first.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t RecordingSetup(const std::string_view recording_path);

  /**
   * @brief Bring up your live visualization (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t VisualizationSetup();

  /**
   * @brief Return the default rate for the given channel. Simian will call
   *        this function once for every channel. For every channel, you need
   *        to provide either a rate or a period.
   * @param channel Channel description for which to return the rate.
   * @return Channel rate in Hertz. If you want to provide the channel
   *         period instead, return 0.
   */
  virtual int32_t GetDefaultRate(const simulator::v2::Channel& channel);

  /**
   * @brief Return the default channel period in nanoseconds. Simian will call
   *        this function once for every channel. For every channel, you need
   *        to provide either a rate or a period.
   * @param channel Channel description for which to return the period.
   * @return Channel period in nanoseconds, i.e. 1e9 is once per second. If you
   *         want to provide the channel rate instead, return 0.
   */
  virtual int64_t GetDefaultPeriodNs(const simulator::v2::Channel& channel);

  /**
   * @brief Bring up any listener for the given channel (optional). Simian will
   *        call this function once for every channel.
   * @param channel Channel description.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelListenSetup(const simulator::v2::Channel& channel);

  /**
   * @brief Initialize any publisher for the given channel (optional). Simian
   *        will call this function once for every channel.
   * @param channel Channel description.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelPublishSetup(const simulator::v2::Channel& channel);

  /**
   * @brief Open a drive log (optional) with the given filename, to the given
   *        slot, listening to the given channel, which should be held whenever
   *        a log_read() call is made.
   * @param options Options for opening the log.
   * @param output Output containing information about the log being opened.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t LogOpen(const simian_public::simulator::v2::LogOpenOptions& options,
                          simian_public::simulator::v2::LogOpenOutput* output);

  /**
   * @brief Initialize anything else in the interface (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t Initialize();

  /****************************************************************
   * CORE LOOP - INPUT TO SIMIAN
   * These functions are called in the order in which the channels
   * are defined in the scenario.
   ****************************************************************/

  /**
   * @brief Provide motion model input to Simian.
   * @param channel_name Name of CONTROLS channel.
   * @param ego_input Motion model input for Simian.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertControlsToSimian(const std::string_view channel_name,
                                          motion_model::Input* ego_input);

  /**
   * @brief Provide stack state to Simian.
   * @param channel_name Name of STACK_STATE Simian channel.
   * @param stack_state Stack state for Simian.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertStackStateToSimian(const std::string_view channel_name,
                                            simulator::v2::StackState* stack_state);

  /****************************************************************
   * CORE LOOP - OUTPUT FROM SIMIAN
   * These functions are called in the order in which the channels
   * are defined in the scenario.
   ****************************************************************/

  /**
   * @brief Convert timestamp to stack format.
   * @param channel_name Name of TIME Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTimeFromSimian(const std::string_view channel_name,
                                        const google::protobuf::Timestamp& time);

  /**
   * @brief Convert ego vehicle pose to stack format. You should prefer output
   *        from the localization sensor for more exact data.
   * @param channel_name Name of POSE Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPoseFromSimian(const std::string_view channel_name,
                                        const simulator::v2::Pose& pose);

  /**
   * @brief Convert localization message to stack format. Prefer output from the
   *        localization sensor for more exact data.
   * @param channel_name Name of LOCALIZATION Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLocalizationFromSimian(const std::string_view channel_name,
                                                const spatial::Pose& pose);

  /**
   * @brief Convert motion feedback message to stack format.
   * @param channel_name Name of MOTION_FEEDBACK Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertMotionFeedbackFromSimian(const std::string_view channel_name,
                                                  const motion_model::Feedback& motion_feedback);

  /**
   * @brief Convert Simian's predicted controls for the ego to stack format.
   * @param channel_name Name of CONTROLS Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPredictedControlFromSimian(const std::string_view channel_name,
                                                    const motion_model::Input& predicted_controls);

  /**
   * @brief Convert ego trigger message to stack format. Note that actor
   *        triggers are reported through the ACTORS channel.
   * @param channel_name Name of MOTION_FEEDBACK Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertEgoTriggersFromSimian(const std::string_view channel_name,
                                               const simulator::v2::Trigger& ego_triggers);

  /**
   * @brief Convert the ego's trip agent command message to stack format.
   * @param channel_name Name of TRIP_AGENT Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTripAgentFromSimian(const std::string_view channel_name,
                                             const common::TripAgentOutput& trip_agent);

  /**
   * @brief Convert the ego vehicle's stack state to stack format.
   * @param channel_name Name of STACK_STATE Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertStackStateFromSimian(const std::string_view channel_name,
                                              const simulator::v2::StackState& stack_state);

  /****************************
   * CORE LOOP - SENSOR OUTPUT
   * These functions are called in the order in which the channels
   * are defined in the scenario.
   ****************************/

  /**
   * @brief Convert the actor sensor output to stack format.
   * @param channel_name Name of ACTORS Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertActorSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::ActorSensor& actor_sensor);

  /**
   * @brief Convert the lane sensor output to stack format.
   * @param channel_name Name of LANE_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLaneSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::LaneSensor& lane_sensor);

  /**
   * @brief Convert the traffic light sensor output to stack format.
   * @param channel_name Name of TRAFFIC_LIGHTS Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTrafficLightSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::TrafficLightSensor& traffic_light_sensor);

  /**
   * @brief Convert the localization sensor output to stack format.
   * @param channel_name Name of LOCALIZATION_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLocalizationSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::LocalizationSensor& localization_sensor);

  /**
   * @brief Convert the planar Lidar sensor output to stack format.
   * @param channel_name Name of PLANAR_LIDAR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPlanarLidarSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::PlanarLidarSensor& planar_lidar_sensor);

  /**
   * @brief Convert the planar occupancy grid sensor output to stack format.
   * @param channel_name Name of PLANAR_OCCUPANCY_GRID Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPlanarOccupancyGridSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::PlanarOccupancyGridSensor& planar_occupancy_grid_sensor);

  /**
   * @brief Convert the occlusion grid sensor output to stack format.
   * @param channel_name Name of OCCLUSION_GRID Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertOcclusionGridSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::OcclusionGridSensor& occlusion_grid_sensor);

  /**
   * @brief Convert the free space sensor output to stack format.
   * @param channel_name Name of FREE_SPACE_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertFreeSpaceSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::FreeSpaceSensor& free_space_sensor);

  /**
   * @brief Convert the traffic light block sensor output to stack format.
   * @param channel_name Name of TRAFFIC_LIGHT_BLOCKS Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTrafficLightBlockSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::TrafficLightBlockSensor& traffic_light_block_sensor);

  /**
   * @brief Convert the localization object sensor output to stack format.
   * @param channel_name Name of LOCALIZATION_OBJECT_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLocalizationObjectSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::LocalizationObjectSensor& localization_object_sensor);

  /**
   * @brief Convert the traffic sign sensor output to stack format.
   * @param channel_name Name of TRAFFIC_SIGN_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTrafficSignSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::TrafficSignSensor& traffic_sign_sensor);

  /**
   * @brief Convert the IMU sensor output to stack format.
   * @param channel_name Name of IMU_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertImuSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::IMUSensor& imu_sensor);

  /**
   * @brief Convert the wheel speed sensor output to stack format.
   * @param channel_name Name of WHEEL_SPEED_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertWheelSpeedSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::WheelSpeedSensor& wheel_speed_sensor);

  /**
   * @brief Convert the map sensor output to stack format.
   * @param channel_name Name of MAP_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertMapSensorFromSimian(
      const std::string_view channel_name,
      const perception::PerceptionChannel::MapSensor& map_sensor);

  /**
   * @brief Convert the Lidar output to stack format.
   * @param channel_name Name of LIDAR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLidarSensorFromSimian(
      const std::string_view channel_name,
      const sensor_model::SensorOutput::LidarCloud& lidar_cloud);

  /**
   * @brief Convert the radar output to stack format.
   * @param channel_name Name of RADAR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertRadarSensorFromSimian(
      const std::string_view channel_name,
      const sensor_model::SensorOutput::RadarTrack& radar_track);

  /**
   * @brief Convert the camera output to stack format.
   * @param channel_name Name of CAMERA Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertCameraSensorFromSimian(
      const std::string_view channel_name,
      const sensor_model::SensorOutput::CameraImage& camera_image);

  /**
   * @brief Convert the ultrasound range sensor to stack format.
   * @param channel_name Name of ULTRASOUND Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertUltrasoundSensorFromSimian(
      const std::string_view channel_name,
      const sensor_model::SensorOutput::Range& ultrasound_range);

  /***********************************
   * CORE LOOP - ADDITIONAL FUNCTIONS
   ***********************************/

  /**
   * @brief Publish currently held messages for the given channel.
   * @param channel Channel information for which to publish messages.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelPublishSend(const simulator::v2::Channel& channel);

  /************************
   * LOG-READING FUNCTIONS
   ************************/

  /**
   * @brief Read up until the provided offset into the bag, optionally returning
   *        early. Any data seen meant for the stack should be sent directly,
   *        preferably after updating the stack's time. This is equivalent to
   *        updating the TIME channel, calling publish_send() on it and the
   *        channel received from the log, without the Simian version of those
   *        channels.
   * @param options Options for reading from the log.
   * @param output Log data.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t LogRead(const simulator::v2::LogReadOptions& options,
                          simulator::v2::LogReadOutput* output);

  /**
   * @brief When a message in a log needs to be patched, after the relevant
   *        log_read() and convert_to_simian() calls, log__patch() will be
   *        called with the changes necessary for the channel's type.
   * @param options Log patching options.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t LogPatch(const simulator::v2::PatchOptions& patch_options);

  /***************
   * FINALIZATION
   ***************/

  /**
   * @brief Process the simulation summary (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t SimulationSummary(const simian_public::common::SimulationSummary& summary);

  /**
   * @brief Finalize anything in the interface not covered by the other
   * functions (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t Finalize();

  /**
   * @brief Close and clean up the log in the given slot (optional).
   * @param options Options for closing the log.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t LogClose(const simulator::v2::LogCloseOptions& options);

  /**
   * @brief Clean up any publishers created for the given channel (optional).
   *        Simian will call this function once for every channel.
   * @param channel Channel description.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelPublishTeardown(const simulator::v2::Channel& channel);

  /**
   * @brief Clean up any listener for the given channel (optional). Simian will
   *        call this function once for every channel.
   * @param channel Channel description.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelListenTeardown(const simulator::v2::Channel& channel);

  /**
   * @brief Clean up live visualization (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t VisualizationTeardown();

  /**
   * @brief Clean up customer log recording (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t RecordingTeardown();

  /**
   * @brief Clean up your AV stack (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t StackTeardown();

  /**
   * @brief Clean up your middleware (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t MiddlewareTeardown();

  /***************************************************
   * HELPER FUNCTIONS TO BE CALLED FROM THE INTERFACE
   * Do not override, simply call these functions.
   ***************************************************/

  /**
   * @brief Send a stack log line to Simian.
   * @param log_line Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendStackLog(const simian_public::stack_logs::StackLogLine& log_line) const;

  /**
   * @brief Send a drawing to Simian.
   * @param drawing Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendDrawing(const simian_public::drawing::Drawing& drawing) const;

  /**
   * @brief Send a data point to Simian.
   * @param data_point Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendDataPoint(const simian_public::common::DataPoint& data_point) const;

  /**
   * @brief Send custom data point metadata to Simian.
   * @param metadata Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendCustomDataPointMetadata(
      const simian_public::common::CustomDataPointMetadata& metadata) const;

  /**
   * @brief Send a simulation command to Simian.
   * @param sim_command Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendSimCommand(const simian_public::sim_command::SimCommand& sim_command) const;

  /**
   * @brief Send an observer event to Simian.
   * @param event Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendObserverEvent(const simian_public::common::ObserverEvent& event) const;

  /**
   * @brief Send a message to Simian.
   * @param data_point Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendMessage(const simian_public::common::Message& message) const;

  /***********************************************
   * INTERNAL FUNCTIONS - DO NOT CALL OR OVERRIDE
   ***********************************************/
  void SetStartupOptionsInternal(const simulator::v2::InterfaceStartupOptions& startup_options);

  constexpr static int32_t kProtoSerializationError = -1000;

 protected:
  std::string ego_name_;
  simulator::v2::InterfaceStartupOptions startup_options_;
};

}  // namespace simian_public
