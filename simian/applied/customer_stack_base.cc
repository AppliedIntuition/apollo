#include "applied/customer_stack_base.h"

#include <functional>

#include "applied/stack_interface_c_v2.h"

namespace simian_public {

CustomerStackBase::CustomerStackBase(const std::string& ego_name) : ego_name_(ego_name) {}

int32_t CustomerStackBase::MiddlewareSetup() { return 0; }

int32_t CustomerStackBase::StackSetup() { return 0; }

int32_t CustomerStackBase::RecordingSetup(const std::string_view recording_path) { return 0; }

int32_t CustomerStackBase::VisualizationSetup() { return 0; }

int32_t CustomerStackBase::GetDefaultRate(const simian_public::simulator::v2::Channel& channel) {
  return 0;
}

int64_t CustomerStackBase::GetDefaultPeriodNs(
    const simian_public::simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::ChannelListenSetup(
    const simian_public::simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::ChannelPublishSetup(
    const simian_public::simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::LogOpen(const simian_public::simulator::v2::LogOpenOptions& opts,
                                   simian_public::simulator::v2::LogOpenOutput* output) {
  return 0;
}

int32_t CustomerStackBase::Initialize() { return 0; }

void CustomerStackBase::SetStartupOptionsInternal(
    const simian_public::simulator::v2::InterfaceStartupOptions& startup_options) {
  startup_options_ = startup_options;
}

int32_t CustomerStackBase::ConvertControlsToSimian(const std::string_view channel_name,
                                                   simian_public::motion_model::Input* ego_input) {
  return 0;
}

int32_t CustomerStackBase::ConvertStackStateToSimian(
    const std::string_view channel_name, simian_public::simulator::v2::StackState* stack_state) {
  stack_state->set_stack_state(simian_public::sim_data::SimulatorInput::NOT_READY);
  return 0;
}

int32_t CustomerStackBase::ConvertTimeFromSimian(const std::string_view channel_name,
                                                 const google::protobuf::Timestamp& time) {
  return 0;
}

int32_t CustomerStackBase::ConvertPoseFromSimian(const std::string_view channel_name,
                                                 const simulator::v2::Pose& pose) {
  return 0;
}

int32_t CustomerStackBase::ConvertLocalizationFromSimian(const std::string_view channel_name,
                                                         const spatial::Pose& pose) {
  return 0;
}

int32_t CustomerStackBase::ConvertMotionFeedbackFromSimian(
    const std::string_view channel_name, const motion_model::Feedback& motion_feedback) {
  return 0;
}

int32_t CustomerStackBase::ConvertPredictedControlFromSimian(
    const std::string_view channel_name, const motion_model::Input& predicted_controls) {
  return 0;
}

int32_t CustomerStackBase::ConvertEgoTriggersFromSimian(
    const std::string_view channel_name, const simulator::v2::Trigger& ego_triggers) {
  return 0;
}

int32_t CustomerStackBase::ConvertTripAgentFromSimian(const std::string_view channel_name,
                                                      const common::TripAgentOutput& trip_agent) {
  return 0;
}

int32_t CustomerStackBase::ConvertStackStateFromSimian(
    const std::string_view channel_name, const simulator::v2::StackState& stack_state) {
  return 0;
}

int32_t CustomerStackBase::ConvertActorSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::ActorSensor& actor_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertLaneSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::LaneSensor& lane_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertTrafficLightSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::TrafficLightSensor& traffic_light_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertLocalizationSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::LocalizationSensor& localization_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertPlanarLidarSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::PlanarLidarSensor& planar_lidar_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertPlanarOccupancyGridSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::PlanarOccupancyGridSensor& planar_occupancy_grid_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertOcclusionGridSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::OcclusionGridSensor& occlusion_grid_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertFreeSpaceSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::FreeSpaceSensor& free_space_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertTrafficLightBlockSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::TrafficLightBlockSensor& traffic_light_block_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertLocalizationObjectSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::LocalizationObjectSensor& localization_object_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertTrafficSignSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::TrafficSignSensor& traffic_sign_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertImuSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::IMUSensor& imu_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertWheelSpeedSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::WheelSpeedSensor& wheel_speed_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertMapSensorFromSimian(
    const std::string_view channel_name,
    const perception::PerceptionChannel::MapSensor& map_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertLidarSensorFromSimian(
    const std::string_view channel_name,
    const sensor_model::SensorOutput::LidarCloud& lidar_cloud) {
  return 0;
}

int32_t CustomerStackBase::ConvertRadarSensorFromSimian(
    const std::string_view channel_name,
    const sensor_model::SensorOutput::RadarTrack& radar_track) {
  return 0;
}

int32_t CustomerStackBase::ConvertCameraSensorFromSimian(
    const std::string_view channel_name,
    const sensor_model::SensorOutput::CameraImage& camera_image) {
  return 0;
}

int32_t CustomerStackBase::ConvertUltrasoundSensorFromSimian(
    const std::string_view channel_name,
    const sensor_model::SensorOutput::Range& ultrasound_range) {
  return 0;
}

int32_t CustomerStackBase::ChannelPublishSend(const simulator::v2::Channel& channel) { return 0; }

int32_t CustomerStackBase::LogRead(const simulator::v2::LogReadOptions& options,
                                   simulator::v2::LogReadOutput* output) {
  return 0;
}

int32_t CustomerStackBase::LogPatch(const simulator::v2::PatchOptions& patch_options) { return 0; }

int32_t CustomerStackBase::SimulationSummary(
    const simian_public::common::SimulationSummary& summary) {
  return 0;
}

int32_t CustomerStackBase::Finalize() { return 0; }

int32_t CustomerStackBase::LogClose(const simian_public::simulator::v2::LogCloseOptions& options) {
  return 0;
}

int32_t CustomerStackBase::ChannelPublishTeardown(const simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::ChannelListenTeardown(const simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::VisualizationTeardown() { return 0; }

int32_t CustomerStackBase::RecordingTeardown() { return 0; }

int32_t CustomerStackBase::StackTeardown() { return 0; }

int32_t CustomerStackBase::MiddlewareTeardown() { return 0; }

namespace {

template <typename Proto>
int32_t SendToSimian(const CustomerStackBase* self, const Proto& proto,
                     const std::function<int32_t(const void* self, const uint8_t* input,
                                                 uint64_t input_size)>& send_func) {
  std::vector<uint8_t> buffer(proto.ByteSize());
  if (!proto.SerializeToArray(buffer.data(), buffer.size())) {
    return CustomerStackBase::kProtoSerializationError;
  }
  return send_func(self, buffer.data(), buffer.size());
}

}  // namespace

int32_t CustomerStackBase::SendStackLog(
    const simian_public::stack_logs::StackLogLine& log_line) const {
  return SendToSimian(this, log_line, &customer_interface_v2__send_stack_log);
}

int32_t CustomerStackBase::SendDrawing(const simian_public::drawing::Drawing& drawing) const {
  return SendToSimian(this, drawing, &customer_interface_v2__send_drawing);
}

int32_t CustomerStackBase::SendDataPoint(const simian_public::common::DataPoint& data_point) const {
  return SendToSimian(this, data_point, &customer_interface_v2__send_data_point);
}

int32_t CustomerStackBase::SendCustomDataPointMetadata(
    const simian_public::common::CustomDataPointMetadata& metadata) const {
  return SendToSimian(this, metadata, &customer_interface_v2__send_custom_data_point_metadata);
}

int32_t CustomerStackBase::SendSimCommand(
    const simian_public::sim_command::SimCommand& sim_command) const {
  return SendToSimian(this, sim_command, &customer_interface_v2__send_sim_command);
}

int32_t CustomerStackBase::SendObserverEvent(
    const simian_public::common::ObserverEvent& event) const {
  return SendToSimian(this, event, &customer_interface_v2__send_observer_event);
}

int32_t CustomerStackBase::SendMessage(const simian_public::common::Message& message) const {
  return SendToSimian(this, message, &customer_interface_v2__send_message);
}

}  // namespace simian_public
