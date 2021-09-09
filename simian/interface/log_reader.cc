#include "log_reader.h"

LogReader::LogReader(const google::protobuf::Value& extra_data, Mailbox* mailbox)
    : mailbox_(mailbox) {}

bool LogReader::Open(const simian_public::simulator::v2::LogOpenOptions& options,
                     simian_public::simulator::v2::LogOpenOutput* output) {
  return true;
}

bool LogReader::Read(const simian_public::simulator::v2::LogReadOptions& options,
                     simian_public::simulator::v2::LogReadOutput* output) {
  if (options.offset().seconds() == 0 && options.offset().nanos() == 0) {
    std::cout << "Reading initial pose from log into memory." << std::endl;
    return ReadInitialPose(output);
  }
  return true;
}

bool LogReader::ReadInitialPose(simian_public::simulator::v2::LogReadOutput* output) {
  // Implement functionality to read initial pose from the log
  // and save it in the `mailbox_` here...
  output->set_data_remaining(true);
  output->add_seen_channel_names("simian_pose");
  return true;
}

bool LogReader::Close(const simian_public::simulator::v2::LogCloseOptions& options) { return true; }
