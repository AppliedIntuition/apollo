#pragma once

#include <google/protobuf/struct.pb.h>
#include <string>

// Simian dependencies
#include "applied/simian/public/proto/v2/io.pb.h"

// Local dependencies
#include "mailbox.h"

class LogReader {
 public:
  explicit LogReader(const google::protobuf::Value& extra_data, Mailbox* mailbox);
  bool Open(const simian_public::simulator::v2::LogOpenOptions& options,
            simian_public::simulator::v2::LogOpenOutput* output);
  bool Read(const simian_public::simulator::v2::LogReadOptions& options,
            simian_public::simulator::v2::LogReadOutput* output);
  bool Close(const simian_public::simulator::v2::LogCloseOptions& options);

 private:
  bool ReadInitialPose(simian_public::simulator::v2::LogReadOutput* output);

  Mailbox* mailbox_;
  simian_public::simulator::v2::LogOpenOptions open_options_;
  std::string bag_path_;
};
