
// Store data in the mailbox in your format. As data comes from the stack or
// your logs, store it here directly. For data that comes from Simian, convert
// it in the ConvertFromSimian function to your format and store it here, and
// data that's requested for Simian should be converted and returned from
// ConvertToSimian.

// Delete this struct, it's just an example type.
struct Pose {
  int x, y;
};

struct Mailbox {
  Pose pose;
};
