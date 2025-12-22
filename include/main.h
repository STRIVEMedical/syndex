// CAN Communication vvv

#include "ODriveFlexCAN.hpp"

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

bool setupCan();

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

void onCanMessage(const CanMsg& msg);

void setup();

// void loop();
