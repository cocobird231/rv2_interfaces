# Control signal modes, defined by the way control signals are transmitted.

string CONTROL_SIGNAL_MODE_TOPIC = "topic"      # Source: publisher, sink: subscriber
string CONTROL_SIGNAL_MODE_SERVICE = "service"  # Source: service client, sink: service server
string CONTROL_SIGNAL_MODE_UNKNOWN = "unknown"  # Error message when control signal mode is not recognized.



# Control signal types, defined by the message type of the topic or service used to send control signals.

string CONTROL_SIGNAL_TYPE_JOY = "joy"          # sensor_msgs::msg::Joy
string CONTROL_SIGNAL_TYPE_TWIST = "twist"      # geometry_msgs::msg::Twist
string CONTROL_SIGNAL_TYPE_STRING = "string"    # std_msgs::msg::String
string CONTROL_SIGNAL_TYPE_UNKNOWN = "unknown"  # Error message when control signal type is not recognized.



# Control signal priority, defined by the priority of the control signal source.
#  Higher priority control signal will firstly be used as fallback 
#  when the current control signal source fails.
# Higher proiority control signal emergency stop signal will override all other 
#  control signals.

int8 CONTROL_SIGNAL_PRIORITY_EMERGENCY_STOP = 100   # All control signal with this priority will be treated as emergency stop signal.
int8 CONTROL_SIGNAL_PRIORITY_HIGH = 80              # E.g. Safety server control signal.
int8 CONTROL_SIGNAL_PRIORITY_MEDIUM = 50            # E.g. Remote control signal.
int8 CONTROL_SIGNAL_PRIORITY_LOW = 20               # E.g. AI control signal.
int8 CONTROL_SIGNAL_PRIORITY_UNKNOWN = -1           # Error message when control signal priority is not recognized



# Control signal state, mirrors the ControlSignalState enum in control_signal_transport.h.
#  Used to store / exchange runtime state values as integer constants.

int8 CONTROL_SIGNAL_STATE_UNKNOWN      = 0   # Initial state; no activity observed yet.
int8 CONTROL_SIGNAL_STATE_ACTIVE       = 1   # Transport operating normally (messages arriving within send_freq_hz).
int8 CONTROL_SIGNAL_STATE_LOW_FREQ     = 2   # Messages arriving but slower than send_freq_hz; above timeout.
int8 CONTROL_SIGNAL_STATE_TIMEOUT      = 3   # No message received within timeout_ns.
int8 CONTROL_SIGNAL_STATE_DISCONNECTED = 4   # Removed by CSM after exceeding disconnect_timeout_ns.