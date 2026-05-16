string target_csm_name                      # Target control signal manager name.
string control_signal_mode                  # Control signal mode, defined in ControlSignalConst.msg with CONTROL_SIGNAL_MODE_XXX.
string control_signal_type                  # Control signal type, defined in ControlSignalConst.msg with CONTROL_SIGNAL_TYPE_XXX.
string channel_name                         # Could be topic name or service name depending on the control_signal_mode.
float32 send_freq_hz                        # Expected publish frequency in Hz. 0 = no frequency check.
                                            #  Sink transitions to LOW_FREQ when message interval > 1/send_freq_hz but < timeout_ns.
int64 timeout_ns                            # Timeout for source, sink and CSM to determine if the connection is still alive.
bool use_keep_alive                         # If true, the target CSM will publish keep-alive message to a topic, and source CSM 
                                            #  will subscribe to the topic to determine if the connection is still alive.
int64 keep_alive_interval_ns                # If use_keep_alive is true, the interval for target CSM to publish keep-alive message.
int8 priority                               # Control signal priority, defined in ControlSignalConst.msg with CONTROL_SIGNAL_PRIORITY_XXX.
int64 disconnect_timeout_ns                 # If > 0 and the Source/Sink has been in TIMEOUT state for longer than this duration,
                                            #  the CSM will automatically remove it from the managed list and release the object.
                                            #  Set to 0 to disable automatic disconnection.
