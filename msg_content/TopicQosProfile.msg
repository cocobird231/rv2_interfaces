# Topic QoS Profile

## Topic QoS type.
### UNKNOWN: Should not be used.
### PUBLISHER: The QoS profile for publisher.
### SUBSCRIPTION: The QoS profile for subscription.
### BOTH: The QoS profile for both publisher and subscription. (Only for service register and request)
uint8 QOS_TYPE_UNKNOWN = 0
uint8 QOS_TYPE_PUBLISHER = 1
uint8 QOS_TYPE_SUBSCRIPTION = 2
uint8 QOS_TYPE_BOTH = 3

# ======== Variables Field ========

## Topic name.
string topic_name

## Topic qos type. Set to QOS_TYPE_XXX.
uint8 qos_type 0

# QoS Profile Setting
QosProfile qos_profile
