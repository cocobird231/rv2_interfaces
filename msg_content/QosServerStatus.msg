# Control signal for qos server.

## Functionalities:
### 1. Control the behavior of qmap. (save, remove record.json file)
### 2. Control the behavior of topic device info map. (save, remove record.json file)
### 3. Send re-register request to topic device via InteractiveNode service.


######## Definitions ########


# Qmap behavior.

## The action of qmap behavior.
### NONE: No action for qmap
### SAVE_FILE: Save current qmap to record.json file.
### REMOVE_FILE: Remove record.json file.
uint8 QMAP_ACTION_NONE = 0
uint8 QMAP_ACTION_SAVE_FILE = 1
uint8 QMAP_ACTION_REMOVE_FILE = 2


# Topic device info map behavior.

## The action of topic device info map behavior.
### NONE: No action for topic device info map
### SAVE_FILE: Save current topic device info map to record.json file.
### REMOVE_FILE: Remove record.json file.
uint8 TOPIC_DEVICE_ACTION_NONE = 0
uint8 TOPIC_DEVICE_ACTION_SAVE_FILE = 1
uint8 TOPIC_DEVICE_ACTION_REMOVE_FILE = 2


# Re-register request behavior.

## The action of re-register request behavior.
### NONE: No action for re-register request.
### UNREGISTERED: Send re-register request to unregistered topic device.
### ALL: Send re-register request to all topic devices.
uint8 RE_REGISTER_ACTION_NONE = 0
uint8 RE_REGISTER_ACTION_UNREGISTERED = 1
uint8 RE_REGISTER_ACTION_ALL = 2


######## Variables ########


# Qmap behavior.

## Set to QMAP_ACTION_XXX.
uint8 qmap_action 0


# Topic device info map behavior.

## Set to TOPIC_DEVICE_ACTION_XXX.
uint8 topic_device_action 0


# Re-register request behavior.

## Set to RE_REGISTER_ACTION_XXX.
uint8 re_register_action 0
