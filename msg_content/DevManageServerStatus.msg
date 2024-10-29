# Control signal for device management server.

######## Definitions ########

# Node address behavior.

## The action of node address behavior.
### NONE: No action for node address.
### ADD: Add the node address to list.
### REMOVE: Remove the node address from list.
uint8 NODE_ADDR_ACTION_NONE = 0
uint8 NODE_ADDR_ACTION_ADD = 1
uint8 NODE_ADDR_ACTION_REMOVE = 2

# Device management server behavior.

## The action of device management server.
### NONE: No action for device management server.
### SET_TIMER: Set device management server timer status, including procedure request timer.
### SET_PERIOD: Set device management server timer periods, including procedure request timer.
### KILL_NODE: Kill the node with specific node name.
uint8 SERVER_ACTION_NONE = 0
uint8 SERVER_ACTION_SET_TIMER = 1
uint8 SERVER_ACTION_SET_PERIOD = 2
uint8 SERVER_ACTION_KILL_NODE = 4

## Timer status.
### NONE: Ignore timer status change.
### STOP: Stop timer.
### START: Start timer.
uint8 TIMER_STATUS_NONE = 0
uint8 TIMER_STATUS_STOP = 1
uint8 TIMER_STATUS_START = 2

## Timer period.
### If value <= 0, the set period action will be ignored at specific timer.


######## Variables ########


# Node address behavior.

## Set to NODE_ADDR_ACTION_XXX.
uint8 node_addr_action 0

## The node address to be added or removed.
### If <node_addr_action> set to ADD, the node address will be added to list.
### If <node_addr_action> set to REMOVE, the node addresses which intersect with <node_addr> will be removed from list.
NodeAddr node_addr


# device management server behavior.

## Set to SERVER_ACTION_XXX.
### The device management server will ignored set timer status and period actions if set to NONE.
### If multiple actions need to be configured, use 'or' operator to combine the SERVER_ACTION_XXX.
uint8 server_action 0

## The status of procedure request timer. Set to TIMER_STATUS_XXX.
uint8 proc_status_req_timer_status 0

## The period of procedure request timer in _ns. Server will ignore this if value <= 0.
int64 proc_status_req_timer_period_ns 0

## The name of the node to be killed. Ignored action if empty.
string server_kill_node_name

## The exit code of the killed node.
int8 server_kill_node_exit_code 1

## The delay time for killing node.
int64 server_kill_node_delay_ns 0
