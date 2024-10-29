# Procedure status message

# Status of the procedure
uint8 PROCEDURE_STATUS_OK = 0
uint8 PROCEDURE_STATUS_WARN = 1
uint8 PROCEDURE_STATUS_ERROR = 2
uint8 PROCEDURE_STATUS_NO_RESPONSE = 3

# ======== Variables Field ========

# The procedure name
string proc_name

# The timeout of the procedure
int64 timeout_ns

# The status of the procedure
uint8 status 3

# The message of the procedure
string message

# The timestamp of the last update
builtin_interfaces/Time timestamp
