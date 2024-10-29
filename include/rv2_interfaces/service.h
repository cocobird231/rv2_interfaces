#pragma once

namespace rv2_interfaces
{

/**
 * ServiceResponseStatus is used to indicate the response status of the service.
 * SRV_RES_SUCCESS: The service registration or request is successful.
 * SRV_RES_IGNORED: The service registration or request is ignored.
 * SRV_RES_WARNING: The service registration or request has done with warning.
 * SRV_RES_ERROR: The service registration or request occurs an error.
 * SRV_RES_UNKNOWN_ERROR: The service registration or request occurs an unknown error.
 */
enum ServiceResponseStatus 
{
    SRV_RES_SUCCESS = 0, 
    SRV_RES_IGNORED, 
    SRV_RES_WARNING, 
    SRV_RES_ERROR, 
    SRV_RES_UNKNOWN_ERROR
};



static inline const char* GetServiceResponseStatusMsg(ServiceResponseStatus status)
{
    switch (status)
    {
    case SRV_RES_SUCCESS:
        return "Success";
    case SRV_RES_IGNORED:
        return "Ignored";
    case SRV_RES_WARNING:
        return "Warning";
    case SRV_RES_ERROR:
        return "Error";
    case SRV_RES_UNKNOWN_ERROR:
        return "Unknown Error";
    default:
        return "Unknown Status";
    }
}

static inline const char* GetServiceResponseStatusMsg(int8_t status)
{
    return GetServiceResponseStatusMsg(static_cast<ServiceResponseStatus>(status));
}

} // namespace rv2_interfaces
