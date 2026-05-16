/**
 * control_signal_info.h
 *
 * Utility functions for validating rv2_interfaces::msg::ControlSignalInfo fields.
 *
 * Call validateControlSignalInfo() before registering a Source with the CSM to
 * catch configuration errors early and get a human-readable error message.
 *
 * Validation rules
 * ────────────────
 *  1. send_freq_hz    must be >= 0.
 *  2. timeout_ns      must be >= 0.
 *  3. disconnect_timeout_ns must be >= 0.
 *  4. If send_freq_hz > 0 and timeout_ns > 0:
 *       send_period_ns (= 1e9 / send_freq_hz) must be strictly less than timeout_ns.
 *       Otherwise the sink transitions to LOW_FREQ immediately and then to TIMEOUT
 *       before the next message is even expected.
 *  5. If disconnect_timeout_ns > 0:
 *       timeout_ns must be > 0 (disconnect requires a finite timeout to trigger).
 *  6. If disconnect_timeout_ns > 0 and timeout_ns > 0:
 *       disconnect_timeout_ns must be strictly greater than timeout_ns.
 *       A channel cannot disconnect before it first times out.
 */

#pragma once

#include <rv2_interfaces/msg/control_signal_info.hpp>
#include <string>
#include <cstdint>

namespace rv2_interfaces
{

/**
 * @brief Result of a ControlSignalInfo validation check.
 */
struct ControlSignalInfoValidation
{
    bool        valid;   ///< true if all rules pass.
    std::string error;   ///< Human-readable description of the first failing rule; empty when valid.
};

/**
 * @brief Validate the fields of a ControlSignalInfo for internal consistency.
 *
 * Checks numeric constraints and cross-field dependencies among
 * send_freq_hz, timeout_ns, and disconnect_timeout_ns.
 *
 * @param info  The ControlSignalInfo to validate.
 * @return      ControlSignalInfoValidation with valid=true on success,
 *              or valid=false and an error string describing the violation.
 */
inline ControlSignalInfoValidation validateControlSignalInfo(
    const rv2_interfaces::msg::ControlSignalInfo& info)
{
    // Rule 1: send_freq_hz must be non-negative.
    if (info.send_freq_hz < 0.0f)
        return {false, "send_freq_hz must be >= 0 (0 disables frequency checking)"};

    // Rule 2: timeout_ns must be non-negative.
    if (info.timeout_ns < 0)
        return {false, "timeout_ns must be >= 0 (0 disables timeout checking)"};

    // Rule 3: disconnect_timeout_ns must be non-negative.
    if (info.disconnect_timeout_ns < 0)
        return {false, "disconnect_timeout_ns must be >= 0 (0 disables auto-disconnect)"};

    // Rule 4: send period must be shorter than timeout.
    if (info.send_freq_hz > 0.0f && info.timeout_ns > 0)
    {
        const int64_t sendPeriodNs =
            static_cast<int64_t>(1e9 / static_cast<double>(info.send_freq_hz));
        if (sendPeriodNs >= info.timeout_ns)
            return {false,
                "send period (1/send_freq_hz = " + std::to_string(sendPeriodNs) +
                " ns) must be less than timeout_ns (" +
                std::to_string(info.timeout_ns) +
                " ns); otherwise the sink reaches TIMEOUT before the next message arrives"};
    }

    // Rule 5: disconnect requires a finite timeout.
    if (info.disconnect_timeout_ns > 0 && info.timeout_ns == 0)
        return {false,
            "disconnect_timeout_ns is set but timeout_ns is 0; "
            "auto-disconnect requires a finite timeout_ns to trigger TIMEOUT first"};

    // Rule 6: disconnect_timeout_ns must be greater than timeout_ns.
    if (info.disconnect_timeout_ns > 0 && info.timeout_ns > 0 &&
        info.disconnect_timeout_ns <= info.timeout_ns)
        return {false,
            "disconnect_timeout_ns (" + std::to_string(info.disconnect_timeout_ns) +
            " ns) must be greater than timeout_ns (" +
            std::to_string(info.timeout_ns) +
            " ns); a channel cannot be disconnected before it times out"};

    return {true, ""};
}

} // namespace rv2_interfaces
