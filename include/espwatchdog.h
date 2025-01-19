/*
 * @file espwatchdog.h
 * @brief Provides auto ESP restart functionality for microcontrollers based on conditions defined in the calling code.
 * 
 * This file provides a watchdog timer class for ESP8266 microcontrollers. It monitors a flag and restarts the ESP if the flag is not set within a specified timeout period.
 * 
 * @note At present, this file will only work for ESP8266 microcontrollers. It will be enhanced for ESP32 in the future.
 */

#ifndef ESPWATCHDOG_H
#define ESPWATCHDOG_H

#define WD_TIMEOUT 300 // in seconds

#ifdef ESP8266
#include <Arduino.h>

/**
 * @class watchDog
 * @brief A class to implement a watchdog timer for ESP8266 microcontrollers.
 * 
 * This class provides methods to set and get the timeout period, and to update the watchdog timer based on a monitoring flag.
 */
class watchDog
{
public:
    /**
     * @brief Constructor to initialize the watchdog timer with a specified timeout.
     * 
     * @param timeout The timeout period in seconds. Default is 600 seconds.
     */
    watchDog(int timeout = WD_TIMEOUT)
    {
        _timeout = timeout * 1000;
    }

    /**
     * @brief Sets the timeout period for the watchdog timer.
     * 
     * @param timeout The timeout period in seconds.
     */
    void setTimeout(int timeout)
    {
        _timeout = timeout * 1000;
    }

    /**
     * @brief Gets the current timeout period of the watchdog timer.
     * 
     * @return The timeout period in seconds.
     */
    int getTimeout()
    {
        return _timeout / 1000;
    }

    /**
     * @brief Updates the watchdog timer based on the monitoring flag.
     * 
     * If the monitoring flag is set, the watchdog timer is reset. If the flag is not set and the timeout period has elapsed, the ESP is restarted.
     * 
     * @param monitor_flag The flag to monitor. If true, the watchdog timer is reset.
     * @param instant_restart If true, the ESP is restarted immediately if the monitoring flag is not set.
     */
    void update(bool monitor_flag, bool instant_restart = false)
    {
        _monitor_flag = monitor_flag;
        if (monitor_flag)
        {
            _last_timestamp = millis();
        }
        else
        {
            if (instant_restart || (millis() - _last_timestamp > _timeout))
            {
                ESP.restart();
            }
        }
    }

private:
    int _timeout;           ///< The timeout period in milliseconds.
    bool _monitor_flag;     ///< The flag to monitor.
    unsigned long _last_timestamp; ///< The last timestamp when the watchdog timer was reset.
};

#endif // ESP8266

#endif // ESPWATCHDOG_H