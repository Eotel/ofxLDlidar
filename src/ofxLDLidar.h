//
// Created by Daiki Miura on 2024/09/27.
//
#ifndef OFXLDLIDAR_H
#define OFXLDLIDAR_H

#pragma once

#if defined(_WIN32) || defined(_WIN64)
#include "ldlidar_driver_win.h"
#elif defined(__linux__)
#include "ldlidar_driver_linux.h"
#elif defined(__APPLE__)
#include "ldlidar_driver_unix.h"
#else
#error "Unsupported platform"
#endif

#include "ofMain.h"
#include "ofSerial.h"
#include "ofThread.h"

#include "DoubleBuffer.h"

namespace ldlidar
{
    class LDLidarDriver;
}

namespace ofx::ldlidar
{
    uint64_t GetTimestamp(void);

    namespace device
    {
        class GenericDevice final : public ofThread
        {
        public:
            explicit GenericDevice(const std::string& lidarType);
            virtual ~GenericDevice();
            static std::vector<ofSerialDeviceInfo> getDeviceList();
            bool connect(const std::string& serialPath);
            bool reconnect();
            bool disconnect();
            bool isConnected() const;

            bool start(bool threaded = true);
            bool stop();
            void update();

            bool isFrameNew() const { return _isFrameNew; }

            ::ldlidar::Points2D getResult();

            /*
             *@brief Enables or disables the filter algorithm.
             *
             *@param enable true to enable the filter, false to disable.
             */
            void enableFilterAlgorithm(bool enable) const;

            /**
             * @brief Sets the laser scan direction.
             *
             * @param counterclockwise true for counterclockwise scanning, false for clockwise.
             */
            void setLaserScanDirection(bool counterclockwise);

            /**
             * @brief Gets the LiDAR scan frequency.
             *
             * @return The current scan frequency in Hz.
             */
            float getLidarScanFrequency() const;

        private:
#if defined(_WIN32) || defined(_WIN64)
            ::ldlidar::LDLidarDriverWinInterface* _driver;
#elif defined(__linux__)
            ::ldlidar::LDLidarDriverLinuxInterface* _driver;
#elif defined(__APPLE__)
            ::ldlidar::LDLidarDriverUnixInterface* _driver;
#else
#error "Unsupported platform"
#endif
            DoubleBuffer<::ldlidar::Points2D> _result;
            std::string _serialPath;
            ::ldlidar::LDType _lidarType;
            int _baudRate;
            bool _clockWise{true};
            double _scanFrequency{0};
            std::atomic<bool> _hasNewFrame{false};
            std::atomic<bool> _isFrameNew{false};
            std::atomic<bool> _isScanning{false};
            std::chrono::steady_clock::time_point _lastScanTime;
            void threadedFunction() override;
            ::ldlidar::Points2D scan();
        };
    }
}

using ofxLDLidar = ofx::ldlidar::device::GenericDevice;

#endif // OFXLDLIDAR_H
