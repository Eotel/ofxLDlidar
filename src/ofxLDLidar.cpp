//
// Created by Daiki Miura on 2024/09/27.
//

#include "ofxLDLidar.h"
#include "ofUtils.h"

using namespace ofx::ldlidar;

struct DeviceType
{
    std::string name;
    ::ldlidar::LDType type;
    uint32_t baudrate;
};

DeviceType deviceTypes[] = {
    {"LD14", ::ldlidar::LDType::LD_14, 115200},
    {"LD14P", ::ldlidar::LDType::LD_14P, 230400},
    {"LD06", ::ldlidar::LDType::LD_06, 230400},
    {"LD19", ::ldlidar::LDType::LD_19, 230400},
};

ldlidar::LDType GetLdsType(const std::string& in_str)
{
    for (const auto& device : deviceTypes)
    {
        if (in_str == device.name)
        {
            return device.type;
        }
    }
    return ldlidar::LDType::NO_VER;
}

uint32_t GetLdsSerialPortBaudrateValue(const std::string& in_str)
{
    for (const auto& device : deviceTypes)
    {
        if (in_str == device.name)
        {
            return device.baudrate;
        }
    }
    return 0;
}

uint64_t ofx::ldlidar::GetTimestamp(void)
{
    const std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    const auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    return static_cast<uint64_t>(tmp.count());
}

bool isDeviceLDLidar(ofSerialDeviceInfo& deviceInfo)
{
#if defined(_WIN32) || defined(_WIN64)
        return ofIsStringInString(deviceInfo.getDeviceName(), "Silicon Labs CP210x USB to UART Bridge");
#elif defined(__linux__)
        // TODO: GPIO で Serial 通信している場合を考慮
        return ofIsStringInString(deviceInfo.getDeviceName(), "ttyUSB");
#elif defined(__APPLE__)
    return ofIsStringInString(deviceInfo.getDeviceName(), "tty.SLAB_USBtoUART");
#else
#error "Unsupported platform"
#endif
}

device::GenericDevice::GenericDevice(const std::string& lidarType)
{
    const ::ldlidar::LDType type = GetLdsType(lidarType);
    if (type == ::ldlidar::LDType::NO_VER)
    {
        ofLogWarning("ofxLDLidar") << "lidarType value is not sure: " << lidarType;
        exit(EXIT_FAILURE);
    }

    const uint32_t serialBaudrate = GetLdsSerialPortBaudrateValue(lidarType);
    if (!serialBaudrate)
    {
        ofLogWarning("ofxLDLidar") << "lidarType value is not sure: " << lidarType;
        exit(EXIT_FAILURE);
    }

#if defined(_WIN32) || defined(_WIN64)
    _driver = ::ldlidar::LDLidarDriverWinInterface::Create();
#elif defined(__linux__)
    _driver = ::ldlidar::LDLidarDriverLinuxInterface::Create();
#elif defined(__APPLE__)
    _driver = ::ldlidar::LDLidarDriverUnixInterface::Create();
#else
#error "Unsupported platform"
#endif

    ofLogNotice("ofxLDLidar") << "LDLiDAR SDK Pack Version is " << _driver->GetLidarSdkVersionNumber();

    _driver->RegisterGetTimestampFunctional(std::bind(&GetTimestamp));
    _driver->EnablePointCloudDataFilter(true);

    _baudRate = serialBaudrate;
    _lidarType = type;
}

device::GenericDevice::~GenericDevice()
{
    disconnect();
    if (_driver) ::ldlidar::LDLidarDriverUnixInterface::Destory(_driver);
}

bool device::GenericDevice::connect(const string& serialPath)
{
    _serialPath = serialPath;

    if (_driver->Connect(_lidarType, _serialPath, _baudRate))
    {
        ofLogNotice("ofxLDLidar") << "ldlidar serial connect is success";

        if (_driver->WaitLidarComm(3500))
        {
            ofLogNotice("ofxLDLidar") << "ldlidar communication is normal.";
        }
        else
        {
            ofLogError("ofxLDLidar") << "ldlidar communication is abnormal.";
            _driver->Disconnect();
            return false;
        }

        return true;
    }

    ofLogError("ofxLDLidar") << "ldlidar serial connect is fail";
    return false;
}

bool device::GenericDevice::isConnected() const
{
    return _driver && _driver->Ok();
}

void device::GenericDevice::update()
{
    if (isThreadRunning())
    {
        _isFrameNew = _hasNewFrame.exchange(false);
    }
    else
    {
        _result.back() = scan();
        _result.swap();
        _isFrameNew = true;
    }
}

bool device::GenericDevice::reconnect()
{
    return connect(_serialPath);
}

bool device::GenericDevice::disconnect()
{
    if (isConnected())
    {
        stop();
        return _driver->Disconnect();
    }
    return false;
}

bool device::GenericDevice::start(const bool threaded)
{
    if (isConnected() && _driver->Start())
    {
        if (threaded)
        {
            startThread();
        }
        return true;
    }
    return false;
}

bool device::GenericDevice::stop()
{
    if (isThreadRunning())
    {
        stopThread();
        waitForThread();
    }
    if (isConnected()
        && _driver->Stop()
    )
    {
        return true;
    }
    return false;
}

::ldlidar::Points2D device::GenericDevice::scan()
{
    ::ldlidar::Points2D ret;
    _isScanning = true;
    auto start = std::chrono::steady_clock::now();

    while (_isScanning && std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count() < 1500)
    {
        switch (_driver->GetLaserScanData(ret, 100))
        {
        case ::ldlidar::LidarStatus::NORMAL:
            _driver->GetLidarScanFreq(_scanFrequency);
            _isScanning = false;
            _lastScanTime = std::chrono::steady_clock::now();
            return ret;
        case ::ldlidar::LidarStatus::DATA_TIME_OUT:
            // ofLogWarning("ofxLDLidar") << "Point cloud data publish time out, retrying...";
            break;
        case ::ldlidar::LidarStatus::DATA_WAIT:
            break;
        default:
            break;
        }
    }

    _isScanning = false;
    if (ret.empty())
    {
        ofLogError("ofxLDLidar") << "Failed to get laser scan data within timeout period";
    }
    return ret;
}

void device::GenericDevice::threadedFunction()
{
    while (isThreadRunning())
    {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - _lastScanTime).count() >= 66) // ~15Hz
        {
            if (::ldlidar::Points2D points = scan(); !points.empty())
            {
                _result.back() = points;
                _result.swap();
                _hasNewFrame = true;
            }
        }
        ofSleepMillis(1);
        if (!isConnected())
        {
            stopThread();
        }
    }
}

::ldlidar::Points2D device::GenericDevice::getResult()
{
    if (isThreadRunning())
    {
        return _result.front();
    }
    return scan();
}

void device::GenericDevice::enableFilterAlgorithm(const bool enable) const
{
    _driver->EnablePointCloudDataFilter(enable);
}

void device::GenericDevice::setLaserScanDirection(bool counterclockwise)
{
    _clockWise = !counterclockwise;
}

float device::GenericDevice::getLidarScanFrequency() const
{
    return _scanFrequency;
}

std::vector<ofSerialDeviceInfo> device::GenericDevice::getDeviceList()
{
    ofSerial serial;
    auto ret = serial.getDeviceList();
    ret.erase(remove_if(begin(ret), std::end(ret), [](ofSerialDeviceInfo& info)
    {
        return !isDeviceLDLidar(info);
    }), std::end(ret));
    return ret;
}
