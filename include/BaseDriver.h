
#pragma once

#ifndef BASE_DRIVER_H
#define BASE_DRIVER_H

#include <functional>
#include <string>
#include <mutex>

namespace Lidar_Low_Detection
{

class BaseDriver{

public:
    BaseDriver();
    // virtual void Init() = 0;
    virtual int Init() = 0;
    virtual void Start() = 0;
    virtual void Stop() = 0;
    virtual void Free() = 0;
    virtual ~BaseDriver();
};
}
#endif