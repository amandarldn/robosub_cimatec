#include <ros_canbus/Driver.hpp>
#include <ros_canbus/DriverHico.hpp>
#include <ros_canbus/DriverHicoPCI.hpp>
#include <ros_canbus/Driver2Web.hpp>
#include <ros_canbus/DriverEasySYNC.hpp>
#include <ros_canbus/DriverSocket.hpp>
#include <ros_canbus/DriverNetGateway.hpp>
#include <iostream>

#include <stdio.h>
#include <algorithm>
#include <string>
#include <memory>

using namespace canbus;

Driver::~Driver() 
{
}

Driver *canbus::openCanDevice(std::string const& path, DRIVER_TYPE dType)
{
    std::unique_ptr<Driver> driver;
    switch(dType)
    {
        case HICO: 
            driver.reset(new DriverHico());
            break;

        case HICO_PCI:
            driver.reset(new DriverHicoPCI());
            break;

        case SOCKET:
            driver.reset(new DriverSocket());
            break;          

        case CAN2WEB:
            driver.reset(new Driver2Web());
            break;          

        case NET_GATEWAY:
            driver.reset(new DriverNetGateway());
            break;

        case EASY_SYNC:
            driver.reset(new DriverEasySYNC());
            break;

        default:
            return NULL; 
    }

    if (driver->open(path)) {
        std::cout << ("opened CAN device: ") << std::endl;
        std::cout << (path.c_str()) << std::endl; 
        return driver.release();
    }
    else
        std::cout << ("failed to open CAN device: ") << std::endl;
        std::cout << (path.c_str()) << std::endl;

    return NULL;
}

Driver *canbus::openCanDevice(std::string const& path, std::string const& type_upper)
{
    std::string type = type_upper;
    
    std::transform(type_upper.begin(), type_upper.end(), type.begin(), ::tolower);
    
    if(type == std::string("hico"))
    {
        return openCanDevice(path, HICO);
    }

    if(type == std::string("hico_pci"))
    {
        return openCanDevice(path, HICO_PCI);
    }

    if(type == std::string("socket"))
    {
        return openCanDevice(path, SOCKET);
    }

    if (type == std::string("net_gateway")) {
        return openCanDevice(path, NET_GATEWAY);
    }

    if (type == std::string("easy_sync")) {
        return openCanDevice(path, EASY_SYNC);
    }

    return NULL;
}

Interface::Interface(){
}

Interface::~Interface(){
}
