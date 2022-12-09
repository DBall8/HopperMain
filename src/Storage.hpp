#ifndef STORAGE_HPP
#define STORAGE_HPP

#include "devices.hpp"
#include "app/DoorController.hpp"

const static uint8_t STORAGE_REVISION = 2;

struct  StorageItems
{
    uint8_t revision;
    Hopper::HopperCalibration calibration;
};

extern StorageItems* pStorageItems;

#endif