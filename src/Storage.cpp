#include "Storage.hpp"

static StorageItems storageItems =
{
    .revision = 0,
    .calibration = {.adcMin = 0, .adcMax = 0, .angleClosed = 0, .angleOpen = 0}
};
StorageItems* pStorageItems = &storageItems;