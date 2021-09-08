/**
 * Copyright (c) 2018  Zubax Robotics  <info@zubax.com>
 */

#include "appShared.hpp"
// #include <os.hpp>
#include <kocherga.hpp>

/// Provided by linker; see the linker script
extern std::uint8_t AppSharedStruct[];


namespace app_shared
{
namespace
{

auto makeMarshaller()
{
    // The storage area is defined in the linker script.
    void* const address = static_cast<void*>(&AppSharedStruct[0]);

    // The address is frozen for binary compatibility; checking it
    assert(reinterpret_cast<std::size_t>(address) == 0x2001ff00U);

    return kocherga::makeAppDataExchangeMarshaller<AppShared>(address);
}

}

std::optional<AppShared> readAndInvalidateSharedStruct()
{
    return makeMarshaller().readAndErase();
}

void writeSharedStruct(const AppShared& shared)
{
    makeMarshaller().write(shared);
}

}
