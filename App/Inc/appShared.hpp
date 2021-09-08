/**
 * Copyright (c) 2018  Zubax Robotics  <info@zubax.com>
 */

#pragma once

#include <cstdint>
#include <utility>
#include <optional>


namespace app_shared
{
/**
 * This struct is used to exchange data between the bootloader and the application.
 * Its format allows for future extensions.
 */
struct AppShared
{
    std::uint32_t reserved_a = 0;                               ///< Reserved for future use
    std::uint32_t reserved_b = 0;                               ///< Reserved for future use

    /*
     * UAVCAN part
     */
    std::uint32_t can_bus_speed = 0;                            ///< App <-> Bootloader
    std::uint8_t uavcan_node_id = 0;                            ///< App <-> Bootloader
    std::uint8_t uavcan_fw_server_node_id = 0;                  ///< App --> Bootloader

    static constexpr std::uint8_t UAVCANFileNameCapacity = 201;
    char uavcan_file_name[UAVCANFileNameCapacity] = {};         ///< App --> Bootloader

    /*
     * General part
     */
    bool stay_in_bootloader = false;                            ///< App --> Bootloader

    /*
     * More reserved fields
     */
    std::uint64_t reserved_c = 0;                               ///< Reserved for future use
    std::uint64_t reserved_d = 0;                               ///< Reserved for future use
};

static_assert(sizeof(AppShared) <= 240, "AppShared may be larger than the amount of allocated memory");
static_assert(sizeof(bool) == 1, "Please redefine bool as uint8_t in the shared struct (should never happen on ARM)");

/**
 * Reads the bootloader-app shared structure from the shared memory location and returns it by value.
 * The operation will fail if the structure is not written correctly (e.g. if the other part didn't provide any
 * information or if it was using wrong structure layout due to version mismatch, or whatever).
 * If there was no valid structure found, an empty option is returned.
 * Note that the structure will be invalidated after read to prevent deja-vu.
 */
std::optional<AppShared> readAndInvalidateSharedStruct();

/**
 * Writes the bootloader-app shared data structure.
 * This function cannot fail.
 */
void writeSharedStruct(const AppShared& shared);

}
