#ifndef __BOARD_HPP
#define __BOARD_HPP

#include "kocherga_can.hpp"
#include "stm32f3xx.h"
#include "bxcan.h"

#if defined(DEBUG_BUILD)
#define ROMADDR         (0x8000000UL + 0xC800UL) // Start Address of Application
#else
#define ROMADDR         (0x8000000UL + 0x9000UL) // Start Address of Application
#endif
namespace board
{
struct ArgumentsFromApplication
{
    bool linger;        ///< Whether to boot immediately or to wait for commands.

    std::uint16_t uavcan_serial_node_id;                    ///< Invalid if unknown.

    std::uint32_t uavcan_can_bitrate_first;             ///< Zeros if unknown.
    std::uint32_t uavcan_can_bitrate_second;
    std::uint8_t                            uavcan_can_protocol_version;    ///< v0 or v1; 0xFF if unknown.
    std::uint8_t                            uavcan_can_node_id;             ///< Invalid if unknown.

    std::uint8_t                  trigger_node_index;       ///< 0 - serial, 1 - CAN, >1 - none.
    std::uint16_t                 file_server_node_id;      ///< Invalid if unknown.
    std::array<std::uint8_t, 256> remote_file_path;         ///< Null-terminated string.
};
static_assert(std::is_trivial_v<ArgumentsFromApplication>);
/**
 * @brief boot to Application
 * 
 */
void bootApplication();

/**
 * @brief resetting using NVIC
 * 
 */
void reset();
class UAVCANCommunication final : public kocherga::can::ICANDriver
{
    void pollTxQueue(const std::chrono::microseconds now)
    {
        if (const auto* const item = tx_queue_.peek())         // Take the top frame from the prioritized queue.
        {
            const bool expired = now > (item->timestamp + kocherga::can::SendTimeout);  // Drop expired frames.
            if (expired || bxCANPush(0, now.count(), now.count() + 100,   // force_classic_can means no DLE, no BRS.
                                    item->extended_can_id,
                                    item->payload_size,
                                    item->payload))
            {
                tx_queue_.pop();    // Enqueued into the HW TX mailbox or expired -- remove from the SW queue.
            }
        }
    }
    /**
     * @brief 
     * 
     * @param bitrate configure for bitrate
     * @param silent if silent or not
     * @param filter setting up the filter for CAN Bus
     * @return std::optional<Mode> 
     */
    auto configure(const Bitrate&                                  bitrate,
                   const bool                                      silent,
                   const kocherga::can::CANAcceptanceFilterConfig& filter) -> std::optional<Mode> override;

    /**
     * @brief 
     * 
     * @param force_classic_can if it needs classic can
     * @param extended_can_id CAN eID
     * @param payload_size size of the payload
     * @param payload pointer to the payload
     * @return true if push is successful
     * @return false if push is unsuccessful
     */
    auto push(const bool          force_classic_can,
              const std::uint32_t extended_can_id,
              const std::uint8_t  payload_size,
              const void* const   payload) -> bool override;

    /**
     * @brief 
     * 
     * @param payload_buffer get the buffer
     * @return std::optional<std::pair<std::uint32_t, std::uint8_t>> 
     */
    auto pop(PayloadBuffer& payload_buffer) -> std::optional<std::pair<std::uint32_t, std::uint8_t>> override;
    kocherga::can::TxQueue<void*(*)(std::size_t), void(*)(void*)> tx_queue_;
    public:
        UAVCANCommunication(): tx_queue_(&malloc, &free) {}
};

class ROMDriver : public kocherga::IROMBackend
{
    /**
     * @return 0 on success, negative on error
     */
    void beginWrite() override;

    /**
     * @brief 
     * 
     * @param offset 
     * @param data 
     * @param size 
     * @return std::optional<std::size_t> 
     */
    auto write(const std::size_t offset, const std::byte* const data, const std::size_t size) -> std::optional<std::size_t> override;
    
    /**
     * @brief 
     * 
     */
    void endWrite() override;

    /**
     * The size cannot exceed 32767 bytes.
     * @return number of bytes read; negative on error
     */
    auto read(const std::size_t offset, std::byte* const out_data, const std::size_t size) const -> std::size_t override;
    public:
        ROMDriver() {}
};
}


#endif