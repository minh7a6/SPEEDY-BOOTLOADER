#ifndef __BOARD_HPP
#define __BOARD_HPP

#include "kocherga_uavcan.hpp"
#include "kocherga_ymodem.hpp"
#include "stm32f3xx.h"

#define ROMADDR         (0x8000000UL + 0xC800UL) // Start Address of Application
namespace board
{
void bootApplication();
class UAVCANCommunication final : public kocherga_uavcan::IUAVCANPlatform
{
    kocherga::BootloaderController& blc_;
    bool shouldExit_ = false;
    void resetWatchdog() override;
    void sleep(std::chrono::microseconds duration) const override;
    std::uint64_t getRandomUnsignedInteger(std::uint64_t lower_bound,
                                            std::uint64_t upper_bound) const override;
    std::int16_t configure(std::uint32_t bitrate,
                                   CANMode mode,
                                   const CANAcceptanceFilterConfig& acceptance_filter) override;
    std::int16_t send(const ::CanardCANFrame& frame, std::chrono::microseconds timeout) override;
    std::pair<std::int16_t, ::CanardCANFrame> receive(std::chrono::microseconds timeout) override;
    bool shouldExit() const override;
    bool tryScheduleReboot() override;
    public:
        UAVCANCommunication(kocherga::BootloaderController& blc) : blc_(blc)
        {
            shouldExit_ = false;
        }
};

class USARTCommunication final : public kocherga_ymodem::IYModemPlatform
{
    kocherga::BootloaderController& blc_;
    USART_TypeDef *usart_;
    /**
     * Emits one byte into the port.
     * @param byte      The byte to emit.
     * @param timeout   The operation will be aborted if the byte could not be emitted in this amount of time.
     * @return          @ref Result.
     */
    Result emit(std::uint8_t byte, std::chrono::microseconds timeout) override;

    /**
     * Receives one byte from the port.
     * @param out_byte  A reference where to store the received byte.
     * @param timeout   The operation will be aborted if the byte could not be received in this amount of time.
     * @return          @ref Result.
     */
    Result receive(std::uint8_t& out_byte, std::chrono::microseconds timeout) override;

    /**
     * Returns the time since boot as a monotonic (i.e. steady) clock.
     * The clock must never overflow.
     * This is like @ref kocherga::IPlatform::getMonotonicUptime().
     */
    std::chrono::microseconds getMonotonicUptime() const override;
    public:
        USARTCommunication(kocherga::BootloaderController& blc, std::function<void(void)> usart_init, USART_TypeDef *usart) : blc_(blc), usart_(usart)
        {
            usart_init();
        }
};

class ROMDriver : public kocherga::IROMBackend
{
    const uint32_t _size;
    /**
     * @return 0 on success, negative on error
     */
    std::int16_t beginUpgrade() override;

    /**
     * The size cannot exceed 32767 bytes.
     * @return number of bytes written; negative on error
     */
    std::int16_t write(std::size_t offset, const void* data, std::uint16_t size) override;

    /**
     * @return 0 on success, negative on error
     */
    std::int16_t endUpgrade(bool success) override;

    /**
     * The size cannot exceed 32767 bytes.
     * @return number of bytes read; negative on error
     */
    std::int16_t read(std::size_t offset, void* data, std::uint16_t size) const override;
    public:
        ROMDriver(std::uint32_t size) : _size(size) {}
};

class Platform : public kocherga::IPlatform
{
    std::chrono::microseconds getMonotonicUptime() const override;
    public:
        void jumpToApp();
};
}


#endif