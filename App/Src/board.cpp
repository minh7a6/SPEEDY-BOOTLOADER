#include "timeDelay.h"
#include "board.hpp"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_usart.h"
#include "main.h"
#include "canard_stm32.h"
#include "watchdog.h"

#define FLASH_TIMEOUT_VALUE      (50000U) /* 50 s */


extern "C" {
void usleep(volatile uint32_t microseconds)
{
    timeDelayUs(microseconds);
}

bool flashWait(uint32_t timeWait)
{
    uint64_t target = getCounter() + timeWait;
    while(FLASH->SR & FLASH_SR_BSY && getCounter() < target);
    if(target <= getCounter()) return false;
    if (READ_BIT(FLASH->SR, FLASH_SR_EOP))
    {
        /* Clear FLASH End of Operation pending bit */
        CLEAR_BIT(FLASH->SR, FLASH_SR_EOP);
    }

    if(READ_BIT(FLASH->SR, FLASH_SR_WRPERR)  || 
        READ_BIT(FLASH->SR, FLASH_SR_PGERR))
    {
    /*Save the error code*/
        return false;
    }

    /* There is no error flag set */
    return true;
}
}

using namespace board;
void board::bootApplication()
{
    // Loading and validating the application entry point
    const unsigned stacktop   = *reinterpret_cast<unsigned*>(ROMADDR);
    const unsigned entrypoint = *reinterpret_cast<unsigned*>(ROMADDR + 4);

    // We cordially extend our thanks to David Sidrane and Ben Dyer, whose ideas have somewhat inspired this thing.
    asm volatile("cpsid i");

    // Deinit all peripherals that may have been used
    // It is crucial to disable ALL peripherals, else a spurious interrupt will crash the application
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);

    // Kill the sys tick
    SysTick->CTRL = 0;

    // Update the vector table location
    asm volatile("dsb");
    asm volatile("isb");
    SCB->VTOR = ROMADDR;
    asm volatile("dsb");

    // Let's roll!
    asm volatile("msr msp, %[stacktop]          \n"
                 "bx       %[entrypoint]        \n"
                 :: [stacktop] "r"(stacktop), [entrypoint] "r"(entrypoint):);

    for (;;) { }        // Noreturn
}


/**
 * Emits one byte into the port.
 * @param byte      The byte to emit.
 * @param timeout   The operation will be aborted if the byte could not be emitted in this amount of time.
 * @return          @ref Result.
 */
USARTCommunication::Result USARTCommunication::emit(std::uint8_t byte, std::chrono::microseconds timeout)
{
    uint64_t target = getCounter() + timeout.count();
    while(getCounter() < target && !LL_USART_IsActiveFlag_TXE(usart_));
    if(getCounter() >= target) return Result::Timeout;
    LL_USART_TransmitData8(usart_, byte); 
    return Result::Success;
}

/**
 * Receives one byte from the port.
 * @param out_byte  A reference where to store the received byte.
 * @param timeout   The operation will be aborted if the byte could not be received in this amount of time.
 * @return          @ref Result.
 */
USARTCommunication::Result USARTCommunication::receive(std::uint8_t& out_byte, std::chrono::microseconds timeout)
{
    uint64_t target = getCounter() + timeout.count();
    while(getCounter() < target && !LL_USART_IsActiveFlag_RXNE(usart_));
    if(getCounter() >= target) return Result::Timeout;
    out_byte = LL_USART_ReceiveData8(usart_);
    LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    return Result::Success;
}

/**
 * Returns the time since boot as a monotonic (i.e. steady) clock.
 * The clock must never overflow.
 * This is like @ref kocherga::IPlatform::getMonotonicUptime().
 */
std::chrono::microseconds USARTCommunication::getMonotonicUptime() const
{
    return  std::chrono::microseconds(getCounter());
}

void UAVCANCommunication::sleep(std::chrono::microseconds durations) const
{
    timeDelayUs(durations.count());
}

void UAVCANCommunication::resetWatchdog()
{
    watchDogRefresh();
}

std::uint64_t UAVCANCommunication::getRandomUnsignedInteger(std::uint64_t lower_bound, std::uint64_t upper_bound) const
{
    if (lower_bound < upper_bound)
    {
        const auto rnd = std::uint64_t(std::rand()) * std::uint64_t(std::rand());
        const std::uint64_t out = lower_bound + rnd % (upper_bound - lower_bound);
        return out;
    }
    else
    {
        return lower_bound;
    }
}

std::int16_t UAVCANCommunication::configure(std::uint32_t bitrate,
                                      UAVCANCommunication::CANMode mode,
                                      const UAVCANCommunication::CANAcceptanceFilterConfig& acceptance_filter)
{
    std::int16_t res;
    CanardSTM32CANTimings timings;
    CanardSTM32AcceptanceFilterConfiguration config;
    LL_RCC_ClocksTypeDef refClock;
    LL_RCC_GetSystemClocksFreq(&refClock);
    res = canardSTM32ComputeCANTimings(refClock.PCLK1_Frequency, bitrate, &timings);
    if(res < 0) return res;
    res = canardSTM32Init(&timings, (CanardSTM32IfaceMode)mode);
    if(res < 0) return res;
    config.id = acceptance_filter.id;
    config.mask = acceptance_filter.mask;
    res = canardSTM32ConfigureAcceptanceFilters(&config, 1);
    return res;

}

std::int16_t UAVCANCommunication::send(const ::CanardCANFrame& frame, std::chrono::microseconds timeout)
{
    (void)timeout;
    return canardSTM32Transmit(&frame);
}

std::pair<std::int16_t, ::CanardCANFrame> UAVCANCommunication::receive(std::chrono::microseconds timeout)
{
    (void)timeout;
    std::pair<std::int16_t, ::CanardCANFrame> result;
    result.first = canardSTM32Receive(&result.second);
    return result;
}

bool UAVCANCommunication::shouldExit() const
{
    return (this->shouldExit_) || ((this->blc_.getState() == kocherga::State::ReadyToBoot));
}

bool UAVCANCommunication::tryScheduleReboot()
{
    if (!this->shouldExit_)
    {
        this->shouldExit_ = true;
        return true;
    }
    else
    {
        return false;
    }
}

std::int16_t ROMDriver::beginUpgrade()
{
    return kocherga::ErrOK;
}

std::int16_t ROMDriver::endUpgrade(bool success)
{
    (void)success; // TODO: handling this 
    return kocherga::ErrOK; 
}

std::int16_t ROMDriver::read(std::size_t offset, void* data, std::uint16_t size) const
{
    if(offset + size > this->_size) return -kocherga::ErrInvalidParams;
    std::memcpy(data, (void*)(ROMADDR + offset), size);
    return size;
}

std::int16_t ROMDriver::write(std::size_t offset, const void* data, std::uint16_t size)
{
    bool res;
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
        if (FLASH->CR & FLASH_CR_LOCK) return -kocherga::ErrInvalidState;
    }
    uint32_t tempPointer = 0;
    uint16_t tempData;
    if(offset + size > this->_size) return -kocherga::ErrInvalidParams;
    while(tempPointer < size) {
        // Better to erase page first if offset is beginning of a page
        if(((offset + tempPointer) % 0x800) == 0) {
            res = flashWait(FLASH_TIMEOUT_VALUE);
            if(!res) return -kocherga::ErrROMWriteFailure;
            SET_BIT(FLASH->CR, FLASH_CR_PER);
            WRITE_REG(FLASH->AR, ROMADDR + offset + tempPointer);
            SET_BIT(FLASH->CR, FLASH_CR_STRT);
            res = flashWait(FLASH_TIMEOUT_VALUE);
            CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
            if(!res) return -kocherga::ErrROMWriteFailure;
        }
        this->read(offset + tempPointer, &tempData, 2);
        // Check to make sure the address is safe to write
        if(tempData != 0xFFFF) {
            return -kocherga::ErrROMWriteFailure;
        }
        // save us a write :))
        if(*(__IO uint16_t*)((uint32_t)data + tempPointer) != 0xFFFF) {
            SET_BIT(FLASH->CR, FLASH_CR_PG); // select program
            if(size - tempPointer == 1) {
                tempData =  *(__IO uint16_t*)((uint32_t)data + tempPointer) | 0xFF;
            }
            else {
                tempData =  *(__IO uint16_t*)((uint32_t)data + tempPointer) & 0xFFFF;
            }
            *(__IO uint16_t*)(ROMADDR + offset + tempPointer) = tempData; // write data
            res = flashWait(FLASH_TIMEOUT_VALUE);
            if(!res) return -kocherga::ErrROMWriteFailure;
            CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
        }
        tempPointer += 2;
    }
    FLASH->CR |= FLASH_CR_LOCK;
    return size;
}


std::chrono::microseconds Platform::getMonotonicUptime() const
{
    return std::chrono::microseconds(getCounter());
}