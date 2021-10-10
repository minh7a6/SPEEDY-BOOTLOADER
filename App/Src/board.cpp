#include "timeDelay.h"
#include "board.hpp"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_usart.h"
#include "main.h"
#include "bxcan.h"

#define FLASH_TIMEOUT_VALUE      (50000000U) /* 50s */


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
    LL_APB2_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN);

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

void board::reset()
{
    NVIC_SystemReset();
}

auto UAVCANCommunication::configure(const Bitrate& bitrate,
                                        const bool silent,
                                        const kocherga::can::CANAcceptanceFilterConfig& filter) -> std::optional<Mode>
{
    std::int16_t res;
    BxCANTimings timings;
    BxCANFilterParams _filter;
    _filter.extended_id = filter.extended_can_id;
    _filter.extended_mask = filter.mask;
    LL_RCC_ClocksTypeDef refClock;
    LL_RCC_GetSystemClocksFreq(&refClock);
    res = bxCANComputeTimings(refClock.PCLK1_Frequency, bitrate.arbitration, &timings);
    if(!res) Error_Handler();
    res = bxCANConfigure(0, timings, silent);
    if(!res) Error_Handler();
    bxCANConfigureFilters(0, &_filter);
    return Mode::Classic;

}

auto UAVCANCommunication::push(const bool          force_classic_can,
                                const std::uint32_t extended_can_id,
                                const std::uint8_t  payload_size,
                                const void* const   payload) -> bool
{
    (void) force_classic_can;
    uint64_t counter = getCounter();
    const bool ok = tx_queue_.push(std::chrono::microseconds(counter), force_classic_can, extended_can_id, payload_size, payload);
    pollTxQueue(std::chrono::microseconds(counter));
    return ok;
}

auto UAVCANCommunication::pop(kocherga::can::ICANDriver::PayloadBuffer& payload_buffer) -> std::optional<std::pair<std::uint32_t, std::uint8_t>>
{
    uint64_t counter = getCounter();
    pollTxQueue(std::chrono::microseconds(counter));
    std::uint32_t extended_can_id; 
    std::size_t size; 
    if(!bxCANPop(0, &extended_can_id, &size, payload_buffer.data())) {
        return {};
    }
    else {
        return std::make_pair(extended_can_id, size);
    }
}


void ROMDriver::beginWrite()
{
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
        if (FLASH->CR & FLASH_CR_LOCK) Error_Handler();
    }
}

void ROMDriver::endWrite()
{
    FLASH->CR |= FLASH_CR_LOCK;
}

auto ROMDriver::read(const std::size_t offset, std::byte* const out_data, const std::size_t size) const -> std::size_t
{
    std::memcpy((void*)out_data, (void*)(ROMADDR + offset), size);
    return size;
}

auto ROMDriver::write(const std::size_t offset, const std::byte* const data, const std::size_t size) -> std::optional<std::size_t>
{
    bool res;
    uint32_t tempPointer = 0;
    uint16_t tempData;
    while(tempPointer < size) {
        // Better to erase page first if offset is beginning of a page
        if(((offset + tempPointer) % 0x800) == 0) {
            res = flashWait(FLASH_TIMEOUT_VALUE);
            if(!res) return {};
            SET_BIT(FLASH->CR, FLASH_CR_PER);
            WRITE_REG(FLASH->AR, ROMADDR + offset + tempPointer);
            SET_BIT(FLASH->CR, FLASH_CR_STRT);
            res = flashWait(FLASH_TIMEOUT_VALUE);
            CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
            if(!res) return {};
        }
        this->read(offset + tempPointer, (std::byte*)&tempData, 2);
        // Check to make sure the address is safe to write
        if(tempData != 0xFFFF) {
            return {};
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
            if(!res) return {};
            CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
        }
        tempPointer += 2;
    }
    return size;
}