#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(alloc_error_handler)]
#![feature(generators)]

use can_interfaces::*;
use defmt::debug;
use embassy_executor::Spawner;
use embassy_stm32::can::Can;
use embassy_stm32::time::mhz;
use embedded_alloc::Heap;
use {defmt_rtt as _, panic_probe as _};
mod can_interfaces;
#[global_allocator]
static HEAP: Heap = Heap::empty();

pub const BITTIMINGS: u32 = 0x00050007; // 500kps @ 32Mhz // config.rcc.sys_ck = Some(mhz(64)); config.rcc.pclk1 = Some(mhz(24)); << experimental >>

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys_ck = Some(mhz(64));
    config.rcc.pclk1 = Some(mhz(24));
    let p = embassy_stm32::init(config);
    debug!("Gateway test");
    unsafe {
        use embassy_stm32::pac;
        pac::RCC.apb2enr().modify(|w| w.set_afioen(true));
        pac::AFIO.mapr().modify(|w| w.set_usart3_remap(0b01)); // enable USART3 for PC11/PC10
        pac::AFIO.mapr().modify(|w| w.set_can2_remap(true)); // enable Can2 for PB5/PB6
        pac::AFIO.mapr().modify(|w| w.set_tim2_remap(0b11)); // enable TIM2 for PA15
        pac::AFIO.mapr().modify(|w| w.set_swj_cfg(0b010)); // disables JTAG, enables PA15
    }

    let uart = {
        use embassy_stm32::interrupt;
        use embassy_stm32::usart;
        use embassy_stm32::usart::Uart;
        let mut config = usart::Config::default();
        config.baudrate = 115200;
        let irq = interrupt::take!(USART3);
        Uart::new(
            p.USART3, p.PC11, p.PC10, irq, p.DMA1_CH2, p.DMA1_CH3, config,
        )
    };

    let can1 = Can::new(p.CAN1, p.PA11, p.PA12);
    let can2 = Can::new(p.CAN2, p.PB5, p.PB6);

    defmt::unwrap!(spawner.spawn(activity_led(p.PC12)));
    defmt::unwrap!(spawner.spawn(uart_task(uart)));

    defmt::unwrap!(spawner.spawn(bms_rx_processor()));
    defmt::unwrap!(spawner.spawn(inverter_rx_processor()));

    // always start can 1 first
    defmt::unwrap!(spawner.spawn(bms_task(can1)));
    defmt::unwrap!(spawner.spawn(inverter_task(can2)));

    defmt::unwrap!(spawner.spawn(contactor_task(p.PA15, p.TIM2, 15000)));
}