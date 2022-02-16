#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
extern crate stm32l1xx_hal as hal;

mod layout;
#[macro_use]
mod dma_scan;

defmt::timestamp! {"{=u64}", {
        use core::sync::atomic::{AtomicUsize, Ordering};
        static COUNT: AtomicUsize = AtomicUsize::new(0);
        let n = COUNT.load(Ordering::Relaxed);
        COUNT.store(n + 1, Ordering::Relaxed);
        n as u64
    }
}

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers = [TIM7])]
mod app {
    use defmt::{debug, error, info, Debug2Format};
    use embedded_hal::{
        blocking::i2c::{Write, WriteRead},
        digital::v2::{InputPin, OutputPin},
    };
    use hal::{
        gpio::{gpioa, gpiob, Floating, Input, OpenDrain, Output, PullUp},
        prelude::*,
        stm32,
    };
    use keyberon::{debounce, key_code::KbHidReport, layout, matrix};

    const HZ: u32 = 250;
    const LCOLS: usize = 7;
    const RCOLS: usize = 8;
    const ROWS: usize = 5;
    const BOUNCES: usize = 5;

    fn left2global(i: u8, j: u8) -> (u8, u8) {
        (ROWS as u8 - i - 1, LCOLS as u8 - j - 1)
    }

    fn right2global(i: u8, j: u8) -> (u8, u8) {
        (ROWS as u8 - i - 1, LCOLS as u8 + (RCOLS as u8 - j - 1))
    }

    const fn gen_gpio<const N: usize>(pins: [i32; N]) -> [u32; N] {
        let mut result: [u32; N] = [0; N];
        let mut p = 0;
        while p < N {
            if pins[p] >= 0 {
                result[p] = 1 << pins[p];
            }
            p += 1;
        }
        result
    }

    // NOTE: doesn't matter that much whether we invert the roles of rows/columns,
    // actually inverting would be a bit more efficient in terms of writes,
    // but would require reading two ports
    static mut IN_GPIOB: [[u32; LCOLS]; 2 * BOUNCES] = [[69; LCOLS]; 2 * BOUNCES];
    static OUT_GPIOA: [u32; LCOLS] = gen_gpio([-1, -1, -1, -1, -1, 8, -1]);
    static OUT_GPIOB: [u32; LCOLS] = gen_gpio([1, 12, 13, 14, 15, -1, 0]);
    static ROW_GPIOS: [u32; ROWS] = gen_gpio([4, 5, 6, 7, 2]);

    type I2cDev = hal::i2c::I2c<
        stm32::I2C2,
        (
            gpiob::PB10<Output<OpenDrain>>,
            gpiob::PB11<Output<OpenDrain>>,
        ),
    >;

    const ADR_RIGHT_HALF: u8 = 0x27;

    fn setup_mcp(adr: u8, iic: &mut I2cDev) -> Result<(), hal::i2c::Error> {
        const IODIRA: u8 = 0x00;
        const IODIRB: u8 = 0x01;
        const IPOLA: u8 = 0x02;
        const IPOLB: u8 = 0x03;
        const GPPUA: u8 = 0x0c;
        const GPPUB: u8 = 0x0d;
        iic.write(adr, &[IODIRA, 0xff])?; // A* are inputs
        iic.write(adr, &[IPOLA, 0xff])?; // A* are inverted
        iic.write(adr, &[GPPUA, 0xff])?; // A* are pulled up
        iic.write(adr, &[IODIRB, 1 << 7])?; // B* are outputs except B7 (ext btn)
        iic.write(adr, &[IPOLB, 1 << 7])?; // B7 is inverted
        iic.write(adr, &[GPPUB, 1 << 7])?; // B7 is pulled up
        Ok(())
    }

    fn scan_mcp(
        adr: u8,
        iic: &mut I2cDev,
    ) -> Result<matrix::PressedKeys<RCOLS, ROWS>, hal::i2c::Error> {
        const GPIOA: u8 = 0x12;
        const GPIOB: u8 = 0x13;
        let mut mat = [[false; RCOLS]; ROWS];
        for row in 0..ROWS {
            // seems like we need to keep the bit high for a pullup input :/
            iic.write(adr, &[GPIOB, !(1 << row) | (1 << 7)])?;
            cortex_m::asm::delay(32);
            let mut bank: [u8; 1] = [0x69];
            iic.write_read(adr, &[GPIOA], &mut bank)?;
            for col in 0..RCOLS {
                // bits are inverted by the chip via IPOL
                mat[row][col] = bank[0] & (1 << col) != 0;
            }
        }
        Ok(matrix::PressedKeys(mat))
    }

    fn btn_evt(row: u8, bnc: &mut debounce::Debouncer<bool>, state: bool) {
        if bnc.update(state) {
            handle_event::spawn(if *bnc.get() {
                layout::Event::Press(row, (LCOLS + RCOLS) as u8)
            } else {
                layout::Event::Release(row, (LCOLS + RCOLS) as u8)
            })
            .unwrap();
        }
    }

    #[shared]
    struct Shared {
        usb_class: keyberon::Class<'static, hal::usb::UsbBusType, ()>,
        usb_dev: usb_device::device::UsbDevice<'static, hal::usb::UsbBusType>,
        layout: layout::Layout<()>,
    }

    #[local]
    struct Local {
        poll_tim: hal::timer::Timer<stm32::TIM6>,
        knob_btn: gpioa::PA3<Input<PullUp>>,
        knob_cnt:
            hal::qei::Qei<stm32::TIM2, (gpioa::PA0<Input<Floating>>, gpioa::PA1<Input<Floating>>)>,
        knob_last: i16,
        ext_btn: gpioa::PA15<Input<PullUp>>,
        i2c: I2cDev,
        i2c_rdy: bool,
        debouncer_l: debounce::Debouncer<matrix::PressedKeys<LCOLS, ROWS>>,
        debouncer_r: debounce::Debouncer<matrix::PressedKeys<RCOLS, ROWS>>,
        debouncer_knob: debounce::Debouncer<bool>,
        debouncer_ext_l: debounce::Debouncer<bool>,
        debouncer_ext_r: debounce::Debouncer<bool>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {
        cx.device.FLASH.acr.write(|w| w.acc64().set_bit());
        cx.device.FLASH.acr.modify(|_, w| w.prften().set_bit());
        cx.device.FLASH.acr.modify(|_, w| w.latency().set_bit());

        let mut rcc = cx.device.RCC.freeze(hal::rcc::Config::pll(
            hal::rcc::PLLSource::HSE(16.mhz()),
            hal::rcc::PLLMul::Mul6,
            hal::rcc::PLLDiv::Div4,
        ));
        // Disable MSI clock that we've just switched away from to save a tiny microscopic bit of power
        unsafe {
            (*stm32::RCC::ptr()).cr.modify(|_, w| w.msion().clear_bit());
        }

        // DWT is used for I2C timeouts
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();

        // Pull the D+ pin down to send a RESET condition to the USB bus (useful for development)
        let mut usb_dp = gpioa.pa12.into_push_pull_output();
        usb_dp.set_low().unwrap();
        cortex_m::asm::delay(24_0000);
        let usb = hal::usb::Peripheral {
            usb: cx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: usb_dp.into_floating_input(),
        };

        static mut USB_BUS: Option<
            usb_device::class_prelude::UsbBusAllocator<hal::usb::UsbBusType>,
        > = None;
        unsafe {
            USB_BUS = Some(stm32_usbd::UsbBus::new(usb));
        }
        let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };
        let usb_class = keyberon::new_class(&usb_bus, ());
        let usb_dev = usb_device::device::UsbDeviceBuilder::new(
            &usb_bus,
            usb_device::device::UsbVidPid(0x16c0, 0x27db), /* V-USB */
        )
        .manufacturer("unrelenting.technology")
        .product("ErgoNICE")
        .serial_number("69")
        .build();

        let knob_btn = gpioa.pa3.into_pull_up_input();
        let ext_btn = gpioa.pa15.into_pull_up_input();
        let _lr0 = gpiob.pb4.into_pull_down_input();
        let _lr1 = gpiob.pb5.into_pull_down_input();
        let _lr2 = gpiob.pb6.into_pull_down_input();
        let _lr3 = gpiob.pb7.into_pull_down_input();
        let _lr4 = gpiob.pb2.into_pull_down_input();
        let _lc0 = gpiob.pb0.into_push_pull_output();
        let _lc1 = gpiob.pb1.into_push_pull_output();
        let _lc2 = gpiob.pb12.into_push_pull_output();
        let _lc3 = gpiob.pb13.into_push_pull_output();
        let _lc4 = gpiob.pb14.into_push_pull_output();
        let _lc5 = gpiob.pb15.into_push_pull_output();
        let _lc6 = gpioa.pa8.into_push_pull_output();

        unsafe {
            let dma1 = &*stm32::DMA1::ptr();
            let gpioa_odr = (*stm32::GPIOA::ptr()).odr.as_ptr();
            let gpiob_idr = (*stm32::GPIOB::ptr()).idr.as_ptr();
            let gpiob_odr = (*stm32::GPIOB::ptr()).odr.as_ptr();
            dma_reset!(dma1, (*stm32::RCC::ptr()) => dma1en);
            // TIM3_CH3 -> DMA1_CH2
            dma_chan! { dma1:
                cndtr2 [LCOLS]
                cmar2 [&OUT_GPIOA as *const _]
                cpar2 [gpioa_odr]
                ccr2 [mem2per nointr]
            };
            // TIM3_UP -> DMA1_CH3
            dma_chan! { dma1:
                cndtr3 [LCOLS * BOUNCES * 2]
                cmar3 [&mut IN_GPIOB as *mut _]
                cpar3 [gpiob_idr]
                ccr3 [per2mem intr]
            };
            // TIM4_CH1 -> DMA1_CH1
            dma_chan! { dma1:
                cndtr1 [LCOLS]
                cmar1 [&OUT_GPIOB as *const _]
                cpar1 [gpiob_odr]
                ccr1 [mem2per nointr]
            };
        }

        let freq = (LCOLS as u32 * BOUNCES as u32 * HZ).hz(); // -> keyberon tick every 4ms

        // For these, the pin doesn't matter, we can actually provide no pin at the pac level
        let mut scan_pwm1 = cx.device.TIM3.pwm(gpioa.pa6, freq, &mut rcc);
        unsafe {
            (*stm32::TIM3::ptr())
                .dier
                .write(|w| w.cc3de().set_bit().ude().set_bit());
        }
        let mut scan_pwm2 = cx.device.TIM4.pwm(gpiob.pb9, freq, &mut rcc);
        unsafe {
            (*stm32::TIM4::ptr())
                .dier
                .write(|w| w.cc1de().set_bit().ude().set_bit());
        }
        scan_pwm1.enable();
        scan_pwm2.enable();

        // TODO: hal timeouts -> i2c hotplug
        let mut i2c = cx.device.I2C2.i2c(
            (
                gpiob.pb10.into_open_drain_output(),
                gpiob.pb11.into_open_drain_output(),
            ),
            100.khz(),
            &mut rcc,
        );

        let i2c_rdy = if let Err(e) = setup_mcp(ADR_RIGHT_HALF, &mut i2c) {
            error!("mcp setup failed: {}", Debug2Format(&e));
            false
        } else {
            true
        };

        let knob_cnt = cx.device.TIM2.qei((gpioa.pa0, gpioa.pa1), &mut rcc);

        let mut poll_tim = cx.device.TIM6.timer(HZ.hz(), &mut rcc);
        poll_tim.listen();

        info!("init done");

        (
            Shared {
                usb_class,
                usb_dev,
                layout: layout::Layout::new(crate::layout::LAYERS),
            },
            Local {
                poll_tim,
                knob_btn,
                knob_cnt,
                knob_last: 0,
                ext_btn,
                i2c,
                i2c_rdy,
                debouncer_l: debounce::Debouncer::new(
                    matrix::PressedKeys::default(),
                    matrix::PressedKeys::default(),
                    BOUNCES as u16,
                ),
                debouncer_r: debounce::Debouncer::new(
                    matrix::PressedKeys::default(),
                    matrix::PressedKeys::default(),
                    BOUNCES as u16,
                ),
                debouncer_knob: debounce::Debouncer::new(false, false, BOUNCES as u16),
                debouncer_ext_l: debounce::Debouncer::new(false, false, BOUNCES as u16),
                debouncer_ext_r: debounce::Debouncer::new(false, false, BOUNCES as u16),
            },
            init::Monotonics(),
        )
    }

    #[task(binds = USB_LP, priority = 4, shared = [usb_dev, usb_class], local = [])]
    fn usb_lp(mut c: usb_lp::Context) {
        use usb_device::class::UsbClass;
        debug!("USB_LP");
        c.shared.usb_dev.lock(|usb_dev| {
            c.shared.usb_class.lock(|usb_class| {
                if usb_dev.poll(&mut [usb_class]) {
                    usb_class.poll();
                }
            })
        });
    }

    #[task(priority = 3, capacity = 8, shared = [layout])]
    fn handle_event(mut cx: handle_event::Context, evt: layout::Event) {
        cx.shared.layout.lock(|layout| layout.event(evt));
    }

    #[task(priority = 3, shared = [usb_dev, usb_class, layout])]
    fn tick_keyberon(mut cx: tick_keyberon::Context) {
        let _tick = cx.shared.layout.lock(|layout| layout.tick());
        if cx.shared.usb_dev.lock(|d| d.state()) != usb_device::device::UsbDeviceState::Configured {
            info!("USB not configured");
            return;
        }
        let report: KbHidReport = cx.shared.layout.lock(|layout| layout.keycodes().collect());
        if !cx
            .shared
            .usb_class
            .lock(|c| c.device_mut().set_keyboard_report(report.clone()))
        {
            return;
        }
        while let Ok(0) = cx.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
    }

    #[task(binds = TIM6, priority = 2, shared = [], local = [poll_tim, knob_btn, knob_cnt, knob_last, ext_btn, i2c, i2c_rdy, debouncer_r, debouncer_knob, debouncer_ext_l, debouncer_ext_r])]
    fn poll_stuff(mut cx: poll_stuff::Context) {
        let i2c = &mut cx.local.i2c;
        let knob_now = cx.local.knob_cnt.count() as i16;
        let knob_row = if knob_now > *cx.local.knob_last { 1 } else { 0 };
        for _ in 0..(knob_now - *cx.local.knob_last).abs() / 2 {
            handle_event::spawn(layout::Event::Press(knob_row, (LCOLS + RCOLS) as u8)).unwrap();
            handle_event::spawn(layout::Event::Release(knob_row, (LCOLS + RCOLS) as u8)).unwrap();
        }
        *cx.local.knob_last = knob_now;
        btn_evt(
            2,
            &mut cx.local.debouncer_knob,
            cx.local.knob_btn.is_low().unwrap(),
        );
        btn_evt(
            3,
            &mut cx.local.debouncer_ext_l,
            cx.local.ext_btn.is_low().unwrap(),
        );

        if !*cx.local.i2c_rdy {
            match setup_mcp(ADR_RIGHT_HALF, i2c) {
                Err(e) => {
                    error!("mcp setup failed: {}", Debug2Format(&e));
                }
                Ok(()) => {
                    *cx.local.i2c_rdy = true;
                    info!("mcp reconnected");
                }
            }
        }

        if *cx.local.i2c_rdy {
            match scan_mcp(ADR_RIGHT_HALF, i2c) {
                Ok(keys) => {
                    for ev in cx.local.debouncer_r.events(keys) {
                        debug!(
                            "R: {:?} -> {:?}",
                            Debug2Format(&ev),
                            Debug2Format(&ev.transform(right2global))
                        );
                        handle_event::spawn(ev.transform(right2global)).unwrap();
                    }
                }
                Err(e) => {
                    error!("mcp poll failed: {}", Debug2Format(&e));
                    *cx.local.i2c_rdy = false;
                }
            }

            // have to do after scan_mcp otherwise it hangs o_0
            btn_evt(4, &mut cx.local.debouncer_ext_r, {
                const GPIOB: u8 = 0x13;
                let mut bank: [u8; 1] = [0x69];
                if i2c.write_read(ADR_RIGHT_HALF, &[GPIOB], &mut bank).is_err() {
                    error!("mcp read ext failed");
                    false
                } else {
                    // inverted by the chip via IPOL
                    bank[0] & (1 << 7) != 0
                }
            });
        }

        cx.local.poll_tim.clear_irq();
    }

    #[task(binds = DMA1_CHANNEL3, priority = 2, local = [debouncer_l])]
    fn scan_dma(cx: scan_dma::Context) {
        let dma1 = unsafe { &*stm32::DMA1::ptr() };
        let is_half = dma1.isr.read().htif3().bit();
        let scans = unsafe {
            &IN_GPIOB[if is_half {
                0..BOUNCES
            } else {
                BOUNCES..2 * BOUNCES
            }]
        };
        for scan in scans {
            let mut mat = [[false; LCOLS]; ROWS];
            for (c, colscan) in scan.iter().enumerate() {
                for (r, rowmask) in ROW_GPIOS.iter().enumerate() {
                    mat[r][c] = (colscan & rowmask) != 0;
                }
            }
            for ev in cx.local.debouncer_l.events(matrix::PressedKeys(mat)) {
                debug!(
                    "L: {:?} -> {:?}",
                    Debug2Format(&ev),
                    Debug2Format(&ev.transform(left2global))
                );
                handle_event::spawn(ev.transform(left2global)).unwrap();
            }
        }
        tick_keyberon::spawn().unwrap();

        dma1.ifcr.write(|w| {
            if is_half {
                w.chtif3().set_bit()
            } else {
                w.ctcif3().set_bit()
            }
        });
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rtic::export::wfi()
        }
    }
}
