use core::marker::PhantomData;
use rp2040_hal::pac as rp2040_pac;
use rp2040_hal::pac::DMA;

use rp2040_hal::gpio::{DynPinId, Function, Pin, PullNone};
use rp2040_hal::pio::{
    Buffers, PIOBuilder, PIOExt, PinDir, ShiftDirection, StateMachineIndex, UninitStateMachine, PIO,
};

/// DMA unit.
pub trait DMAExt {
    /// Splits the DMA unit into its individual channels.
    fn split(self) -> Channels;
}

/// DMA channel.
pub struct Channel<CH: ChannelIndex> {
    _phantom: PhantomData<CH>,
}

/// DMA channel identifier.
pub trait ChannelIndex {
    /// Numerical index of the DMA channel (0..11).
    fn id() -> u8;
}

macro_rules! channels {
    (
        $($CHX:ident: ($chX:ident, $x:expr),)+
    ) => {
        impl DMAExt for DMA {
            fn split(self) -> Channels {
                Channels {
                    $(
                        $chX: Channel {
                            _phantom: PhantomData,
                        },
                    )+
                }
            }
        }

        /// Set of DMA channels.
        pub struct Channels {
            $(
                /// DMA channel.
                pub $chX: Channel<$CHX>,
            )+
        }
        $(
            /// DMA channel identifier.
            pub struct $CHX;
            impl ChannelIndex for $CHX {
                fn id() -> u8 {
                    $x
                }
            }
        )+
    }
}

channels! {
    CH0: (ch0, 0),
    CH1: (ch1, 1),
    CH2: (ch2, 2),
    CH3: (ch3, 3),
    CH4: (ch4, 4),
    CH5: (ch5, 5),
    CH6: (ch6, 6),
    CH7: (ch7, 7),
    CH8: (ch8, 8),
    CH9: (ch9, 9),
    CH10:(ch10, 10),
    CH11:(ch11, 11),
}

pub trait ChannelRegs {
    unsafe fn ptr() -> *const rp2040_pac::dma::CH;
    fn regs(&self) -> &rp2040_pac::dma::CH;
}

impl<CH: ChannelIndex> ChannelRegs for Channel<CH> {
    unsafe fn ptr() -> *const rp2040_pac::dma::CH {
        &(*rp2040_pac::DMA::ptr()).ch[CH::id() as usize] as *const _
    }

    fn regs(&self) -> &rp2040_pac::dma::CH {
        unsafe { &*Self::ptr() }
    }
}


#[derive(Clone, Copy, Default)]
#[repr(packed)]
struct AngleInfo {
    pixel_buf_idx: u16,
    addr_buf_idx: u16,
    lines: u16,
}

/// Mapping between GPIO pins and HUB75 pins
pub struct DisplayPins<F: Function> {
    pub r1: Pin<DynPinId, F, PullNone>,
    pub g1: Pin<DynPinId, F, PullNone>,
    pub b1: Pin<DynPinId, F, PullNone>,
    pub r2: Pin<DynPinId, F, PullNone>,
    pub g2: Pin<DynPinId, F, PullNone>,
    pub b2: Pin<DynPinId, F, PullNone>,
    pub addra: Pin<DynPinId, F, PullNone>,
    pub addrb: Pin<DynPinId, F, PullNone>,
    pub addrc: Pin<DynPinId, F, PullNone>,
    pub addrd: Pin<DynPinId, F, PullNone>,
    pub addre: Pin<DynPinId, F, PullNone>,
    pub oe0: Pin<DynPinId, F, PullNone>,
    pub oe1: Pin<DynPinId, F, PullNone>,
    pub oe2: Pin<DynPinId, F, PullNone>,
    pub clk: Pin<DynPinId, F, PullNone>,
    pub lat: Pin<DynPinId, F, PullNone>,
}

/// The HUB75 display driver
pub struct Display<CH0: ChannelIndex, CH1: ChannelIndex>
{
    fb_ch: Channel<CH0>,
    row_ch: Channel<CH1>,
    pixel_buf: &'static [u8],
    addr_buf: &'static [u8],
    angle_buf: &'static [u8],
}

impl<CH0, CH1> Display<CH0, CH1> where
    CH0: ChannelIndex,
    CH1: ChannelIndex,
{
    pub fn new<PE, SM0, SM1>(
        pins: DisplayPins<PE::PinFunction>,
        pio_block: &mut PIO<PE>,
        pio_sms: (UninitStateMachine<(PE, SM0)>, UninitStateMachine<(PE, SM1)>),
        dma_chs: (Channel<CH0>, Channel<CH1>),
        pixel_buf: &'static [u8],
        addr_buf: &'static [u8],
        angle_buf: &'static [u8],
    ) -> Self
        where
            PE: PIOExt,
            SM0: StateMachineIndex,
            SM1: StateMachineIndex,
    {
        // Setup PIO SMs
        let (data_sm, row_sm) = pio_sms;

        // Data SM
        let (data_sm, data_sm_tx) = {
            let program_data = pio_proc::pio_asm!(
                ".side_set 1",
                "out isr, 32    side 0b0",
                ".wrap_target",
                "mov x isr      side 0b0",
                // Wait for the row program to set the ADDR pins
                "pixel:",
                "out pins, 8    side 0b0",
                "jmp x-- pixel  side 0b1", // clock out the pixel
                "irq 4          side 0b1", // tell the row program to set the next row
                "wait 1 irq 5   side 0b0",
                ".wrap",
            );
            let installed = pio_block.install(&program_data.program).unwrap();
            let (mut sm, _, mut tx) = PIOBuilder::from_program(installed)
                .out_pins(pins.r1.id().num, 6)
                .side_set_pin_base(pins.clk.id().num)
                .clock_divisor_fixed_point(2, 0)// 不能为1 否则屏幕乱码
                .out_shift_direction(ShiftDirection::Right)
                .autopull(true)
                .buffers(Buffers::OnlyTx)
                .build(data_sm);
            sm.set_pindirs([
                (pins.r1.id().num, PinDir::Output),
                (pins.g1.id().num, PinDir::Output),
                (pins.b1.id().num, PinDir::Output),
                (pins.r2.id().num, PinDir::Output),
                (pins.g2.id().num, PinDir::Output),
                (pins.b2.id().num, PinDir::Output),
                (pins.clk.id().num, PinDir::Output),
            ]);
            // Configure the width of the screen
            let one_line_loops: u32 = 63;
            tx.write(one_line_loops);
            (sm, tx)
        };

        let (row_sm, row_sm_tx) = {
            let program_data = pio_proc::pio_asm!(
                ".side_set 1",
                ".wrap_target",
                "set pins, 0b111 side 0b0",  // disable
                "pull            side 0b0",
                "wait 1 irq 4    side 0b0",  // wait one line
                "nop             side 0b1 [4]",  // latch first
                "nop             side 0b0 [4]",  //
                "irq 5           side 0b1",  // notify, latch again for this special hub75
                "out pins, 5     side 0b1 [4]",  // write addr
                "out pins, 8     side 0b1",  // write addr and oe to enable
                "out x, 19       side 0b1",  // delay cnt
                "loop1:",       // delay for pwm
                "jmp x--, loop1  side 0b0",  // enable
                ".wrap",
            );
            let installed = pio_block.install(&program_data.program).unwrap();
            let (mut sm, _, tx) = PIOBuilder::from_program(installed)
                .out_pins(pins.addra.id().num, 8)
                .set_pins(pins.oe0.id().num, 3)
                .side_set_pin_base(pins.lat.id().num)
                .clock_divisor_fixed_point(1, 0) // 数值越大越亮
                .out_shift_direction(ShiftDirection::Right)
                .autopull(false)
                .buffers(Buffers::OnlyTx)
                .build(row_sm);
            sm.set_pindirs([
                (pins.addra.id().num, PinDir::Output),
                (pins.addrb.id().num, PinDir::Output),
                (pins.addrc.id().num, PinDir::Output),
                (pins.addrd.id().num, PinDir::Output),
                (pins.addre.id().num, PinDir::Output),
                (pins.oe0.id().num, PinDir::Output),
                (pins.oe1.id().num, PinDir::Output),
                (pins.oe2.id().num, PinDir::Output),
                (pins.lat.id().num, PinDir::Output),
            ]);
            (sm, tx)
        };
        // Setup DMA
        let (fb_ch, row_ch) = dma_chs;

        // Framebuffer channel
        fb_ch.regs().ch_al1_ctrl.write(|w| unsafe {
            w
                // Increase the read addr as we progress through the buffer
                .incr_read()
                .bit(true)
                // Do not increase the write addr because we always want to write to PIO FIFO
                .incr_write()
                .bit(false)
                // Read 32 bits at a time
                .data_size()
                .size_word()
                // Setup PIO FIFO as data request trigger
                .treq_sel()
                .bits(data_sm_tx.dreq_value())
                // Turn off interrupts
                .irq_quiet()
                .bit(true)
                // Chain to the channel selecting the framebuffers
                .chain_to()
                .bits(CH0::id())
                // Enable the channel
                .en()
                .bit(true)
        });
        fb_ch
            .regs()
            .ch_write_addr
            .write(|w| unsafe { w.bits(data_sm_tx.fifo_address() as u32) });

        // Output enable channel
        row_ch.regs().ch_al1_ctrl.write(|w| unsafe {
            w
                // Increase the read addr as we progress through the buffer
                .incr_read()
                .bit(true)
                // Do not increase the write addr because we always want to write to PIO FIFO
                .incr_write()
                .bit(false)
                // Read 32 bits at a time
                .data_size()
                .size_word()
                // Setup PIO FIFO as data request trigger
                .treq_sel()
                .bits(row_sm_tx.dreq_value())
                // Turn off interrupts
                .irq_quiet()
                .bit(true)
                .chain_to()
                .bits(CH1::id())
                // Enable the channel
                .en()
                .bit(true)
        });
        row_ch
            .regs()
            .ch_write_addr
            .write(|w| unsafe { w.bits(row_sm_tx.fifo_address() as u32) });

        data_sm.start();
        row_sm.start();

        Display {
            fb_ch,
            row_ch,
            pixel_buf,
            addr_buf,
            angle_buf,
        }
    }

    pub fn fb_loop_busy(&self) -> bool {
        self.fb_ch
            .regs()
            .ch_ctrl_trig
            .read()
            .busy()
            .bit_is_set() | self.row_ch
            .regs()
            .ch_ctrl_trig
            .read()
            .busy()
            .bit_is_set()
    }

    /// Flips the display buffers
    ///
    /// Has to be called once you have drawn something onto the currently inactive buffer.
    pub fn commit(&mut self) -> bool {
        let busy = self.fb_loop_busy();
        while self.fb_loop_busy() {}
        busy
    }

    pub fn refresh(&mut self, angle: usize) {
        let angle_info_ptr = self.angle_buf.as_ptr() as *const AngleInfo;
        let angle_info = unsafe { &*angle_info_ptr.add(angle) };
        // let angle_info_list: &[AngleInfo] = unsafe { core::slice::from_raw_parts(angle_info_ptr, self.angle_buf.len() / core::mem::size_of::<AngleInfo>()) };
        // let angle_info: &AngleInfo = angle_info_list[angle];
        let pixels_cnt = angle_info.lines as u32 * 64 / 4;
        let pixels_addr = self.pixel_buf.as_ptr() as u32 + angle_info.pixel_buf_idx as u32;
        let addr_cnt = angle_info.lines as u32;
        let addrs_addr = self.addr_buf.as_ptr() as u32 + angle_info.addr_buf_idx as u32;

        self.fb_ch
            .regs()
            .ch_trans_count
            .write(|w| unsafe { w.bits(pixels_cnt) });
        self.fb_ch
            .regs()
            .ch_al3_read_addr_trig
            .write(|w| unsafe { w.bits(pixels_addr) });
        // self.fb_ch.regs().ch_ctrl_trig.write(|w| unsafe {
        //     w.en().bit(true);
        // });

        self.row_ch
            .regs()
            .ch_trans_count
            .write(|w| unsafe { w.bits(addr_cnt) });
        self.row_ch
            .regs()
            .ch_al3_read_addr_trig
            .write(|w| unsafe { w.bits(addrs_addr) });
    }
}
