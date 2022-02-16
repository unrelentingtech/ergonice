macro_rules! dma_reset {
    ($dma:expr, $rcc:expr => $dmaen:ident) => {
        $rcc.ahbenr.modify(|_, w| w.$dmaen().set_bit());
        $dma.ccr1.reset();
        $dma.ccr2.reset();
        $dma.ccr3.reset();
    };
}

macro_rules! dma_chan_dir {
    (mem2per) => {
        true
    };
    (per2mem) => {
        false
    };
}

macro_rules! dma_chan_intr {
    (intr) => {
        true
    };
    (nointr) => {
        false
    };
}

macro_rules! dma_chan {
    {
        $dma:ident :
        $ndtr:ident [ $items:expr ]
        $mar:ident [ $memaddr:expr ]
        $par:ident [ $peraddr:expr ]
        $cr:ident [ $dir:ident $intr:ident ]
    } => {
        $dma.$ndtr.write(|w| w.ndt().bits($items as u16));
        $dma.$mar.write(|w| w.ma().bits($memaddr as u32));
        $dma.$par.write(|w| w.pa().bits($peraddr as u32));
        $dma.$cr.write(|w| {
            w.pinc()
                .clear_bit()
                .minc()
                .set_bit()
                .psize()
                .bits(0b10) // 32
                .msize()
                .bits(0b10) // 32
                .circ()
                .set_bit()
                .dir()
                .bit(dma_chan_dir!($dir))
                .htie()
                .bit(dma_chan_intr!($intr))
                .tcie()
                .bit(dma_chan_intr!($intr))
                .en()
                .set_bit()
        });
    };
}
