use embassy_stm32::peripherals::*;

/*
pub struct GpioPins {
    p1:  PA8,
    p2:  PC9,
    p3:  PC8,
    p4:  PC7,
    p5:  PC6,
    p6:  PD15,
    p7:  PD14,
    p8:  PD13,
    p9:  PD12,
    p10: PD11,
    p11: PD10,
    p12: PD9
}
*/

pub struct GpioPins {
    pub p1: PA4,
    pub p2: PC4,
    pub p3: PC5,
    pub p4: PB0,
    pub p5: PB1,
    pub p6: PB2,
    pub p7: PE7,
    pub p8: PE8,
    pub p9: PE9,
    pub p10: PE10,
    pub p11: PD7,
    pub p12: PD6,
    pub p13: PD5,
    pub p14: PD4,
    //pub p15: PD3,
    //pub p16: PD1,
}