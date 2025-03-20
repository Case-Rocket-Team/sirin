use embedded_io::Write;
use w25q::W25Q;
use crate::spi::SpiDev;
pub struct Flash {
    w25q: W25Q<SpiDev>,
    cursor: u32,
    buffer: [u8]
}
// TODO: Create a buffer which we can write into synchronously, then set up a task to occasionally
// write it asynchronously.
//Writes to a buffer, returns the amount of bytes written
impl Write for Flash {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if self.buffer.len() - self.cursor > buf.len(){
            for i in 0..buf.len(){
                self.buffer[self.cursor] = buf[i];
                self.cursor = self.cursor + 1;
            }
            Ok(buf.len())
        }
        else if self.buffer.len() - self.cursor == 0{
            Timer::after_millis(10)
        }
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        todo!()
    }
}