use embedded_io::{Write, ErrorType, ErrorKind};
use w25q::W25Q;
use crate::spi::SpiDev;

pub struct Flash<'a>{
    w25q: W25Q<SpiDev>,
    writer: &'a mut [u8],
}

// TODO: Create a buffer which we can write into synchronously, then set up a task to occasionally
// write it asynchronously.
// impl <'a> ErrorType for Flash <'a> {
//     type Error = ErrorKind;
// }

//Writes to a buffer, returns the amount of bytes written
// impl <'a> Write for Flash<'a> {
    // fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
    //     //Writes the whole buffer if theres room
    //     if self.buffer.len() - self.cursor as usize > buf.len(){
    //         for i in 0..buf.len(){
    //             self.buffer[self.cursor as usize] = buf[i];
    //             self.cursor = self.cursor + 1;
    //         }
    //         Ok(buf.len())
    //     }
    //     //Just waits and tries again when completely full
    //     else if self.buffer.len() - self.cursor as usize == 0{
    //         //Insert a delay here
    //         self.write(buf)
    //     }
    //     //Partial write, then wait and try again
    //     else{
            
    //         todo!()
    //     }
    // }
// }