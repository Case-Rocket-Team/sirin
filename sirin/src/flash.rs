use embedded_io::{Write, ErrorType, ErrorKind};
use w25q::{W25Q};
use crate::spi::SpiDev;

pub struct Flash<'a>{
    w25q: W25Q<SpiDev>,
    writer: &'a mut [u8], //The buffer
    writer_cursor: u32, //Where in the buffer you are
}

// TODO: Create a writer which we can write into synchronously, then set up a task to occasionally
// write it asynchronously.
impl <'a> ErrorType for Flash <'a> {
    type Error = ErrorKind;
}

//Writes to a buffer, returns the amount of bytes written
impl <'a> Write for Flash<'a> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        //Writes the whole writer if theres room
        if self.writer.len() - self.writer_cursor as usize > buf.len(){
            for i in 0..buf.len(){
                self.writer[self.writer_cursor as usize] = buf[i];
                self.writer_cursor = self.writer_cursor + 1;
            }
            Ok(buf.len())
        }
        //Just waits and tries again when completely full
        else if self.writer.len() - self.writer_cursor as usize == 0{
            //Need to put a delay here
            self.write(buf)
        }
        //Partial write, then wait and try again
        else{
            let mut i: usize = 0;
            while i < buf.len() && self.writer.len() - self.writer_cursor as usize != 0 {
                self.writer[self.writer_cursor as usize] = buf[i];
                self.writer_cursor = self.writer_cursor + 1;
                i = i + 1;
            }
            self.write(&buf[i..buf.len()])
        }
    }
    //I'm pretty sure this doesn't need to do anything because all data writing and buffering is handled in the buffer 
    fn flush(&mut self) -> Result<(), Self::Error>{
        Ok(())
    }
}