You can trivially turn any `SpiBus` into `SpiHandle` with `SpiBusHandle(spi_bus)`.

I didn't like `embedded-hal`'s API for `SpiDevice`, so I made my own, called `SpiHandle`. Just call `select` on the handle and you will get something that implements `SpiBus` that gauruntees exclusive access of the bus until it is dropped.