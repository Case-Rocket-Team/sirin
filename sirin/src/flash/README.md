Sirin uses a NOR flash to store flight data. The chip is broken up into **sectors** of 4096 bytes each. Data can be written bytewise, but you cannot overwrite already written data; instead you must first erase the entire sector before writing again.

Config is stored in the first two sectors. The second sector has priority; if it is non-0xFF's then that is the config; otherwise the first sector is the config. When the config is updated, the non-active config sector is erased and overwritten.

The rest of the sectors are cyclic flight logs. A pre-determined number of sectors at the end form a bitmap of already written sectors. The flight_id is simply the address (within these sectors) that the flight data starts at.