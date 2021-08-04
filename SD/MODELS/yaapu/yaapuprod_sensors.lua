----------------------------------------
-- custom sensors configuration file
----------------------------------------
local sensors = {
  -- Sensor 1
[1]=  {
    "Celm",   -- label
    "Celm",     -- OpenTX sensor name
    2,          -- precision: number of decimals 0,1,2
    "Vmin",         -- label for unit of measure
    1,          -- multiplier if < 1 than divides
    "-",        -- "+" track max values, "-" track min values with
    1,          -- font size 1=small, 2=big
    3.65,        -- warning level (nil is do not use feature)
    3.30,        -- critical level (nil is do not use feature)
  },

  -- Sensor 2
[2]=  {
    "Celd",   -- label
    "Celd",     -- OpenTX sensor name
    2,          -- precision: number of decimals 0,1,2
    "Vdelta",         -- label for unit of measure
    1,          -- multiplier if < 1 than divides
    "+",        -- "+" track max values, "-" track min values with
    1,          -- font size 1=small, 2=big
    0.2,        -- warning level (nil is do not use feature)
    0.4,        -- critical level (nil is do not use feature)
  },

  -- Sensor 3
[3]=  {
    "Cel1",   -- label
    "Cel1",     -- OpenTX sensor name
    2,          -- precision: number of decimals 0,1,2
    "V1",         -- label for unit of measure
    1,          -- multiplier if < 1 than divides
    "-",        -- "+" track max values, "-" track min values with
    1,          -- font size 1=small, 2=big
    3.65,        -- warning level (nil is do not use feature)
    3.30,        -- critical level (nil is do not use feature)
  },

  -- Sensor 4
[4]=  {
    "Cel2",   -- label
    "Cel2",     -- OpenTX sensor name
    2,          -- precision: number of decimals 0,1,2
    "V2",         -- label for unit of measure
    1,          -- multiplier if < 1 than divides
    "-",        -- "+" track max values, "-" track min values with
    1,          -- font size 1=small, 2=big
    3.65,        -- warning level (nil is do not use feature)
    3.30,        -- critical level (nil is do not use feature)
  },

  -- Sensor 5
[5]=  {
    "Cel3",   -- label
    "Cel3",     -- OpenTX sensor name
    2,          -- precision: number of decimals 0,1,2
    "V3",         -- label for unit of measure
    1,          -- multiplier if < 1 than divides
    "-",        -- "+" track max values, "-" track min values with
    1,          -- font size 1=small, 2=big
    3.65,        -- warning level (nil is do not use feature)
    3.30,        -- critical level (nil is do not use feature)
  },

  -- Sensor 6
[6]=  {
    "Cel4",   -- label
    "Cel4",     -- OpenTX sensor name
    2,          -- precision: number of decimals 0,1,2
    "V4",         -- label for unit of measure
    1,          -- multiplier if < 1 than divides
    "-",        -- "+" track max values, "-" track min values with
    1,          -- font size 1=small, 2=big
    3.65,        -- warning level (nil is do not use feature)
    3.30,        -- critical level (nil is do not use feature)
  },
}
------------------------------------------------------
-- the script can optionally look up values here
-- for each sensor and display the corresponding text instead
-- as an example to associate a lookup table to sensor 3 declare it like
--
--local lookups = {
-- [3] = {
--     [-10] = "ERR",
--     [0] = "OK",
--     [10] = "CRIT",
--   }
-- }
-- this would display the sensor value except when the value corresponds to one
-- of entered above
-- 
local lookups = {
}

collectgarbage()

return {
  sensors=sensors,lookups=lookups
}
