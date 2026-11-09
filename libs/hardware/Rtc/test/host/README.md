# RTC host tests

Run `python3 libs/hardware/Rtc/test/host/run.py` from the SDK with a native C++20
compiler that supports AddressSanitizer and UndefinedBehaviorSanitizer.

The existing RX8130/PMIC and RTC-disabled tests run alongside `RtcClockTest`.
The clock test exercises the production driver with recording I2C devices:

- PCF85063 primary and Silkscreen's DS3231-to-RV-8263 fallback at 0x51;
- recovery from STOP, external-test and 12-hour modes on time-setting;
- crystal/divider flags checked before accepting time or reporting a successful set;
- preserved calibration, RAM, load-capacitance choice and disabled CLKOUT;
- simulated ticking without MCU polling, and retained time after another `begin()`;
- failures at each I2C operation, a rejected restart, an uncleared oscillator flag,
  a successful retry, and unchanged DS3231 reads/sets.

The simulated clock validates register sequencing, not a physical oscillator.
On hardware, sync once, verify advancing seconds/minutes on later reads, then
sleep for at least ten minutes and compare the next reading with a reference.
The driver does not reset the calendar at boot or wake.
