#!/usr/bin/env python3

import time
import math
import smbus2
import spidev


# ============================================================
# CONFIGURATION
# ============================================================

I2C_BUS = 1

# PCF8574 typically uses 0x20-0x27.
# Currently installed: 0x25, 0x26, 0x27.
PCF_SCAN_ADDRESSES = range(0x20, 0x28)

SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 100_000
SPI_MODE = 1

# MAX31865 / RTD settings
THREE_WIRE = True
FILTER_50HZ = True

# PT100  = 100.0
# PT1000 = 1000.0
RNOMINAL = 100.0

# PT100  MAX31865 module typically uses 430 ohm reference
# PT1000 MAX31865 module typically uses 4300 ohm reference
DEFAULT_RREF = 430.0

# Measurement print interval (seconds)
PRINT_INTERVAL = 1.0

# Discard the first N scans after entering continuous mode
WARMUP_SCANS = 3

# Automatically clear latched fault bits.
# Important mainly when sensors are hot-swapped during operation.
AUTO_CLEAR_FAULTS = True

# If True, every detected MAX31865 module is printed,
# even when no sensor is connected to it.
SHOW_FAILED_CHANNELS = True

# To print only healthy channels, set:
# SHOW_FAILED_CHANNELS = False


# ============================================================
# MAX31865 REGISTERS
# ============================================================

REG_CONFIG = 0x00
REG_RTD_MSB = 0x01
REG_RTD_LSB = 0x02
REG_HIGH_FAULT_MSB = 0x03
REG_HIGH_FAULT_LSB = 0x04
REG_LOW_FAULT_MSB = 0x05
REG_LOW_FAULT_LSB = 0x06
REG_FAULT_STATUS = 0x07

CONFIG_BIAS = 0x80
CONFIG_MODE_AUTO = 0x40
CONFIG_1SHOT = 0x20
CONFIG_3WIRE = 0x10
CONFIG_FAULT_CLEAR = 0x02
CONFIG_FILTER_50HZ = 0x01

# Raw RTD ADC code (15-bit) above which the reading is treated as
# saturated / invalid (close to full scale = open or shorted input).
RAW_NEAR_MAX = 32760


# ============================================================
# HELPERS
# ============================================================

def config_base():
    cfg = CONFIG_BIAS

    if THREE_WIRE:
        cfg |= CONFIG_3WIRE

    if FILTER_50HZ:
        cfg |= CONFIG_FILTER_50HZ

    return cfg


def decode_fault(fault):
    if fault == 0:
        return "OK"

    messages = []

    if fault & 0x80:
        messages.append("RTD high threshold")
    if fault & 0x40:
        messages.append("RTD low threshold")
    if fault & 0x20:
        messages.append("REFIN- > 0.85 x VBIAS")
    if fault & 0x10:
        messages.append("REFIN- < 0.85 x VBIAS AND FORCE- open")
    if fault & 0x08:
        messages.append("RTDIN- < 0.85 x VBIAS AND FORCE- open")
    if fault & 0x04:
        messages.append("over/under voltage")

    if not messages:
        messages.append(f"unknown fault 0x{fault:02X}")

    return ", ".join(messages)


def resistance_to_temperature_celsius(rtd_resistance, rnominal=100.0):
    """
    Convert PT100/PT1000 resistance to temperature using
    the Callendar-Van Dusen equation.
    """

    a = 3.9083e-3
    b = -5.775e-7

    discriminant = a * a - 4 * b * (1 - rtd_resistance / rnominal)

    # Analytical branch for t >= 0 C
    if discriminant >= 0:
        temp = (-a + math.sqrt(discriminant)) / (2 * b)
        if temp >= 0:
            return temp

    # Bisection branch for the sub-zero range (approx -200 to 0 C).
    # Uses the full 4-coefficient CVD form including the C term.
    low = -200.0
    high = 0.0

    for _ in range(50):
        mid = (low + high) / 2.0

        r_mid = rnominal * (
            1
            + a * mid
            + b * mid * mid
            - 4.183e-12 * (mid - 100) * mid * mid * mid
        )

        if r_mid < rtd_resistance:
            low = mid
        else:
            high = mid

    return (low + high) / 2.0


# ============================================================
# MULTI PCF8574 CS CONTROL
# ============================================================

class MultiPCF8574CS:
    def __init__(self, i2c_bus, addresses):
        self.bus = i2c_bus
        self.addresses = list(addresses)

    def all_high(self):
        for addr in self.addresses:
            self.bus.write_byte(addr, 0xFF)

    def select(self, expander_addr, pin):
        """
        Drive every output HIGH on every expander, then pull the selected
        pin LOW on the target expander (active-low chip select).
        """

        for addr in self.addresses:
            if addr == expander_addr:
                state = 0xFF & ~(1 << pin)
            else:
                state = 0xFF

            self.bus.write_byte(addr, state)

    def transfer(self, spi, expander_addr, pin, data):
        """
        Perform one SPI transaction:
        all CS HIGH -> selected CS LOW -> SPI transfer -> all CS HIGH.
        """

        self.select(expander_addr, pin)
        time.sleep(0.0005)

        rx = spi.xfer2(data)

        time.sleep(0.0005)
        self.all_high()
        time.sleep(0.0005)

        return rx


# ============================================================
# MAX31865 SYSTEM
# ============================================================

class MAX31865MultiSystem:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs

    def write_register(self, expander_addr, pin, reg, value):
        self.cs.transfer(
            self.spi,
            expander_addr,
            pin,
            [reg | 0x80, value & 0xFF],
        )

    def read_registers(self, expander_addr, pin, reg, count):
        rx = self.cs.transfer(
            self.spi,
            expander_addr,
            pin,
            [reg & 0x7F] + [0x00] * count,
        )
        return rx[1:]

    def read_register(self, expander_addr, pin, reg):
        return self.read_registers(expander_addr, pin, reg, 1)[0]

    def set_fault_thresholds(self, expander_addr, pin):
        """
        Set thresholds to full range so the threshold registers themselves
        cannot cause spurious high/low fault flags.
        """

        self.write_register(expander_addr, pin, REG_HIGH_FAULT_MSB, 0xFF)
        self.write_register(expander_addr, pin, REG_HIGH_FAULT_LSB, 0xFF)
        self.write_register(expander_addr, pin, REG_LOW_FAULT_MSB, 0x00)
        self.write_register(expander_addr, pin, REG_LOW_FAULT_LSB, 0x00)

    def clear_faults(self, expander_addr, pin, keep_continuous=False):
        """
        Clear the latched fault bits.

        With keep_continuous=True the auto-conversion mode is preserved
        (re-asserted in the same write). This matters because a bare
        CONFIG_FAULT_CLEAR write would otherwise drop the device out of
        continuous mode.
        """

        cfg = config_base() | CONFIG_FAULT_CLEAR

        if keep_continuous:
            cfg |= CONFIG_MODE_AUTO

        self.write_register(
            expander_addr,
            pin,
            REG_CONFIG,
            cfg,
        )

    def check_max31865_present(self, expander_addr, pin):
        """
        MAX31865 has no WHOAMI register, so presence is verified by:
        1) writing and reading back the CONFIG register
        2) writing and reading back a marker in the threshold registers

        Note: bit D0 of the High Fault Threshold LSB is don't-care and
        reads back as 0, so it is masked out of the comparison.
        """

        expected_cfg = config_base()

        self.write_register(expander_addr, pin, REG_CONFIG, expected_cfg)
        time.sleep(0.005)

        actual_cfg = self.read_register(expander_addr, pin, REG_CONFIG)

        if actual_cfg != expected_cfg:
            return False, (
                f"CONFIG mismatch expected=0x{expected_cfg:02X}, "
                f"read=0x{actual_cfg:02X}"
            )

        marker_a = 0xA0 | (pin & 0x0F)
        # LSB bit D0 is don't-care in MAX31865 threshold LSB registers,
        # so force it to 0 to keep the round-trip comparison stable.
        marker_b = (0x50 | (pin & 0x0F)) & 0xFE

        self.write_register(expander_addr, pin, REG_HIGH_FAULT_MSB, marker_a)
        self.write_register(expander_addr, pin, REG_HIGH_FAULT_LSB, marker_b)

        read_marker = self.read_registers(
            expander_addr,
            pin,
            REG_HIGH_FAULT_MSB,
            2,
        )

        if read_marker[0] != marker_a or (read_marker[1] & 0xFE) != marker_b:
            return (
                False,
                f"marker mismatch expected=0x{marker_a:02X} 0x{marker_b:02X}, "
                f"read=0x{read_marker[0]:02X} 0x{read_marker[1]:02X}"
            )

        self.set_fault_thresholds(expander_addr, pin)
        self.clear_faults(expander_addr, pin, keep_continuous=False)

        return True, f"CONFIG OK 0x{actual_cfg:02X}, marker OK"

    def start_continuous(self, expander_addr, pin):
        """
        Enable continuous / auto-conversion mode.
        """

        self.set_fault_thresholds(expander_addr, pin)

        self.write_register(
            expander_addr,
            pin,
            REG_CONFIG,
            config_base() | CONFIG_MODE_AUTO | CONFIG_FAULT_CLEAR,
        )

        time.sleep(0.01)

    def stop_continuous(self, expander_addr, pin):
        """
        Disable continuous mode; VBIAS stays on as defined by config_base().
        """

        self.write_register(
            expander_addr,
            pin,
            REG_CONFIG,
            config_base(),
        )

    def read_raw_and_fault(self, expander_addr, pin):
        data = self.read_registers(expander_addr, pin, REG_RTD_MSB, 2)

        msb = data[0]
        lsb = data[1]

        raw15 = ((msb << 8) | lsb) >> 1
        fault_bit = lsb & 0x01

        fault = 0x00
        if fault_bit:
            fault = self.read_register(expander_addr, pin, REG_FAULT_STATUS)

        return raw15, fault

    def read_channel(self, module):
        expander_addr = module["expander_addr"]
        pin = module["pin"]
        rref = module["rref"]

        raw15, fault = self.read_raw_and_fault(expander_addr, pin)

        # On fault: clear it while preserving continuous mode,
        # wait briefly, then re-read.
        if AUTO_CLEAR_FAULTS and fault:
            self.clear_faults(expander_addr, pin, keep_continuous=True)
            time.sleep(0.1)

            raw15, fault = self.read_raw_and_fault(expander_addr, pin)

        if fault:
            return {
                "ok": False,
                "raw": raw15,
                "fault": fault,
                "reason": decode_fault(fault),
                "resistance": None,
                "temperature": None,
            }

        if raw15 == 0:
            return {
                "ok": False,
                "raw": raw15,
                "fault": 0x00,
                "reason": "raw=0 (RTD disconnected?)",
                "resistance": None,
                "temperature": None,
            }

        if raw15 >= RAW_NEAR_MAX:
            return {
                "ok": False,
                "raw": raw15,
                "fault": 0x00,
                "reason": "raw near maximum",
                "resistance": None,
                "temperature": None,
            }

        resistance = raw15 * rref / 32768.0
        temperature = resistance_to_temperature_celsius(resistance, RNOMINAL)

        return {
            "ok": True,
            "raw": raw15,
            "fault": 0x00,
            "reason": "OK",
            "resistance": resistance,
            "temperature": temperature,
        }


# ============================================================
# I2C EXPANDER DETECTION
# ============================================================

def scan_pcf8574(i2c):
    found = []

    print("Scanning for PCF8574 expanders...")

    for addr in PCF_SCAN_ADDRESSES:
        try:
            i2c.read_byte(addr)
            found.append(addr)
            print(f"  expander found: 0x{addr:02X}")
        except OSError:
            pass

    if not found:
        print("  no PCF8574 expander found")

    print()
    return found


# ============================================================
# MAX31865 MODULE DETECTION
# ============================================================

def detect_max31865_modules(system, expander_addresses):
    modules = []
    logical_index = 0

    print("Detecting MAX31865 modules behind each CS pin...")

    for expander_addr in expander_addresses:
        print(f"Expander 0x{expander_addr:02X}:")

        for pin in range(8):
            try:
                ok, msg = system.check_max31865_present(expander_addr, pin)

                if ok:
                    module = {
                        "index": logical_index,
                        "name": f"E0x{expander_addr:02X}-P{pin}",
                        "expander_addr": expander_addr,
                        "pin": pin,
                        "rref": DEFAULT_RREF,
                    }
                    modules.append(module)

                    print(f"  P{pin}: MAX31865 OK      {msg}")
                    logical_index += 1
                else:
                    print(f"  P{pin}: ---             {msg}")

            except Exception as e:
                print(f"  P{pin}: ---             read error: {e}")

    print()

    if modules:
        print("Detected MAX31865 modules:")
        for m in modules:
            print(
                f"  CH{m['index']:02d}: "
                f"{m['name']}  "
                f"expander=0x{m['expander_addr']:02X} "
                f"pin=P{m['pin']} "
                f"RREF={m['rref']}"
            )
    else:
        print("No MAX31865 module detected.")

    print()
    return modules


# ============================================================
# MEASUREMENT OUTPUT
# ============================================================

def print_measurement_row(module, result):
    label = f"CH{module['index']:02d} {module['name']}"

    if result["ok"]:
        print(
            f"{label}: OK      "
            f"{result['temperature']:8.3f} C   "
            f"R={result['resistance']:9.3f} Ohm   "
            f"raw={result['raw']}"
        )
    else:
        print(
            f"{label}: NOT OK  "
            f"{result['reason']}   "
            f"raw={result['raw']}   "
            f"fault=0x{result['fault']:02X}"
        )


# ============================================================
# MAIN
# ============================================================

def main():
    print("MAX31865 multi-board reader")
    print("==========================")
    print(f"I2C bus:       {I2C_BUS}")
    print(f"SPI bus/dev:   {SPI_BUS}.{SPI_DEVICE}")
    print(f"SPI speed:     {SPI_SPEED_HZ}")
    print(f"SPI mode:      {SPI_MODE}")
    print(f"3-wire:        {THREE_WIRE}")
    print(f"50Hz filter:   {FILTER_50HZ}")
    print(f"RNOMINAL:      {RNOMINAL} Ohm")
    print(f"default RREF:  {DEFAULT_RREF} Ohm")
    print()

    with smbus2.SMBus(I2C_BUS) as i2c:
        expander_addresses = scan_pcf8574(i2c)

        if not expander_addresses:
            return

        cs = MultiPCF8574CS(i2c, expander_addresses)

        # Safe default state: drive all CS lines HIGH.
        cs.all_high()

        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.no_cs = True
        spi.mode = SPI_MODE
        spi.max_speed_hz = SPI_SPEED_HZ

        system = MAX31865MultiSystem(spi, cs)

        modules = []

        try:
            modules = detect_max31865_modules(system, expander_addresses)

            if not modules:
                return

            print("Starting continuous mode on detected modules...")
            for module in modules:
                system.start_continuous(
                    module["expander_addr"],
                    module["pin"],
                )

            time.sleep(0.3)

            print("Discarding warmup scans...")
            for _ in range(WARMUP_SCANS):
                for module in modules:
                    system.read_channel(module)
                time.sleep(0.05)

            print()
            print("Measurement started.")
            print()

            while True:
                print("============================================================")
                print(time.strftime("%Y-%m-%d %H:%M:%S"))

                scan_start = time.perf_counter()
                ok_count = 0
                fail_count = 0

                for module in modules:
                    try:
                        result = system.read_channel(module)

                        if result["ok"]:
                            ok_count += 1
                            print_measurement_row(module, result)
                        else:
                            fail_count += 1
                            if SHOW_FAILED_CHANNELS:
                                print_measurement_row(module, result)

                    except Exception as e:
                        fail_count += 1
                        if SHOW_FAILED_CHANNELS:
                            print(
                                f"CH{module['index']:02d} {module['name']}: "
                                f"NOT OK  read error: {e}"
                            )

                scan_time = time.perf_counter() - scan_start

                print(
                    f"scan time: {scan_time * 1000:.3f} ms   "
                    f"OK={ok_count}   NOT_OK={fail_count}"
                )
                print()

                time.sleep(PRINT_INTERVAL)

        finally:
            print()
            print("Stopping continuous mode and driving all CS lines HIGH...")

            for module in modules:
                try:
                    system.stop_continuous(
                        module["expander_addr"],
                        module["pin"],
                    )
                except Exception:
                    pass

            cs.all_high()
            spi.close()


if __name__ == "__main__":
    main()
