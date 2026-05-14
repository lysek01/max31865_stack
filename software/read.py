#!/usr/bin/env python3

import time
import math
import smbus2
import spidev


# ============================================================
# NASTAVENÍ
# ============================================================

I2C_BUS = 1

# PCF8574 typicky 0x20–0x27.
# Ty máš aktuálně 0x25, 0x26, 0x27.
PCF_SCAN_ADDRESSES = range(0x20, 0x28)

SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 100_000
SPI_MODE = 1

# MAX31865 / RTD nastavení
THREE_WIRE = True
FILTER_50HZ = True

# PT100 = 100.0
# PT1000 = 1000.0
RNOMINAL = 100.0

# PT100 MAX31865 modul typicky 430 Ω
# PT1000 MAX31865 modul typicky 4300 Ω
DEFAULT_RREF = 430.0

# Výpis měření
PRINT_INTERVAL = 1.0

# Zahodit první scany po startu continuous režimu
WARMUP_SCANS = 3

# Automaticky čistit fault bity.
# Důležité hlavně při přepojování čidel za běhu.
AUTO_CLEAR_FAULTS = True

# Pokud je True, vypisují se všechny nalezené MAX31865 moduly,
# i když nemají připojené čidlo.
SHOW_FAILED_CHANNELS = True

# Pokud chceš vypisovat jen OK kanály, dej:
# SHOW_FAILED_CHANNELS = False


# ============================================================
# MAX31865 REGISTRY
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


# ============================================================
# POMOCNÉ FUNKCE
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
        messages.append("REFIN- < 0.85 x VBIAS / FORCE open")
    if fault & 0x08:
        messages.append("RTDIN- < 0.85 x VBIAS / FORCE open")
    if fault & 0x04:
        messages.append("over/under voltage")

    if not messages:
        messages.append(f"unknown fault 0x{fault:02X}")

    return ", ".join(messages)


def resistance_to_temperature_celsius(rtd_resistance, rnominal=100.0):
    """
    Přepočet odporu PT100/PT1000 na teplotu podle Callendar-Van Dusen.
    """

    a = 3.9083e-3
    b = -5.775e-7

    discriminant = a * a - 4 * b * (1 - rtd_resistance / rnominal)

    # Pro teploty >= 0 °C
    if discriminant >= 0:
        temp = (-a + math.sqrt(discriminant)) / (2 * b)
        if temp >= 0:
            return temp

    # Pro záporné teploty přibližně -200 až 0 °C
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
# MULTI PCF8574 CS ŘÍZENÍ
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
        Na všech expanderech nastaví všechny výstupy HIGH.
        Na cílovém expanderu stáhne jeden pin LOW.
        """

        for addr in self.addresses:
            if addr == expander_addr:
                state = 0xFF & ~(1 << pin)
            else:
                state = 0xFF

            self.bus.write_byte(addr, state)

    def transfer(self, spi, expander_addr, pin, data):
        """
        Jedna SPI transakce:
        všechny CS HIGH -> jeden CS LOW -> SPI transfer -> všechny CS HIGH
        """

        self.select(expander_addr, pin)
        time.sleep(0.0005)

        rx = spi.xfer2(data)

        time.sleep(0.0005)
        self.all_high()
        time.sleep(0.0005)

        return rx


# ============================================================
# MAX31865 SYSTÉM
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
        Nastavení thresholdů na celý rozsah.
        Tím threshold registry samy nezpůsobují falešné high/low chyby.
        """

        self.write_register(expander_addr, pin, REG_HIGH_FAULT_MSB, 0xFF)
        self.write_register(expander_addr, pin, REG_HIGH_FAULT_LSB, 0xFF)
        self.write_register(expander_addr, pin, REG_LOW_FAULT_MSB, 0x00)
        self.write_register(expander_addr, pin, REG_LOW_FAULT_LSB, 0x00)

    def clear_faults(self, expander_addr, pin, keep_continuous=False):
        """
        Vyčistí latched fault bity.

        Pokud keep_continuous=True, zachová/znovu zapne auto-conversion režim.
        To je důležité, protože samotný CONFIG_FAULT_CLEAR zápis by jinak
        mohl continuous režim vypnout.
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
        MAX31865 nemá WHOAMI registr.
        Ověřujeme:
        1) zápis/čtení CONFIG registru
        2) zápis/čtení markeru do threshold registrů
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
        marker_b = 0x50 | (pin & 0x0F)

        self.write_register(expander_addr, pin, REG_HIGH_FAULT_MSB, marker_a)
        self.write_register(expander_addr, pin, REG_HIGH_FAULT_LSB, marker_b)

        read_marker = self.read_registers(
            expander_addr,
            pin,
            REG_HIGH_FAULT_MSB,
            2,
        )

        if read_marker[0] != marker_a or read_marker[1] != marker_b:
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
        Zapne continuous / auto-conversion režim.
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
        Vypne continuous režim, bias nechá zapnutý podle config_base().
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

        raw16 = (msb << 8) | lsb
        raw15 = raw16 >> 1
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

        # Pokud se objeví fault, vyčistíme ho a zároveň zachováme continuous režim.
        # Pak krátce počkáme a přečteme hodnotu znovu.
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
                "reason": "raw=0",
                "resistance": None,
                "temperature": None,
            }

        if raw15 >= 32760:
            return {
                "ok": False,
                "raw": raw15,
                "fault": 0x00,
                "reason": "raw skoro maximum",
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
# DETEKCE I2C EXPANDERŮ
# ============================================================

def scan_pcf8574(i2c):
    found = []

    print("Skenuji PCF8574 expandery...")

    for addr in PCF_SCAN_ADDRESSES:
        try:
            i2c.read_byte(addr)
            found.append(addr)
            print(f"  nalezen expander: 0x{addr:02X}")
        except OSError:
            pass

    if not found:
        print("  nenalezen žádný PCF8574 expander")

    print()
    return found


# ============================================================
# DETEKCE MAX31865 MODULŮ
# ============================================================

def detect_max31865_modules(system, expander_addresses):
    modules = []
    logical_index = 0

    print("Detekuji MAX31865 moduly za jednotlivými CS piny...")

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
                print(f"  P{pin}: ---             chyba: {e}")

    print()

    if modules:
        print("Nalezené MAX31865 moduly:")
        for m in modules:
            print(
                f"  CH{m['index']:02d}: "
                f"{m['name']}  "
                f"expander=0x{m['expander_addr']:02X} "
                f"pin=P{m['pin']} "
                f"RREF={m['rref']}"
            )
    else:
        print("Nebyl nalezen žádný MAX31865 modul.")

    print()
    return modules


# ============================================================
# VÝPIS MĚŘENÍ
# ============================================================

def print_measurement_row(module, result):
    label = f"CH{module['index']:02d} {module['name']}"

    if result["ok"]:
        print(
            f"{label}: OK      "
            f"{result['temperature']:8.3f} °C   "
            f"R={result['resistance']:9.3f} Ω   "
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
    print(f"RNOMINAL:      {RNOMINAL} Ω")
    print(f"default RREF:  {DEFAULT_RREF} Ω")
    print()

    with smbus2.SMBus(I2C_BUS) as i2c:
        expander_addresses = scan_pcf8574(i2c)

        if not expander_addresses:
            return

        cs = MultiPCF8574CS(i2c, expander_addresses)

        # Bezpečný výchozí stav: všechny CS HIGH.
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

            print("Zapínám continuous režim na nalezených modulech...")
            for module in modules:
                system.start_continuous(
                    module["expander_addr"],
                    module["pin"],
                )

            time.sleep(0.3)

            print("Zahazuji první warmup scany...")
            for _ in range(WARMUP_SCANS):
                for module in modules:
                    system.read_channel(module)
                time.sleep(0.05)

            print()
            print("Start měření.")
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
                                f"NOT OK  chyba čtení: {e}"
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
            print("Vypínám continuous režim a nastavuji všechny CS HIGH...")

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