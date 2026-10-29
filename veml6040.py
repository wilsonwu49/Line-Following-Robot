import machine
import ustruct


VEML6040_I2C_ADDR = 0x10

# VEML6040 Register Addresses
_VEML6040_REG_CONF    = 0x00  # Configuration Register (R/W)
_VEML6040_REG_R_DATA  = 0x08  # Red Channel Data (R)
_VEML6040_REG_G_DATA  = 0x09  # Green Channel Data (R)
_VEML6040_REG_B_DATA  = 0x0A  # Blue Channel Data (R)
_VEML6040_REG_W_DATA  = 0x0B  # White Channel Data (R)

# Configuration Register Bit Masks and Values (for the low byte)
_SD_MASK   = 0x01  # Shutdown Bit (Bit 0)
_AF_MASK   = 0x02  # Auto/Force Mode Bit (Bit 1)
_TRIG_MASK = 0x04  # Trigger Bit (Bit 2, for Force Mode)
_IT_MASK   = 0x70  # Integration Time Bits (Bits 4, 5, 6)

# Integration Time (IT) settings for the CONF register (IT2, IT1, IT0 bits)
# These values are shifted to occupy bits 4, 5, 6 in the low byte.
IT_40MS   = (0b000 << 4) # 40 ms
IT_80MS   = (0b001 << 4) # 80 ms
IT_160MS  = (0b010 << 4) # 160 ms
IT_320MS  = (0b011 << 4) # 320 ms
IT_640MS  = (0b100 << 4) # 640 ms
IT_1280MS = (0b101 << 4) # 1280 ms

class VEML6040:
    """
    MicroPython driver for the VEML6040 RGBW Color Sensor.

    This class provides methods to initialize the sensor, configure its
    operation mode (auto/force, integration time), and read the 16-bit
    Red, Green, Blue, and White channel data.
    """

    def __init__(self, i2c, address=VEML6040_I2C_ADDR):
        """
        Initializes the VEML6040 sensor.

        Args:
            i2c (machine.I2C): The initialized I2C bus object.
            address (int): The I2C address of the VEML6040 sensor.
                           Defaults to 0x10.
        """
        self.i2c = i2c
        self.address = address
        self._current_conf = 0x0000 # Default configuration (all zeros)

        # Initialize sensor with default settings:
        # - Enable sensor (SD = 0)
        # - Auto mode (AF = 0)
        # - No trigger (TRIG = 0)
        # - Integration time 40ms (IT = 0b000)
        self.enable_sensor()
        self.set_integration_time(IT_1280MS)
        self.set_auto_mode()
        # The _current_conf is updated by the setter methods.
        self._write_word(_VEML6040_REG_CONF, self._current_conf)

    def _read_word(self, register):
        """
        Reads a 16-bit word from the specified register.
        The VEML6040 sends LSB first, then MSB.

        Args:
            register (int): The address of the register to read.

        Returns:
            int: The 16-bit value read from the register.
        """
        # MicroPython i2c.readfrom_mem returns bytes.
        # We need to send the register address, then read 2 bytes.
        data = self.i2c.readfrom_mem(self.address, register, 2)
        # Unpack as little-endian short (h means 2-byte short, < means little-endian)
        return ustruct.unpack('<H', data)[0]

    def _write_word(self, register, value):
        """
        Writes a 16-bit word to the specified register.
        The VEML6040 expects LSB first, then MSB.

        Args:
            register (int): The address of the register to write.
            value (int): The 16-bit value to write.
        """
        # Pack as little-endian short (h means 2-byte short, < means little-endian)
        data = ustruct.pack('<H', value)
        # MicroPython i2c.writeto_mem expects register address and then bytes.
        self.i2c.writeto_mem(self.address, register, data)

    def set_integration_time(self, it_value):
        """
        Sets the integration time for the color sensor.

        Args:
            it_value (int): One of the IT_ constants (e.g., IT_40MS, IT_1280MS).
        """
        # Clear existing IT bits and set new ones
        self._current_conf = (self._current_conf & ~_IT_MASK) | (it_value & _IT_MASK)
        self._write_word(_VEML6040_REG_CONF, self._current_conf)

    def enable_sensor(self):
        """
        Enables the VEML6040 color sensor.
        """
        self._current_conf &= (~_SD_MASK) # Clear SD bit (0 = enable)
        self._write_word(_VEML6040_REG_CONF, self._current_conf)

    def disable_sensor(self):
        """
        Disables the VEML6040 color sensor (puts it in shutdown mode).
        """
        self._current_conf |= _SD_MASK # Set SD bit (1 = disable)
        self._write_word(_VEML6040_REG_CONF, self._current_conf)

    def set_auto_mode(self):
        """
        Sets the sensor to Auto Measurement Mode.
        In this mode, the sensor continuously measures ambient light.
        """
        self._current_conf &= (~_AF_MASK) # Clear AF bit (0 = auto mode)
        self._write_word(_VEML6040_REG_CONF, self._current_conf)

    def set_force_mode(self):
        """
        Sets the sensor to Force Measurement Mode.
        In this mode, a measurement is triggered manually by calling trigger_measurement().
        """
        self._current_conf |= _AF_MASK # Set AF bit (1 = force mode)
        self._write_word(_VEML6040_REG_CONF, self._current_conf)

    def trigger_measurement(self):
        """
        Triggers a one-time measurement cycle in Force Measurement Mode.
        This method has no effect in Auto Measurement Mode.
        """
        # Set the TRIG bit to trigger a measurement
        temp_conf = self._current_conf | _TRIG_MASK
        self._write_word(_VEML6040_REG_CONF, temp_conf)
        # The TRIG bit automatically clears after the measurement.
        # No need to clear it in software unless we want to ensure it's off immediately.
        # For simplicity, we just set it and let hardware clear.

    def read_red(self):
        """
        Reads the 16-bit Red channel data.

        Returns:
            int: The raw 16-bit Red channel value.
        """
        return self._read_word(_VEML6040_REG_R_DATA)

    def read_green(self):
        """
        Reads the 16-bit Green channel data.

        Returns:
            int: The raw 16-bit Green channel value.
        """
        return self._read_word(_VEML6040_REG_G_DATA)

    def read_blue(self):
        return self._read_word(_VEML6040_REG_B_DATA)

    def read_white(self):
        return self._read_word(_VEML6040_REG_W_DATA)

    def read_rgbw(self):
        red = self._read_word(_VEML6040_REG_R_DATA)
        green = self._read_word(_VEML6040_REG_G_DATA)
        blue = self._read_word(_VEML6040_REG_B_DATA)
        white = self._read_word(_VEML6040_REG_W_DATA)
        return (red, green, blue, white)

