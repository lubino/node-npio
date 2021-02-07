/*
 * Copyright (c) 2015 Jonathan Perkin <jonathan@perkin.org.uk>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

var binding = require('bindings')('npio');
var fs = require('fs');
var util = require('util');
var EventEmitter = require('events').EventEmitter;

/*
 * Event Emitter gloop.
 */
function npio() {
  EventEmitter.call(this);
}
util.inherits(npio, EventEmitter);
module.exports = new npio;

/*
 * Constants.
 */
npio.prototype.LOW = 0x0;
npio.prototype.HIGH = 0x1;

/*
 * Supported function select modes.  INPUT and OUTPUT match the bcm2835
 * function select integers.  PWM is handled specially.
 */
npio.prototype.INPUT = 0x0;
npio.prototype.OUTPUT = 0x1;
npio.prototype.PWM = 0x2;

/*
 * Configure builtin pullup/pulldown resistors.
 */
npio.prototype.PULL_OFF = 0x0;
npio.prototype.PULL_DOWN = 0x1;
npio.prototype.PULL_UP = 0x2;

/*
 * Pin edge detect events.  Default to both.
 */
npio.prototype.POLL_LOW = 0x1;	/* Falling edge detect */
npio.prototype.POLL_HIGH = 0x2;	/* Rising edge detect */
npio.prototype.POLL_BOTH = 0x3;	/* POLL_LOW | POLL_HIGH */

/*
 * Reset pin status on close (default), or preserve current status.
 */
npio.prototype.PIN_PRESERVE = 0x0;
npio.prototype.PIN_RESET = 0x1;

/*
 * GPIO Pad Control
 */
npio.prototype.PAD_GROUP_0_27     = 0x0;
npio.prototype.PAD_GROUP_28_45    = 0x1;
npio.prototype.PAD_GROUP_46_53    = 0x2;
npio.prototype.PAD_DRIVE_2mA      = 0x00;
npio.prototype.PAD_DRIVE_4mA      = 0x01;
npio.prototype.PAD_DRIVE_6mA      = 0x02;
npio.prototype.PAD_DRIVE_8mA      = 0x03;
npio.prototype.PAD_DRIVE_10mA     = 0x04;
npio.prototype.PAD_DRIVE_12mA     = 0x05;
npio.prototype.PAD_DRIVE_14mA     = 0x06;
npio.prototype.PAD_DRIVE_16mA     = 0x07;
npio.prototype.PAD_HYSTERESIS     = 0x08;
npio.prototype.PAD_SLEW_UNLIMITED = 0x10;

/*
 * Default pin mode is 'physical'.  Other option is 'gpio'
 */
var npio_inited = false;
var npio_options = {
  gpiomem: true,
  mapping: 'physical',
  mock: false,
};

/* Default mock mode if hardware is unsupported. */
var defmock = "raspi-3";

/*
 * Wrapper functions.
 */
function bindcall(bindfunc, optarg) {
  if (npio_options.mock)
    return;

  return bindfunc(optarg);
}

function bindcall2(bindfunc, arg1, arg2) {
  if (npio_options.mock)
    return;

  return bindfunc(arg1, arg2);
}

function bindcall3(bindfunc, arg1, arg2, arg3) {
  if (npio_options.mock)
    return;

  return bindfunc(arg1, arg2, arg3);
}

function warn(msg) {
  console.error("WARNING: " + msg);
}

/*
 * Map physical pin to BCM GPIOxx numbering.  There are currently three
 * layouts:
 *
 *   PINMAP_26_R1:  26-pin early original models A and B (PCB rev 1.0)
 *   PINMAP_26:     26-pin standard models A and B (PCB rev 2.0)
 *   PINMAP_40:     40-pin models
 *
 * A -1 indicates an unusable pin.  Each table starts with a -1 so that we
 * can index into the array by pin number.
 */
var pincache = {};
var pinmap = null;
var chipType = null;
var pinmaps = {
	/*
	 * Original Raspberry Pi, PCB revision 1.0
	 */
	PINMAP_26_R1: [
		-1,
		-1, -1,		/*  P1  P2 */
		 0, -1,		/*  P3  P4 */
		 1, -1,		/*  P5  P6 */
		 4, 14,		/*  P7  P8 */
		-1, 15,		/*  P9  P10 */
		17, 18,		/* P11  P12 */
		21, -1,		/* P13  P14 */
		22, 23,		/* P15  P16 */
		-1, 24,		/* P17  P18 */
		10, -1,		/* P19  P20 */
		 9, 25,		/* P21  P22 */
		11,  8,		/* P23  P24 */
		-1,  7		/* P25  P26 */
	],
	/*
	 * Original Raspberry Pi, PCB revision 2.0.
	 *
	 * Differs to R1 on pins 3, 5, and 13.
	 *
	 * XXX: no support yet for the P5 header pins.
	 */
	PINMAP_26: [
		-1,
		-1, -1,		/*  P1  P2 */
		 2, -1,		/*  P3  P4 */
		 3, -1,		/*  P5  P6 */
		 4, 14,		/*  P7  P8 */
		-1, 15,		/*  P9  P10 */
		17, 18,		/* P11  P12 */
		27, -1,		/* P13  P14 */
		22, 23,		/* P15  P16 */
		-1, 24,		/* P17  P18 */
		10, -1,		/* P19  P20 */
		 9, 25,		/* P21  P22 */
		11,  8,		/* P23  P24 */
		-1,  7		/* P25  P26 */
	],
	/*
	 * Raspberry Pi 40-pin models.
	 *
	 * First 26 pins are the same as PINMAP_26.
	 */
	PINMAP_40: [
		-1,
		-1, -1,		/*  P1  P2 */
		 2, -1,		/*  P3  P4 */
		 3, -1,		/*  P5  P6 */
		 4, 14,		/*  P7  P8 */
		-1, 15,		/*  P9  P10 */
		17, 18,		/* P11  P12 */
		27, -1,		/* P13  P14 */
		22, 23,		/* P15  P16 */
		-1, 24,		/* P17  P18 */
		10, -1,		/* P19  P20 */
		 9, 25,		/* P21  P22 */
		11,  8,		/* P23  P24 */
		-1,  7,		/* P25  P26 */
		 0,  1,		/* P27  P28 */
		 5, -1,		/* P29  P30 */
		 6, 12,		/* P31  P32 */
		13, -1,		/* P33  P34 */
		19, 16,		/* P35  P36 */
		26, 20,		/* P37  P38 */
		-1, 21		/* P39  P40 */
	],
  PINMAP_ORANGEPI: [
    -1,
    -1, -1,
    12, -1,
    11, -1,
     6, 13,
    -1, 14,
     1, 110,
     0, -1,
     3, 68,
    -1, 71,
    64, -1,
    65, 2,
    66, 67,
    -1, 21,
    19, 18,
     7, -1,
     8, 200,
     9, -1,
    10, 201,
    20, 198,
    -1, 199
	],
	PINMAP_NANOPI_M1_PLUS: [
    -1,
    -1, -1,
    12, -1,
    11, -1,
    203, 198,
    -1, 199,
     0, 6,
     2, -1,
     3, 200,
    -1, 201,
    64, -1,
    65, 1,
    66, 67,
    -1, 17,
    19, 18,
    20, -1,
    21, -1,
    -1, -1,
    -1, -1,
     9, -1,
    -1, -1
  ],
  PINMAP_NEO: [
    -1,
    -1, -1,
    12, -1,
    11, -1,
    203, 198,
    -1, 199,
     0, 6,
     2, -1,
     3, 200,
    -1, 201,
    64, -1,
    65, 1,
    66, 67,
    -1, 17,
    19, 18,
    20, -1,
    21, 7,
     8, -1,
    16, 13,
     9,15,
    -1, 14
  ],
  PINMAP_NEO2: [
    -1,
    -1, -1,
    12, -1,
    11, -1,
    203, 198,
    -1, 199,
    0, 6,
    2, -1,
    3, 200,
    -1, 201,
    64, -1,
    65, 1,
    66, 77
  ]
};

/*
 * Pin event polling.  We track which pins are being monitored, and create a
 * bitmask for efficient checks.  The event_poll function is executed in a
 * setInterval() context whenever any pins are being monitored, and emits
 * events when their EDS bit is set.
 */
var event_pins = {};
var event_mask = 0x0;
var event_running = false;

function event_poll() {
  var active = bindcall(binding.gpio_event_poll, event_mask);

  for (var gpiopin in event_pins) {
    if (active & (1 << gpiopin))
      module.exports.emit('pin' + gpiopin);
  }
}

function read(path) {
  try {
    var contents = fs.readFileSync(path, 'ascii');
    if (contents) {
      return contents.toString();
    }
  } catch (err) {
    /* It is the purpose of this function to silently catch this error */
  }
  return false;
}

/*
 * Set up GPIO mapping based on board revision.
 */
function detect_pinmap() {

  var cpuinfo, boardrev, match, cpu, mips;

  cpuinfo = read('/proc/cpuinfo');

  if (!cpuinfo) return false;

  cpuinfo.split(/\n/).forEach(function(line) {
    if (match = line.match(/^Revision.*(.{4})/)) {
      boardrev = parseInt(match[1], 16);
    } else if (match = line.match(/^Hardware.*:\s([^\s]+)/)) {
      cpu = match[1];
    } else if (match = line.match(/^BogoMIPS.*:\s([^\s]+)/)) {
      mips = +match[1];
    }
  });

  switch (boardrev) {
    case 0x2:
    case 0x3:
      pinmap = "PINMAP_26_R1";
      chipType = "BCM2835";
      break;
    case 0x4:
    case 0x5:
    case 0x6:
    case 0x7:
    case 0x8:
    case 0x9:
    case 0xd:
    case 0xe:
    case 0xf:
      pinmap = "PINMAP_26";
      chipType = "BCM2835";
      break;
    case 0x10:
    case 0x12:
    case 0x13:
    case 0x15:
    case 0x92:
    case 0x93:
    case 0xc1:
    case 0x1041:
    case 0x2042:
    case 0x2082:
      pinmap = "PINMAP_40";
      chipType = "BCM2835";
      break;

    default:
      switch (cpu) {
        case 'Allwinner':
          {
            var model = read('/proc/device-tree/model');
            switch (model) {
              // Not well tested. Working on my device running Armbian
              case 'FriendlyElec NanoPi M1 Plus\u0000':
                chipType = "SUNXI";
                pinmap = "PINMAP_NANOPI_M1_PLUS";
                break;
              case 'FriendlyARM NanoPi NEO\u0000':
                chipType = "SUNXI";
                pinmap = "PINMAP_NEO";
                break;

              default:
                return false;
            }
          }
          break;

        case 'Allwinnersun50iw2Family':
          //Allwinner H5 CPU
          pinmap = "PINMAP_NEO2";
          chipType = "SUNXI";
          break;

        case 'sun50i':
        case 'sun8i':
          //Allwinner H3 CPU
          chipType = "SUNXI";
          if (mips > 1000) {
            pinmap = "PINMAP_ORANGEPI";
          } else {
            pinmap = "PINMAP_NEO";
          }
          break;

        default:
          return false;
      }
  }

  return true;
}

function set_mock_pinmap() {
  switch (npio_options.mock) {
    case 'raspi-b-r1':
      pinmap = "PINMAP_26_R1";
      chipType = "BCM2835";
      break;

    case 'raspi-a':
    case 'raspi-b':
      pinmap = "PINMAP_26";
      chipType = "BCM2835";
      break;

    case 'raspi-a+':
    case 'raspi-b+':
    case 'raspi-2':
    case 'raspi-3':
    case 'raspi-zero':
    case 'raspi-zero-w':
      pinmap = "PINMAP_40";
      chipType = "BCM2835";
      break;

    default:
      return false;
  }

  return true;
}

function pin_to_gpio(pin) {
  if (pincache[pin])
    return pincache[pin];

  switch (npio_options.mapping) {
    case 'physical':
      if (pinmaps[pinmap][pin] == -1 || pinmaps[pinmap][pin] == null)
        throw new Error("Invalid pin: physical=" + pin);
      pincache[pin] = pinmaps[pinmap][pin];
      break;
    case 'gpio':
      if (pinmaps[pinmap].indexOf(pin) === -1)
        throw new Error("Invalid pin: gpio=" + pin);
      pincache[pin] = pin;
      break;
    default:
      throw new Error("Unsupported GPIO mode");
  }

  return pincache[pin];
}

function check_sys_gpio(pin) {
  if (chipType === "BCM2835" && fs.existsSync('/sys/class/gpio/gpio' + pin))
    throw new Error("GPIO" + pin + " is currently in use by /sys/class/gpio");
}

function get_pwm_function(pin) {
  var gpiopin = pin_to_gpio(pin);

  switch (gpiopin) {
    case 12:
    case 13:
      return 4; /* BCM2835_GPIO_FSEL_ALT0 */
    case 18:
    case 19:
      return 2; /* BCM2835_GPIO_FSEL_ALT5 */
    default:
      throw new Error("Pin " + pin + " does not support hardware PWM");
  }
}

function get_pwm_channel(pin) {
  var gpiopin = pin_to_gpio(pin);

  switch (gpiopin) {
    case 12:
    case 18:
      return 0;
    case 13:
    case 19:
      return 1;
    default:
      throw new Error("Unknown PWM channel for pin " + pin);
  }
}

function set_pin_pwm(pin) {
  var gpiopin = pin_to_gpio(pin);
  var channel, func;

  if (npio_options.gpiomem)
    throw new Error("PWM not available in gpiomem mode");

  check_sys_gpio(gpiopin);

  /*
   * PWM channels and alternate functions differ from pin to pin, set
   * them up appropriately based on selected pin.
   */
  channel = get_pwm_channel(pin);
  func = get_pwm_function(pin);

  bindcall2(binding.gpio_function, gpiopin, func);

  /*
   * For now we assume mark-space mode, balanced is unsupported.
   */
  bindcall3(binding.pwm_set_mode, channel, 1, 1);
}

/*
 * GPIO
 */

/*
 * Default warning handler, if the user registers their own then this one
 * is cancelled.
 */
function default_warn_handler(msg) {
  if (module.exports.listenerCount('warn') > 1) {
    module.exports.removeListener('warn', default_warn_handler);
    return;
  }
  warn(msg);
}

module.exports.on('warn', default_warn_handler);

npio.prototype.init = function(opts) {
  opts = opts || {};

  for (var k in npio_options) {
    if (k in opts)
      npio_options[k] = opts[k];
  }

  /*
   * Invalidate the pin cache and mapping as we may be in the process
   * of changing them.
   */
  pincache = {};
  pinmap = null;
  chipType = null;

  /*
   * Allow the user to specify a mock board to emulate, otherwise try
   * to autodetect the board, and fall back to mock mode if running on
   * an unsupported platform.
   */
  if (npio_options.mock) {
    if (!set_mock_pinmap())
      throw new Error("Unsupported mock mode " + npio_options.mock);
  } else {
    if (!detect_pinmap()) {
      module.exports.emit('warn',
        'Hardware auto-detect failed, running in ' +
        defmock + ' mock mode');
      npio_options.mock = defmock;
      set_mock_pinmap();
    }
  }

  /*
   * init npio library
   */
  var gpiomem = Number(npio_options.gpiomem);
  if (chipType === "SUNXI") gpiomem += 2;
  bindcall(binding.npio_init, gpiomem);
  npio_inited = true;
};

npio.prototype.open = function(pin, mode, init) {
  if (!npio_inited) {
    /* PWM requires full /dev/mem */
    if (mode === npio.prototype.PWM)
      npio_options.gpiomem = false;
    npio.prototype.init();
  }

  var gpiopin = pin_to_gpio(pin);

  check_sys_gpio(gpiopin);

  switch (mode) {
    case npio.prototype.INPUT:
      if (init !== undefined)
        bindcall2(binding.gpio_pud, gpiopin, init);
      return bindcall2(binding.gpio_function, gpiopin, npio.prototype.INPUT);
    case npio.prototype.OUTPUT:
      if (init !== undefined)
        bindcall2(binding.gpio_write, gpiopin, init);
      return bindcall2(binding.gpio_function, gpiopin, npio.prototype.OUTPUT);
    case npio.prototype.PWM:
      return set_pin_pwm(pin);
    default:
      throw new Error("Unsupported mode " + mode);
  }
};

npio.prototype.mode = function(pin, mode) {
  var gpiopin = pin_to_gpio(pin);

  switch (mode) {
    case npio.prototype.INPUT:
      return bindcall2(binding.gpio_function, gpiopin, npio.prototype.INPUT);
    case npio.prototype.OUTPUT:
      return bindcall2(binding.gpio_function, gpiopin, npio.prototype.OUTPUT);
    case npio.prototype.PWM:
      return set_pin_pwm(pin);
    default:
      throw new Error("Unsupported mode " + mode);
  }
};

npio.prototype.read = function(pin) {
  return bindcall(binding.gpio_read, pin_to_gpio(pin));
};

npio.prototype.readbuf = function(pin, buf, len) {
  if (len === undefined)
    len = buf.length;

  if (len > buf.length)
    throw new Error("Buffer not large enough to accommodate request");

  return bindcall3(binding.gpio_readbuf, pin_to_gpio(pin), buf, len);
};

npio.prototype.write = function(pin, value) {
  return bindcall2(binding.gpio_write, pin_to_gpio(pin), value);
};

npio.prototype.writebuf = function(pin, buf, len) {
  if (len === undefined)
    len = buf.length;

  if (len > buf.length)
    throw new Error("Buffer not large enough to accommodate request");

  return bindcall3(binding.gpio_writebuf, pin_to_gpio(pin), buf, len);
};

npio.prototype.readpad = function(group) {
  if (npio_options.gpiomem)
    throw new Error("Pad control not available in gpiomem mode");

  return bindcall(binding.gpio_pad_read, group);
};

npio.prototype.writepad = function(group, control) {
  if (npio_options.gpiomem)
    throw new Error("Pad control not available in gpiomem mode");

  bindcall2(binding.gpio_pad_write, group, control);
};

npio.prototype.pud = function(pin, state) {
  bindcall2(binding.gpio_pud, pin_to_gpio(pin), state);
};

npio.prototype.poll = function(pin, cb, direction) {
  var gpiopin = pin_to_gpio(pin);

  if (direction === undefined)
    direction = npio.prototype.POLL_BOTH;

  /*
   * If callback is a function, set up pin for polling, otherwise
   * clear it.
   */
  if (typeof(cb) === 'function') {
    if (gpiopin in event_pins)
      throw new Error("Pin " + pin + " is already listening for events.");

    bindcall2(binding.gpio_event_set, gpiopin, direction);

    var pincb = function() {
      cb(pin);
    };
    module.exports.on('pin' + gpiopin, pincb);

    event_pins[gpiopin] = pincb;
    event_mask |= (1 << gpiopin);

    if (!(event_running))
      event_running = setInterval(event_poll, 1);
  } else {
    if (!(gpiopin in event_pins))
      throw new Error("Pin " + pin + " is not listening for events.");

    bindcall(binding.gpio_event_clear, gpiopin);

    npio.prototype.removeListener('pin' + gpiopin, event_pins[gpiopin]);

    delete event_pins[gpiopin];
    event_mask &= ~(1 << gpiopin);

    if (Object.keys(event_pins).length === 0) {
      clearInterval(event_running);
      event_running = false;
    }
  }
};

npio.prototype.close = function(pin, reset) {
  var gpiopin = pin_to_gpio(pin);

  if (reset === undefined)
    reset = npio.prototype.PIN_RESET;

  if (gpiopin in event_pins)
    npio.prototype.poll(pin, null);

  if (reset) {
    if (!npio_options.gpiomem)
      npio.prototype.pud(pin, npio.prototype.PULL_OFF);
    npio.prototype.mode(pin, npio.prototype.INPUT);
  }
};

/*
 * PWM
 */
npio.prototype.pwmSetClockDivider = function(divider) {
  if (divider !== 0 && (divider & (divider - 1)) !== 0)
    throw new Error("Clock divider must be zero or power of two");

  return bindcall(binding.pwm_set_clock, divider);
};

npio.prototype.pwmSetRange = function(pin, range) {
  var channel = get_pwm_channel(pin);

  return bindcall2(binding.pwm_set_range, channel, range);
};

npio.prototype.pwmSetData = function(pin, data) {
  var channel = get_pwm_channel(pin);

  return bindcall2(binding.pwm_set_data, channel, data);
};

/*
 * i²c
 */
npio.prototype.i2cBegin = function() {
  if (!npio_inited) {
    /* i²c requires full /dev/mem */
    npio_options.gpiomem = false;
    npio.prototype.init();
  }

  if (npio_options.gpiomem)
    throw new Error("i²c not available in gpiomem mode");

  bindcall(binding.i2c_begin);
};

npio.prototype.i2cSetSlaveAddress = function(addr) {
  return bindcall(binding.i2c_set_slave_address, addr);
};

npio.prototype.i2cSetClockDivider = function(divider) {
  if ((divider % 2) !== 0)
    throw new Error("Clock divider must be an even number");

  return bindcall(binding.i2c_set_clock_divider, divider);
};

npio.prototype.i2cSetBaudRate = function(baud) {
  return bindcall(binding.i2c_set_baudrate, baud);
};

npio.prototype.i2cRead = function(buf, len) {
  if (len === undefined)
    len = buf.length;

  if (len > buf.length)
    throw new Error("Buffer not large enough to accommodate request");

  return bindcall2(binding.i2c_read, buf, len);
};

npio.prototype.i2cWrite = function(buf, len) {
  if (len === undefined)
    len = buf.length;

  if (len > buf.length)
    throw new Error("Buffer not large enough to accommodate request");

  return bindcall2(binding.i2c_write, buf, len);
};

npio.prototype.i2cEnd = function() {
  bindcall(binding.i2c_end);
};

/*
 * SPI
 */
npio.prototype.spiBegin = function() {
  if (!npio_inited) {
    /* SPI requires full /dev/mem */
    npio_options.gpiomem = false;
    npio.prototype.init();
  }

  if (npio_options.gpiomem)
    throw new Error("SPI not available in gpiomem mode");

  bindcall(binding.spi_begin);
};

npio.prototype.spiChipSelect = function(cs) {
  return bindcall(binding.spi_chip_select, cs);
};

npio.prototype.spiSetCSPolarity = function(cs, active) {
  return bindcall2(binding.spi_set_cs_polarity, cs, active);
};

npio.prototype.spiSetClockDivider = function(divider) {
  if ((divider % 2) !== 0 || divider < 0 || divider > 65536)
    throw new Error("Clock divider must be an even number between 0 and 65536");

  return bindcall(binding.spi_set_clock_divider, divider);
};

npio.prototype.spiSetDataMode = function(mode) {
  return bindcall(binding.spi_set_data_mode, mode);
};

npio.prototype.spiTransfer = function(txbuf, rxbuf, len) {
  return bindcall3(binding.spi_transfer, txbuf, rxbuf, len);
};

npio.prototype.spiWrite = function(buf, len) {
  return bindcall2(binding.spi_write, buf, len);
};

npio.prototype.spiEnd = function() {
  bindcall(binding.spi_end);
};

/*
 * Misc functions.
 */
npio.prototype.sleep = function(secs) {
  bindcall(binding.npio_usleep, secs * 1000000);
};

npio.prototype.msleep = function(msecs) {
  bindcall(binding.npio_usleep, msecs * 1000);
};

npio.prototype.usleep = function(usecs) {
  bindcall(binding.npio_usleep, usecs);
};

process.on('exit', function(code) {
  bindcall(binding.npio_close);
});
