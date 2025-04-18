// Copyright (c) 2020-2021 Damien P. George
// Copyright (c) 2024 - 2025 Kevin G. Schlosser


#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "extmod/modmachine.h"

#include "../../../../micropy_updates/common/mp_spi_common.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

#define SPI_UNUSED(x) ((void)x)

#define MICROPY_HW_SPI_MAX      (2)

#define DEFAULT_SPI_BAUDRATE    (1000000)
#define DEFAULT_SPI_POLARITY    (0)
#define DEFAULT_SPI_PHASE       (0)
#define DEFAULT_SPI_BITS        (8)
#define DEFAULT_SPI_FIRSTBIT    (SPI_MSB_FIRST)
#define DEFAULT_SPI_CS_PIN      (-1)

#ifdef MICROPY_HW_SPI_NO_DEFAULT_PINS

// With no default SPI, need to require the pin args.
#define MICROPY_HW_SPI0_SCK     (0)
#define MICROPY_HW_SPI0_MOSI    (0)
#define MICROPY_HW_SPI0_MISO    (0)
#define MICROPY_HW_SPI1_SCK     (0)
#define MICROPY_HW_SPI1_MOSI    (0)
#define MICROPY_HW_SPI1_MISO    (0)
#define MICROPY_SPI_PINS_ARG_OPTS MP_ARG_REQUIRED

#else

// Most boards do not require pin args.
#define MICROPY_SPI_PINS_ARG_OPTS 0

#ifndef MICROPY_HW_SPI0_SCK
#if PICO_DEFAULT_SPI == 0
#define MICROPY_HW_SPI0_SCK     (PICO_DEFAULT_SPI_SCK_PIN)
#define MICROPY_HW_SPI0_MOSI    (PICO_DEFAULT_SPI_TX_PIN)
#define MICROPY_HW_SPI0_MISO    (PICO_DEFAULT_SPI_RX_PIN)
#else
#define MICROPY_HW_SPI0_SCK     (6)
#define MICROPY_HW_SPI0_MOSI    (7)
#define MICROPY_HW_SPI0_MISO    (4)
#endif
#endif

#ifndef MICROPY_HW_SPI1_SCK
#if PICO_DEFAULT_SPI == 1
#define MICROPY_HW_SPI1_SCK     (PICO_DEFAULT_SPI_SCK_PIN)
#define MICROPY_HW_SPI1_MOSI    (PICO_DEFAULT_SPI_TX_PIN)
#define MICROPY_HW_SPI1_MISO    (PICO_DEFAULT_SPI_RX_PIN)
#else
#define MICROPY_HW_SPI1_SCK     (10)
#define MICROPY_HW_SPI1_MOSI    (11)
#define MICROPY_HW_SPI1_MISO    (8)
#endif
#endif

#endif

// SPI0 can be GP{0..7,16..23}, SPI1 can be GP{8..15,24..29}.
#define IS_VALID_PERIPH(spi, pin)   ((((pin) & 8) >> 3) == (spi))
// GP{2,6,10,14,...}
#define IS_VALID_SCK(spi, pin)      (((pin) & 3) == 2 && IS_VALID_PERIPH(spi, pin))
// GP{3,7,11,15,...}
#define IS_VALID_MOSI(spi, pin)     (((pin) & 3) == 3 && IS_VALID_PERIPH(spi, pin))
// GP{0,4,8,10,...}
#define IS_VALID_MISO(spi, pin)     (((pin) & 3) == 0 && IS_VALID_PERIPH(spi, pin))

/*
typedef enum _mp_machine_hw_spi_state_t {
    MP_SPI_STATE_STOPPED,
    MP_SPI_STATE_STARTED,
    MP_SPI_STATE_SENDING
} mp_machine_hw_spi_state_t;

typedef struct _mp_machine_hw_spi_bus_obj_t {
    uint8_t host;
    mp_obj_t sck;
    mp_obj_t mosi;
    mp_obj_t miso;
    int16_t device_count;
    mp_machine_hw_spi_state_t state;
    void *user_data;
} mp_machine_hw_spi_bus_obj_t;


typedef struct _machine_hw_spi_obj_t {
    mp_obj_base_t base;
    uint32_t baudrate;
    uint8_t polarity;
    uint8_t phase;
    uint8_t bits;
    uint8_t firstbit;
    mp_obj_t cs;
    mp_machine_hw_spi_bus_obj_t *spi_bus;
    void *user_data;
} machine_hw_spi_obj_t;

*/

/* Pointers for 2 hardware SPI bus objects */
static mp_machine_hw_spi_bus_obj_t *rp2_machine_spi_bus_objs[MICROPY_HW_SPI_MAX];

/* Forward declarations*/
void machine_hw_spi_bus_deinit_internal(mp_machine_hw_spi_bus_obj_t *self);

/* deinit both busses and their defined devices*/
void mp_machine_hw_spi_bus_deinit_all(void)
{
    for (int i=0;i<MICROPY_HW_SPI_MAX;i++) {
        if (rp2_machine_spi_bus_objs[i] != NULL) {
            machine_hw_spi_bus_deinit_internal(rp2_machine_spi_bus_objs[i]);
            m_del_obj(mp_machine_hw_spi_bus_obj_t, rp2_machine_spi_bus_objs[i]);
            rp2_machine_spi_bus_objs[i] = NULL;
        }
    }
}

void mp_machine_hw_spi_bus_add_device(mp_machine_hw_spi_device_obj_t *device)
{
    device->spi_bus->device_count++;
    device->spi_bus->devices = m_realloc(device->spi_bus->devices, device->spi_bus->device_count * sizeof(mp_machine_hw_spi_device_obj_t *));
}


void mp_machine_hw_spi_bus_remove_device(mp_machine_hw_spi_device_obj_t *device)
{
    uint8_t i;
    /*Find the device in the list*/
    for(i = 0; i < device->spi_bus->device_count; i++) {
        if(device->spi_bus->devices[i] == device) break;
    }

    for( ;i < device->spi_bus->device_count - 1; i++) {
        device->spi_bus->devices[i] = device->spi_bus->devices[i + 1];
    }

    device->spi_bus->device_count--;
    device->spi_bus->devices = m_realloc(device->spi_bus->devices, device->spi_bus->device_count * sizeof(mp_machine_hw_spi_device_obj_t *));

    if (device->spi_bus->device_count == 0) {
        device->spi_bus->deinit(device->spi_bus);
    }
}


static void machine_hw_spi_device_deinit_callback(mp_machine_hw_spi_device_obj_t *self)
{
    if (!self->active) return;

    switch (spi_bus_remove_device((spi_device_handle_t)self->user_data)) {
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("invalid configuration"));
            return;

        case ESP_ERR_INVALID_STATE:
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("SPI device already freed"));
            return;
    }
    int cs = (int)mp_obj_get_int(self->cs);

    if (cs != -1) {
        esp_rom_gpio_pad_select_gpio(cs);
        esp_rom_gpio_connect_out_signal(cs, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction(cs, GPIO_MODE_INPUT);
    }
}

static void machine_hw_spi_device_deinit_internal(mp_machine_hw_spi_device_obj_t *self)
{
    if (!self->active) return;

    switch (spi_bus_remove_device((spi_device_handle_t)self->user_data)) {
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("invalid configuration"));
            return;

        case ESP_ERR_INVALID_STATE:
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("SPI device already freed"));
            return;
    }
    int cs = (int)mp_obj_get_int(self->cs);

    if (cs != -1) {
        esp_rom_gpio_pad_select_gpio(cs);
        esp_rom_gpio_connect_out_signal(cs, SIG_GPIO_OUT_IDX, false, false);
        gpio_set_direction(cs, GPIO_MODE_INPUT);
    }

    mp_machine_hw_spi_bus_remove_device(self);
}


static void machine_hw_spi_device_deinit(mp_obj_base_t *self_in)
{
    mp_machine_hw_spi_device_obj_t *self = (mp_machine_hw_spi_device_obj_t *)self_in;
    machine_hw_spi_device_deinit_internal(self);
}


void machine_hw_spi_bus_deinit_internal(mp_machine_hw_spi_bus_obj_t *self)
{
    if (self->state == MP_SPI_STATE_STOPPED) return;

    for (uint8_t i = 0; i < self->device_count; i++) {
        self->devices[i]->deinit(self->devices[i]);
        self->devices[i]->active = false;
    }

    self->device_count = 0;
    self->devices = m_realloc(self->devices, self->device_count * sizeof(mp_machine_hw_spi_device_obj_t *));

    switch (spi_bus_free((spi_host_device_t)self->host)) {
        case ESP_ERR_INVALID_ARG:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("invalid configuration"));
            return;

        case ESP_ERR_INVALID_STATE:
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("SPI bus already freed"));
            return;
    }

    disable_gpio((int)mp_obj_get_int(self->miso));
    disable_gpio((int)mp_obj_get_int(self->mosi));
    disable_gpio((int)mp_obj_get_int(self->sck));
    disable_gpio((int)mp_obj_get_int(self->data2));
    disable_gpio((int)mp_obj_get_int(self->data3));
    disable_gpio((int)mp_obj_get_int(self->data4));
    disable_gpio((int)mp_obj_get_int(self->data5));
    disable_gpio((int)mp_obj_get_int(self->data6));
    disable_gpio((int)mp_obj_get_int(self->data7));

    self->state = MP_SPI_STATE_STOPPED;
}

static void machine_spi_init(mp_obj_base_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    SPI_UNUSED(self_in);
    SPI_UNUSED(n_args);
    SPI_UNUSED(pos_args);
    SPI_UNUSED(kw_args);
}


static void machine_spi_device_deinit(mp_obj_base_t *self_in)
{
    mp_machine_hw_spi_device_obj_t *self = (mp_machine_hw_spi_device_obj_t *)self_in;
    if (self->active) {
        self->spi_bus->device_count--;
        self->active = false;

        if (self->spi_bus->device_count == 0) {
            self->spi_bus->state = MP_SPI_STATE_STOPPED;
        }
    }
}




static mp_obj_t machine_hw_spi_bus_deinit(mp_obj_t self_in)
{
    mp_machine_hw_spi_bus_obj_t *self = MP_OBJ_TO_PTR(self_in);
    machine_hw_spi_bus_deinit_internal(self);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_spi_bus_deinit_obj, machine_hw_spi_bus_deinit);


/******************************************************************************/
// MicroPython bindings for hw_spi

mp_obj_t machine_hw_spi_bus_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {

    enum { ARG_host, ARG_mosi, ARG_miso, ARG_sck };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_host,       MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_mosi,       MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_miso,       MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_sck,        MP_ARG_KW_ONLY | MP_ARG_REQUIRED | MP_ARG_INT }
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t host = (uint8_t)args[ARG_host].u_int;
    int mosi = (int)args[ARG_mosi].u_int;
    int miso = (int)args[ARG_miso].u_int;
    int sck = (int)args[ARG_sck].u_int;
/*    int data2 = -1;
    int data3 = -1;
    int data4 = -1;
    int data5 = -1;
    int data6 = -1;
    int data7 = -1;

    bool dual = false;
    bool quad = false;
    bool octal = false; */

    mp_machine_hw_spi_bus_obj_t *self;

/*    if (mosi != -1 && mosi != -1) dual = true;

    if (args[ARG_quad_pins].u_obj != mp_const_none) {
        mp_obj_tuple_t *quad_data_pins = MP_OBJ_TO_PTR(args[ARG_quad_pins].u_obj);

        if (!dual) {
            mp_raise_msg(
                &mp_type_ValueError,
                MP_ERROR_TEXT("You MUST supply both the MISO and MOSI pins to use quad mode")
            );
            return mp_const_none;
        }
        if (quad_data_pins->len != 2) {
            mp_raise_msg_varg(
                &mp_type_ValueError,
                MP_ERROR_TEXT("2 additional pins are mneeded for quad SPI not %d"),
                quad_data_pins->len
            );
            return mp_const_none;
        }

        data2 = (int)mp_obj_get_int(quad_data_pins->items[0]);
        data3 = (int)mp_obj_get_int(quad_data_pins->items[1]);

        quad = true;
        octal = false;

    } else if (args[ARG_octal_pins].u_obj != mp_const_none) {
        mp_obj_tuple_t *octal_data_pins = MP_OBJ_TO_PTR(args[ARG_octal_pins].u_obj);

        if (!dual) {
            mp_raise_msg(
                &mp_type_ValueError,
                MP_ERROR_TEXT("You MUST supply both the MISO and MOSI pins to use octal mode")
            );
            return mp_const_none;
        }

        if (octal_data_pins->len != 6) {
            mp_raise_msg_varg(
                &mp_type_ValueError,
                MP_ERROR_TEXT("6 additional pins are needed for octal SPI not %d"),
                octal_data_pins->len
            );
            return mp_const_none;
        }

        data2 = (int)mp_obj_get_int(octal_data_pins->items[0]);
        data3 = (int)mp_obj_get_int(octal_data_pins->items[1]);
        data4 = (int)mp_obj_get_int(octal_data_pins->items[2]);
        data5 = (int)mp_obj_get_int(octal_data_pins->items[3]);
        data6 = (int)mp_obj_get_int(octal_data_pins->items[4]);
        data7 = (int)mp_obj_get_int(octal_data_pins->items[5]);

        quad = true;
        octal = true;
    } */

    if (0 <= host && host < MICROPY_HW_SPI_MAX) {
        self = rp2_machine_spi_bus_objs[host];
    } else {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("SPI(%d) doesn't exist"), host);
        return mp_const_none;
    }

    bool reconfigure = false;

    if (self == NULL) {
        reconfigure = true;
        self = m_new_obj(mp_machine_hw_spi_bus_obj_t);
        self->base.type = &mp_machine_hw_spi_bus_type;
        self->host = host;
        self->deinit = &machine_hw_spi_bus_deinit_internal;
        rp2_machine_spi_bus_objs[host] = self;
    } else {
        if ((int)mp_obj_get_int(self->mosi) != mosi) reconfigure = true;
        if ((int)mp_obj_get_int(self->miso) != miso) reconfigure = true;
        if ((int)mp_obj_get_int(self->sck) != sck) reconfigure = true;
    }

    if (reconfigure) {
        if (self->device_count > 0) {
            mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("SPI(%d) has active devices, unable to change pins"), host);
            return mp_const_none;
        }

        self->miso = mp_obj_new_int((mp_int_t)miso);
        self->mosi = mp_obj_new_int((mp_int_t)mosi);
        self->sck = mp_obj_new_int((mp_int_t)sck);
    }

    return MP_OBJ_FROM_PTR(self);
}


mp_obj_t machine_spi_device_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_id, ARG_baudrate, ARG_polarity, ARG_phase, ARG_bits, ARG_firstbit, ARG_cs, ARG_sck, ARG_mosi, ARG_miso };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id,       MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_baudrate, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = DEFAULT_SPI_BAUDRATE} },
        { MP_QSTR_polarity, MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = DEFAULT_SPI_POLARITY} },
        { MP_QSTR_phase,    MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = DEFAULT_SPI_PHASE} },
        { MP_QSTR_bits,     MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = DEFAULT_SPI_BITS} },
        { MP_QSTR_firstbit, MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = DEFAULT_SPI_FIRSTBIT} },
        { MP_QSTR_cs,       MP_ARG_KW_ONLY  | MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_sck,      MICROPY_SPI_PINS_ARG_OPTS | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_mosi,     MICROPY_SPI_PINS_ARG_OPTS | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_miso,     MICROPY_SPI_PINS_ARG_OPTS | MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none } },
    };

    // Parse the arguments.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Get the SPI bus id.
    int spi_id = mp_obj_get_int(args[ARG_id].u_obj);
    if (spi_id < 0 || spi_id >= MP_ARRAY_SIZE(rp2_machine_spi_bus_objs)) {
        mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("SPI(%d) doesn't exist"), spi_id);
    }

    mp_machine_hw_spi_device_obj_t *self = m_new_obj(mp_machine_hw_spi_device_obj_t);
    self->base.type = &machine_spi_type;

    mp_machine_hw_spi_bus_obj_t *spi_bus = &rp2_machine_spi_bus_objs[spi_id];
    if (spi_bus->sck == MP_OBJ_NULL) {
        if (spi_id == 0) {
            spi_bus->sck = mp_obj_new_int(MICROPY_HW_SPI0_SCK);
            spi_bus->mosi = mp_obj_new_int(MICROPY_HW_SPI0_MOSI);
            spi_bus->miso = mp_obj_new_int(MICROPY_HW_SPI0_MISO);
        } else {
            spi_bus->sck = mp_obj_new_int(MICROPY_HW_SPI1_SCK);
            spi_bus->mosi = mp_obj_new_int(MICROPY_HW_SPI1_MOSI);
            spi_bus->miso = mp_obj_new_int(MICROPY_HW_SPI1_MISO);
        }
    }

    self->spi_bus = spi_bus;
    self->freq = (uint32_t)args[ARG_baudrate].u_int;
    self->bits = (uint8_t)args[ARG_bits].u_int;
    self->polarity = (uint8_t)args[ARG_polarity].u_int;
    self->phase = (uint8_t)args[ARG_phase].u_int;
    self->firstbit = (uint8_t)args[ARG_firstbit].u_int;

    if (self->firstbit == SPI_LSB_FIRST) {
        mp_raise_NotImplementedError(MP_ERROR_TEXT("LSB"));
    }


    if (self->spi_bus->state == MP_SPI_STATE_STOPPED) {
        // Set SCK/MOSI/MISO pins if configured.

        spi_inst_t *const spi_inst = (spi_inst_t *const)self->spi_bus->user_data;

        if (args[ARG_sck].u_obj != mp_const_none) {
            int sck = (int)mp_obj_get_int(args[ARG_sck].u_obj);

            if (!IS_VALID_SCK(self->spi_bus->host, sck)) {
                mp_raise_ValueError(MP_ERROR_TEXT("bad SCK pin"));
            }

            self->spi_bus->sck = args[ARG_sck].u_obj;
        }

        if (args[ARG_mosi].u_obj != mp_const_none) {
            int mosi = (int)mp_obj_get_int(args[ARG_mosi].u_obj);

            if (!IS_VALID_MOSI(self->spi_bus->host, mosi)) {
                mp_raise_ValueError(MP_ERROR_TEXT("bad MOSI pin"));
            }
            self->spi_bus->mosi = args[ARG_mosi].u_obj;
        }

        if (args[ARG_miso].u_obj != mp_const_none) {
            int miso = (int)mp_obj_get_int(args[ARG_miso].u_obj);

            if (!IS_VALID_MISO(self->spi_bus->host, miso)) {
                mp_raise_ValueError(MP_ERROR_TEXT("bad MISO pin"));
            }
            self->spi_bus->miso = args[ARG_miso].u_obj;
        }

        spi_init(spi_inst, self->freq);
        gpio_set_function(mp_hal_get_pin_obj(self->spi_bus->sck), GPIO_FUNC_SPI);
        gpio_set_function(mp_hal_get_pin_obj(self->spi_bus->miso), GPIO_FUNC_SPI);
        gpio_set_function(mp_hal_get_pin_obj(self->spi_bus->mosi), GPIO_FUNC_SPI);

        self->spi_bus->state = MP_SPI_STATE_STARTED;
    }

    self->cs = args[ARG_cs].u_obj;
    if (args[ARG_cs].u_obj != mp_const_none) {
        mp_hal_pin_output(mp_hal_get_pin_obj(self->cs));
        mp_hal_pin_write(mp_hal_get_pin_obj(self->cs), 1);
    }

    self->active = true;
    self->spi_bus->device_count++;

    return MP_OBJ_FROM_PTR(self);
}

static void machine_spi_transfer(mp_obj_base_t *self_in, size_t len, const uint8_t *src, uint8_t *dest)
{
    mp_machine_hw_spi_device_obj_t *self = (mp_machine_hw_spi_device_obj_t *)self_in;

    if (self->spi_bus->state == MP_SPI_STATE_STOPPED) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("transfer on deinitialized SPI"));
        return;
    }

    if (!self->active) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("SPI Device is not longer active"));
        return;
    }

    while (self->spi_bus->state == MP_SPI_STATE_SENDING) {}

    self->spi_bus->state = MP_SPI_STATE_SENDING;

    spi_inst_t *const spi_inst = (spi_inst_t *const)self->spi_bus->user_data;

    spi_set_baudrate(spi_inst, self->freq);
    spi_set_format(spi_inst, self->bits, self->polarity, self->phase, self->firstbit);

    if (self->cs != mp_const_none) {
        mp_hal_pin_write(mp_hal_get_pin_obj(self->cs), 0);
    }

    // Use DMA for large transfers if channels are available
    const size_t dma_min_size_threshold = 32;
    int chan_tx = -1;
    int chan_rx = -1;
    if (len >= dma_min_size_threshold) {
        // Use two DMA channels to service the two FIFOs
        chan_tx = dma_claim_unused_channel(false);
        chan_rx = dma_claim_unused_channel(false);
    }
    bool use_dma = chan_rx >= 0 && chan_tx >= 0;
    // note src is guaranteed to be non-NULL
    bool write_only = dest == NULL;

    if (use_dma) {
        uint8_t dev_null;
        dma_channel_config c = dma_channel_get_default_config(chan_tx);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_dreq(&c, spi_get_index(spi_inst) ? DREQ_SPI1_TX : DREQ_SPI0_TX);
        dma_channel_configure(chan_tx, &c,
            &spi_get_hw(spi_inst)->dr,
            src,
            len,
            false);

        c = dma_channel_get_default_config(chan_rx);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_dreq(&c, spi_get_index(spi_inst) ? DREQ_SPI1_RX : DREQ_SPI0_RX);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, !write_only);
        dma_channel_configure(chan_rx, &c,
            write_only ? &dev_null : dest,
            &spi_get_hw(spi_inst)->dr,
            len,
            false);

        dma_start_channel_mask((1u << chan_rx) | (1u << chan_tx));
        dma_channel_wait_for_finish_blocking(chan_rx);
        dma_channel_wait_for_finish_blocking(chan_tx);
    }

    // If we have claimed only one channel successfully, we should release immediately
    if (chan_rx >= 0) {
        dma_channel_unclaim(chan_rx);
    }
    if (chan_tx >= 0) {
        dma_channel_unclaim(chan_tx);
    }

    if (!use_dma) {
        // Use software for small transfers, or if couldn't claim two DMA channels
        if (write_only) {
            spi_write_blocking(spi_inst, src, len);
        } else {
            spi_write_read_blocking(spi_inst, src, dest, len);
        }
    }

    if (self->cs != mp_const_none) {
        mp_hal_pin_write(mp_hal_get_pin_obj(self->cs), 1);
    }

    self->spi_bus->state = MP_SPI_STATE_STARTED;

}


// Buffer protocol implementation for SPI.
// The buffer represents the SPI data FIFO.
static mp_int_t machine_spi_get_buffer(mp_obj_t o_in, mp_buffer_info_t *bufinfo, mp_uint_t flags) {
    mp_machine_hw_spi_device_obj_t *self = MP_OBJ_TO_PTR(o_in);
    spi_inst_t *const spi_inst = (spi_inst_t *const)self->spi_bus->user_data;

    bufinfo->len = 4;
    bufinfo->typecode = 'I';
    bufinfo->buf = (void *)&spi_get_hw(spi_inst)->dr;

    return 0;
}


static const mp_machine_spi_p_t machine_spi_p = {
    .init = machine_spi_init,
    .transfer = machine_spi_transfer,
    .deinit = machine_spi_deinit
};


MP_DEFINE_CONST_OBJ_TYPE(
    machine_spi_type,
    MP_QSTR_SPI,
    MP_TYPE_FLAG_NONE,
    make_new, machine_spi_make_new,
    protocol, &machine_spi_p,
    buffer, machine_spi_get_buffer,
    locals_dict, &mp_machine_spi_locals_dict
    );


mp_obj_base_t *mp_hal_get_spi_obj(mp_obj_t o) {
    if (mp_obj_is_type(o, &machine_spi_type)) {
        return MP_OBJ_TO_PTR(o);
    }
    #if MICROPY_PY_MACHINE_SOFTSPI
    else if (mp_obj_is_type(o, &mp_machine_soft_spi_type)) {
        return MP_OBJ_TO_PTR(o);
    }
    #endif
    else {
        mp_raise_TypeError(MP_ERROR_TEXT("expecting an SPI object"));
    }
}
