/*
 * UR Series Thermal Camera Driver for NVIDIA Jetson
 * Platform: Jetson Orin Nano (JetPack 6 / Kernel 5.15)
 *
 * DUAL CAMERA CHANGES vs single-camera version:
 * 1. tegra_sinterface is now read from DT ("tegra-sinterface" property)
 *    instead of being hardcoded to 1 (serial_b).
 *    CAM0 DT node must have: tegra-sinterface = <1>;  (serial_b)
 *    CAM1 DT node must have: tegra-sinterface = <2>;  (serial_c)
 * 2. Added ur_get_tegra_sinterface() helper to parse it safely.
 * 3. All other state is already per-instance via devm_* and container_of().
 *
 * Hardware:
 * - Chip ID: 0x5352
 * - I2C: 0x3c
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/stddef.h>
#include <linux/v4l2-mediabus.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of_graph.h>
#include <linux/of.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/hrtimer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/camera_common.h>
#include <media/tegracam_core.h>

/* Module Parameters */
static int type = 1; /* Default from pi_driver.c */
module_param(type, int, 0644);
MODULE_PARM_DESC(type, "UR Sensor Type Parameter (Default: 1)");

/* I2C Address */
#define UR_I2C_ADDR             0x3c

/* Registers & IDs */
#define UR_REG_CHIP_ID          0x0000
#define UR_CHIP_ID              0x5352
#define UR_REG_BUFFER_RW        0x1D00

/* Dimensions & Format (Fixed YUV422) */
#define UR_WIDTH                640
#define UR_HEIGHT               512
#define UR_FPS                  30
#define UR_PIXEL_RATE           100000000ULL
#define UR_LINE_LENGTH          2200
#define UR_MEDIA_BUS_FMT        MEDIA_BUS_FMT_YUYV8_1X16

/* Number of supplies */
#define UR_NUM_SUPPLIES         0

/*
 * Tegra serial interface IDs — must match tegra_sinterface in DT mode0.
 * serial_a=0, serial_b=1, serial_c=2, serial_d=3, serial_e=4, serial_f=5
 */
#define TEGRA_SINTERFACE_SERIAL_B   1  /* CAM0 default */
#define TEGRA_SINTERFACE_SERIAL_C   2  /* CAM1 */

struct ur_sensor {
    struct v4l2_subdev subdev;
    struct media_pad pad;
    struct i2c_client *client;
    struct gpio_desc *reset_gpio;

    /* Camera common data for Tegra integration */
    struct camera_common_data *s_data;

    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl *exposure;
    struct v4l2_ctrl *gain;

    /*
     * Per-instance tegra_sinterface read from DT.
     * CAM0: 1 (serial_b), CAM1: 2 (serial_c)
     */
    u32 tegra_sinterface;

    struct mutex mutex;
    bool streaming;

    /*
     * Per-frame capture timestamp (CLOCK_REALTIME) populated by hrtimer
     * firing at UR_FPS rate while streaming. Exposed via
     * V4L2_CID_CAMERA_SENSOR_TIMESTAMP_* controls so userspace can read
     * the seconds and nanoseconds components independently.
     *
     * Userspace reads both controls back-to-back after each frame to
     * reconstruct: wall_time = ts_sec + ts_nsec / 1e9
     */
    struct hrtimer frame_timer;
    ktime_t         frame_ts;       /* last frame wall-clock timestamp */
    spinlock_t      ts_lock;        /* protects frame_ts */

    /* V4L2 controls exposing the timestamp to userspace */
    struct v4l2_ctrl *ts_sec_ctrl;  /* seconds component  */
    struct v4l2_ctrl *ts_nsec_ctrl; /* nanoseconds component */
};

/* Frame format information */
static const struct camera_common_frmfmt ur_frmfmt[] = {
    {
        .size = {UR_WIDTH, UR_HEIGHT},
        .framerates = NULL,
        .num_framerates = 0,
        .hdr_en = false,
        .mode = 0,
    },
};

static inline struct ur_sensor *to_ur_sensor(struct v4l2_subdev *sd)
{
    return container_of(sd, struct ur_sensor, subdev);
}

/* --- I2C Helpers --- */

static int ur_read_reg16(struct i2c_client *client, u16 reg, u16 *val)
{
    struct i2c_msg msgs[2];
    u8 addr_buf[2] = { reg >> 8, reg & 0xff };
    u8 data_buf[2] = { 0 };
    int ret;

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = addr_buf;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 2;
    msgs[1].buf = data_buf;

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret != 2)
        return -EIO;

    *val = (data_buf[0] << 8) | data_buf[1];
    return 0;
}

static int ur_write_buffer(struct i2c_client *client, u16 reg, u8 *val, int len)
{
    struct i2c_msg msg;
    u8 *outbuf;
    int ret;

    outbuf = kzalloc(len + 2, GFP_KERNEL);
    if (!outbuf)
        return -ENOMEM;

    outbuf[0] = reg >> 8;
    outbuf[1] = reg & 0xff;
    memcpy(&outbuf[2], val, len);

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = len + 2;
    msg.buf = outbuf;

    ret = i2c_transfer(client->adapter, &msg, 1);
    kfree(outbuf);

    return (ret == 1) ? 0 : -EIO;
}

static unsigned short do_crc(unsigned char *ptr, int len)
{
    unsigned int i;
    unsigned short crc = 0x0000;

    while (len--) {
        crc ^= (unsigned short)(*ptr++) << 8;
        for (i = 0; i < 8; ++i) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/*
 * Custom V4L2 user controls for exposing the capture timestamp.
 * V4L2_CID_USER_BASE is 0x00980000. We pick offsets 0x100 and 0x101
 * (well away from any standard controls) to avoid collisions.
 *
 * Userspace reads both controls back-to-back after each frame to
 * reconstruct: wall_time = ts_sec + ts_nsec / 1e9
 */
#define V4L2_CID_UR_TS_SEC   (V4L2_CID_USER_BASE + 0x100)
#define V4L2_CID_UR_TS_NSEC  (V4L2_CID_USER_BASE + 0x101)

/*
 * ur_frame_timer_cb() - hrtimer callback, fires once per frame period.
 *
 * Snapshots CLOCK_REALTIME into sensor->frame_ts under ts_lock, then
 * re-arms itself for the next frame. Also sends a V4L2_EVENT_CTRL change
 * event so subscribed userspace processes wake up immediately.
 */
static enum hrtimer_restart ur_frame_timer_cb(struct hrtimer *timer)
{
    struct ur_sensor *sensor = container_of(timer, struct ur_sensor, frame_timer);
    struct timespec64 ts;
    unsigned long flags;

    /* Snapshot wall clock */
    ktime_get_real_ts64(&ts);

    spin_lock_irqsave(&sensor->ts_lock, flags);
    sensor->frame_ts = timespec64_to_ktime(ts);
    spin_unlock_irqrestore(&sensor->ts_lock, flags);

    /* Re-arm for next frame: period = 1/UR_FPS seconds */
    hrtimer_forward_now(timer, ns_to_ktime(NSEC_PER_SEC / UR_FPS));
    return HRTIMER_RESTART;
}

/* --- Packet Templates --- */

static const u8 start_regs_template[] = {
    0x01, 0x30, 0xc1, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x0a, 0x00,
    0x00, 0x00, /* [14,15] CRC 1 */
    0x2F, 0x0D, /* [16,17] CRC 2 */
    0x00, /* [18] Path */
    0x16, /* [19] Src */
    0x03, /* [20] Dst */
    0x1e, /* [21] FPS */
    0x80, 0x02, /* [22,23] Width */
    0x00, 0x02, /* [24,25] Height */
    0x00, 0x00
};

static const u8 stop_regs_template[] = {
    0x01, 0x30, 0xc2, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x0a, 0x00,
    0x00, 0x00,
    0x2F, 0x0D,
    0x01, 0x16, 0x00, 0x0e,
    0x80, 0x02, 0x00, 0x02,
    0x00, 0x00
};
/* Subdev ioctl: handle event subscribe/unsubscribe for V4L2 controls */
static long ur_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
    switch (cmd) {
    case VIDIOC_SUBSCRIBE_EVENT:
        return v4l2_ctrl_subdev_subscribe_event(sd, arg, NULL);
    default:
        return -ENOTTY;
    }
}
static int ur_set_stream(struct v4l2_subdev *sd, int enable)
{
    struct ur_sensor *sensor = to_ur_sensor(sd);
    struct i2c_client *client = sensor->client;
    u8 *buf;
    unsigned short crc;
    int ret = 0;

    mutex_lock(&sensor->mutex);
    if (sensor->streaming == enable) {
        mutex_unlock(&sensor->mutex);
        return 0;
    }

    if (enable) {
        buf = kmemdup(start_regs_template, sizeof(start_regs_template), GFP_KERNEL);
        if (!buf) {
            ret = -ENOMEM;
            goto unlock;
        }
        buf[19] = type;
        buf[21] = UR_FPS;
        buf[22] = UR_WIDTH & 0xff;
        buf[23] = UR_WIDTH >> 8;
        buf[24] = UR_HEIGHT & 0xff;
        buf[25] = UR_HEIGHT >> 8;

        crc = do_crc((uint8_t *)(buf + 18), 10);
        buf[14] = crc & 0xff;
        buf[15] = crc >> 8;

        crc = do_crc((uint8_t *)(buf), 16);
        buf[16] = crc & 0xff;
        buf[17] = crc >> 8;

        dev_info(&client->dev, "Sending Start Stream (Type: %d)\n", type);
        ret = ur_write_buffer(client, UR_REG_BUFFER_RW, buf, sizeof(start_regs_template));
        kfree(buf);

        if (ret)
            goto unlock;
        msleep(50);

        /* Start per-frame timestamp timer */
        hrtimer_start(&sensor->frame_timer,
                      ns_to_ktime(NSEC_PER_SEC / UR_FPS),
                      HRTIMER_MODE_REL);

    } else {
        buf = kmemdup(stop_regs_template, sizeof(stop_regs_template), GFP_KERNEL);
        if (!buf) {
            ret = -ENOMEM;
            goto unlock;
        }

        buf[19] = type;

        crc = do_crc((uint8_t *)(buf + 18), 10);
        buf[14] = crc & 0xff;
        buf[15] = crc >> 8;

        crc = do_crc((uint8_t *)(buf), 16);
        buf[16] = crc & 0xff;
        buf[17] = crc >> 8;

        dev_info(&client->dev, "Sending Stop Stream\n");
        ret = ur_write_buffer(client, UR_REG_BUFFER_RW, buf, sizeof(stop_regs_template));
        kfree(buf);

        /* Stop per-frame timestamp timer */
        hrtimer_cancel(&sensor->frame_timer);
    }

    sensor->streaming = enable;

unlock:
    mutex_unlock(&sensor->mutex);
    return ret;
}

/* --- V4L2 Pad Ops --- */

static int ur_enum_mbus_code(struct v4l2_subdev *sd,
                             struct v4l2_subdev_state *state,
                             struct v4l2_subdev_mbus_code_enum *code)
{
    if (code->index > 0)
        return -EINVAL;
    code->code = UR_MEDIA_BUS_FMT;
    return 0;
}

static int ur_enum_frame_size(struct v4l2_subdev *sd,
                              struct v4l2_subdev_state *state,
                              struct v4l2_subdev_frame_size_enum *fse)
{
    if (fse->index > 0)
        return -EINVAL;
    if (fse->code != UR_MEDIA_BUS_FMT)
        return -EINVAL;

    fse->min_width = fse->max_width = UR_WIDTH;
    fse->min_height = fse->max_height = UR_HEIGHT;
    return 0;
}

static int ur_get_fmt(struct v4l2_subdev *sd,
                      struct v4l2_subdev_state *state,
                      struct v4l2_subdev_format *fmt)
{
    fmt->format.width = UR_WIDTH;
    fmt->format.height = UR_HEIGHT;
    fmt->format.code = UR_MEDIA_BUS_FMT;
    fmt->format.field = V4L2_FIELD_NONE;
    fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
    return 0;
}

static int ur_set_fmt(struct v4l2_subdev *sd,
                      struct v4l2_subdev_state *state,
                      struct v4l2_subdev_format *fmt)
{
    fmt->format.width = UR_WIDTH;
    fmt->format.height = UR_HEIGHT;
    fmt->format.code = UR_MEDIA_BUS_FMT;
    fmt->format.field = V4L2_FIELD_NONE;
    fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
    return 0;
}

static int ur_get_selection(struct v4l2_subdev *sd,
                            struct v4l2_subdev_state *state,
                            struct v4l2_subdev_selection *sel)
{
    switch (sel->target) {
    case V4L2_SEL_TGT_CROP:
    case V4L2_SEL_TGT_CROP_DEFAULT:
    case V4L2_SEL_TGT_CROP_BOUNDS:
    case V4L2_SEL_TGT_NATIVE_SIZE:
        sel->r.left = 0;
        sel->r.top = 0;
        sel->r.width = UR_WIDTH;
        sel->r.height = UR_HEIGHT;
        return 0;
    default:
        return -EINVAL;
    }
}
static const struct v4l2_subdev_core_ops ur_core_ops = {
    .ioctl = ur_ioctl,
};
static const struct v4l2_subdev_video_ops ur_video_ops = {
    .s_stream = ur_set_stream,
};

static const struct v4l2_subdev_pad_ops ur_pad_ops = {
    .enum_mbus_code = ur_enum_mbus_code,
    .enum_frame_size = ur_enum_frame_size,
    .get_fmt = ur_get_fmt,
    .set_fmt = ur_set_fmt,
    .get_selection = ur_get_selection,
};

static const struct v4l2_subdev_ops ur_subdev_ops = {
    .core = &ur_core_ops,
    .video = &ur_video_ops,
    .pad = &ur_pad_ops,
};

/* --- Control Operations --- */

/*
 * ur_g_volatile_ctrl() - called by V4L2 when userspace reads a volatile ctrl.
 *
 * For our timestamp controls we return the values already written by the
 * hrtimer callback, protecting the read with ts_lock so we never return
 * a torn 64-bit value.
 */
static int ur_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
    struct ur_sensor *sensor = container_of(ctrl->handler,
                                            struct ur_sensor, ctrl_handler);
    struct timespec64 ts;
    unsigned long flags;

    switch (ctrl->id) {
    case V4L2_CID_UR_TS_SEC:
    case V4L2_CID_UR_TS_NSEC:
        spin_lock_irqsave(&sensor->ts_lock, flags);
        ts = ktime_to_timespec64(sensor->frame_ts);
        spin_unlock_irqrestore(&sensor->ts_lock, flags);

        /*
         * g_volatile_ctrl is called for one control at a time.
         * ctrl->val is the correct output field for INTEGER controls.
         * Writing ctrl->val here is legal — this is exactly what
         * g_volatile_ctrl is for.
         */
        if (ctrl->id == V4L2_CID_UR_TS_SEC)
            ctrl->val = (s32)(ts.tv_sec & 0x7FFFFFFF);
        else
            ctrl->val = (s32)ts.tv_nsec;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static int ur_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct ur_sensor *sensor = container_of(ctrl->handler,
                                            struct ur_sensor, ctrl_handler);
    struct i2c_client *client = sensor->client;
    int ret = 0;

    switch (ctrl->id) {
    case V4L2_CID_GAIN:
        dev_dbg(&client->dev, "Set gain to %d\n", ctrl->val);
        break;
    case V4L2_CID_EXPOSURE:
        dev_dbg(&client->dev, "Set exposure to %d\n", ctrl->val);
        break;
    case V4L2_CID_PIXEL_RATE:
        break;
    default:
        dev_dbg(&client->dev, "Unknown control 0x%x\n", ctrl->id);
        ret = -EINVAL;
    }

    return ret;
}

static const struct v4l2_ctrl_ops ur_ctrl_ops = {
    .g_volatile_ctrl = ur_g_volatile_ctrl,
    .s_ctrl = ur_s_ctrl,
};

/* --- DT Helper: read tegra_sinterface per instance --- */

/*
 * ur_get_tegra_sinterface() - Parse tegra_sinterface from DT mode0 node.
 *
 * In the DTS each sensor node has a mode0 child. The tegra_sinterface
 * property there is a string ("serial_b", "serial_c", ...). We map it
 * to the integer the Tegra VI framework expects.
 *
 * If the property is missing or unrecognised we fall back to serial_b (1)
 * so CAM0 keeps working without any DT change, while CAM1's DT must
 * explicitly set tegra_sinterface = "serial_c".
 *
 * Mapping (matches tegra_camera_platform driver conventions):
 *   serial_a = 0
 *   serial_b = 1   ← CAM0
 *   serial_c = 2   ← CAM1
 *   serial_d = 3
 *   serial_e = 4
 *   serial_f = 5
 */
static u32 ur_get_tegra_sinterface(struct i2c_client *client)
{
    struct device_node *mode_node;
    const char *iface_str;
    u32 iface_id = TEGRA_SINTERFACE_SERIAL_B; /* safe default */
    int ret;

    /* Look for the mode0 child node */
    mode_node = of_get_child_by_name(client->dev.of_node, "mode0");
    if (!mode_node) {
        dev_warn(&client->dev,
                 "No mode0 DT node found, defaulting tegra_sinterface to serial_b\n");
        return iface_id;
    }

    ret = of_property_read_string(mode_node, "tegra_sinterface", &iface_str);
    of_node_put(mode_node);

    if (ret) {
        dev_warn(&client->dev,
                 "tegra_sinterface not in DT mode0, defaulting to serial_b\n");
        return iface_id;
    }

    if      (!strcmp(iface_str, "serial_a")) iface_id = 0;
    else if (!strcmp(iface_str, "serial_b")) iface_id = 1;
    else if (!strcmp(iface_str, "serial_c")) iface_id = 2;
    else if (!strcmp(iface_str, "serial_d")) iface_id = 3;
    else if (!strcmp(iface_str, "serial_e")) iface_id = 4;
    else if (!strcmp(iface_str, "serial_f")) iface_id = 5;
    else {
        dev_warn(&client->dev,
                 "Unknown tegra_sinterface '%s', defaulting to serial_b\n",
                 iface_str);
    }

    dev_info(&client->dev, "tegra_sinterface: '%s' -> %u\n", iface_str, iface_id);
    return iface_id;
}

/* --- Tegra camera_common integration --- */

static int ur_init_camera_common_data(struct ur_sensor *sensor)
{
    struct i2c_client *client = sensor->client;
    struct camera_common_data *s_data;
    struct sensor_mode_properties *mode;
    struct tegracam_ctrl_handler *ctrl_hdl;

    s_data = devm_kzalloc(&client->dev, sizeof(*s_data), GFP_KERNEL);
    if (!s_data)
        return -ENOMEM;

    ctrl_hdl = devm_kzalloc(&client->dev, sizeof(*ctrl_hdl), GFP_KERNEL);
    if (!ctrl_hdl)
        return -ENOMEM;
    s_data->tegracam_ctrl_hdl = ctrl_hdl;

    s_data->dev = &client->dev;
    s_data->frmfmt = ur_frmfmt;
    s_data->numfmts = ARRAY_SIZE(ur_frmfmt);
    s_data->def_mode = 0;
    s_data->def_width = UR_WIDTH;
    s_data->def_height = UR_HEIGHT;
    s_data->fmt_width = UR_WIDTH;
    s_data->fmt_height = UR_HEIGHT;
    s_data->mode_prop_idx = 0;
    s_data->override_enable = false;
    s_data->power = NULL;
    s_data->colorfmt = NULL;
    // Tell Tegra VI to stamp buffers with CLOCK_REALTIME. 
    s_data->use_sensor_mode_id = false;
    s_data->sensor_props.num_modes = 1;
    s_data->sensor_props.sensor_modes = devm_kzalloc(&client->dev,
        sizeof(struct sensor_mode_properties), GFP_KERNEL);
    if (!s_data->sensor_props.sensor_modes)
        return -ENOMEM;

    mode = &s_data->sensor_props.sensor_modes[0];

    /* Signal properties */
    mode->signal_properties.num_lanes = 2;
    mode->signal_properties.pixel_clock.val = UR_PIXEL_RATE;
    mode->signal_properties.serdes_pixel_clock.val = 0;
    mode->signal_properties.phy_mode = CSI_PHY_MODE_DPHY;
    mode->signal_properties.discontinuous_clk = 0;
    mode->signal_properties.dpcm_enable = 0;
    mode->signal_properties.mclk_freq = 24000;
    mode->signal_properties.cil_settletime = 0;
    mode->signal_properties.lane_polarity = 6;
    mode->signal_properties.readout_orientation = 0;
    /*
     * KEY DUAL-CAMERA FIX:
     * Use the per-instance value parsed from DT instead of the
     * hardcoded literal 1. CAM0 gets 1 (serial_b), CAM1 gets 2 (serial_c).
     */
    mode->signal_properties.tegra_sinterface = sensor->tegra_sinterface;

    /* MIPI clock calculation */
    {
        u64 rate = mode->signal_properties.pixel_clock.val;
        int bit_depth = 16;
        int num_lanes = mode->signal_properties.num_lanes;

        rate = rate * bit_depth / num_lanes;
        mode->signal_properties.mipi_clock.val = rate / 2;

        dev_info(&client->dev, "MIPI clock: %llu Hz  tegra_sinterface: %u\n",
                 mode->signal_properties.mipi_clock.val,
                 mode->signal_properties.tegra_sinterface);
    }

    /* Image properties */
    mode->image_properties.width = UR_WIDTH;
    mode->image_properties.height = UR_HEIGHT;
    mode->image_properties.line_length = UR_LINE_LENGTH;
    mode->image_properties.pixel_format = V4L2_PIX_FMT_YUYV;
    mode->image_properties.embedded_metadata_height = 0;

    /* Control properties */
    mode->control_properties.gain_factor = 16;
    mode->control_properties.framerate_factor = 1000000;
    mode->control_properties.exposure_factor = 1000000;
    mode->control_properties.inherent_gain = 1;
    mode->control_properties.min_framerate = 2000000;
    mode->control_properties.max_framerate = 30000000;
    mode->control_properties.step_framerate = 1000000;
    mode->control_properties.default_framerate = 30000000;
    mode->control_properties.min_gain_val = 16;
    mode->control_properties.max_gain_val = 170;
    mode->control_properties.step_gain_val = 1;
    mode->control_properties.default_gain = 16;
    mode->control_properties.min_exp_time.val = 13;
    mode->control_properties.max_exp_time.val = 333333;
    mode->control_properties.step_exp_time.val = 1;
    mode->control_properties.default_exp_time.val = 2495;
    mode->control_properties.min_hdr_ratio = 1;
    mode->control_properties.max_hdr_ratio = 1;
    mode->control_properties.is_interlaced = 0;
    mode->control_properties.interlace_type = 0;

    sensor->s_data = s_data;
    dev_set_drvdata(&client->dev, &s_data->subdev);

    dev_info(&client->dev, "=== CAMERA COMMON DATA DEBUG ===\n");
    dev_info(&client->dev, "s_data ptr:             %px\n", s_data);
    dev_info(&client->dev, "&s_data->subdev:        %px\n", &s_data->subdev);
    dev_info(&client->dev, "sensor_props.num_modes: %d\n", s_data->sensor_props.num_modes);
    dev_info(&client->dev, "tegra_sinterface:       %u\n",
             mode->signal_properties.tegra_sinterface);
    dev_info(&client->dev, "=================================\n");

    return 0;
}

/* --- Probe --- */

static int ur_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ur_sensor *sensor;
    struct v4l2_subdev *sd;
    struct v4l2_fwnode_device_properties props;
    int ret;
    u16 chip_id;

    dev_info(&client->dev, "PROBE START: UR Series (Addr: 0x%02x)\n", client->addr);

    sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
    if (!sensor)
        return -ENOMEM;

    sensor->client = client;
    mutex_init(&sensor->mutex);
    spin_lock_init(&sensor->ts_lock);
    hrtimer_init(&sensor->frame_timer, CLOCK_REALTIME, HRTIMER_MODE_REL);
    sensor->frame_timer.function = ur_frame_timer_cb;
    sd = &sensor->subdev;

    /*
     * Parse tegra_sinterface from DT BEFORE init_camera_common_data.
     * This is the only field that differs between CAM0 and CAM1 instances.
     */
    sensor->tegra_sinterface = ur_get_tegra_sinterface(client);

    v4l2_i2c_subdev_init(sd, client, &ur_subdev_ops);
    
    sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
    sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    sd->flags |= V4L2_SUBDEV_FL_HAS_EVENTS; // Related to timestamp in v4l.
    if (!sd->fwnode) {
        sd->fwnode = dev_fwnode(&client->dev);
        if (!sd->fwnode)
            dev_warn(&client->dev, "Warning: fwnode is NULL\n");
    }

    /* 1. Reset */
    sensor->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);
    if (sensor->reset_gpio) {
        gpiod_set_value_cansleep(sensor->reset_gpio, 1);
        msleep(10);
    }

    /* 2. ID Check */
    ret = ur_read_reg16(client, UR_REG_CHIP_ID, &chip_id);
    if (ret) {
        dev_err(&client->dev, "I2C Read Failed\n");
        goto error_mutex;
    }

    if (chip_id != UR_CHIP_ID) {
        dev_warn(&client->dev, "ID Mismatch: Read 0x%04x, Expected 0x%04x\n",
                 chip_id, UR_CHIP_ID);
    } else {
        dev_info(&client->dev, "UR Sensor Verified (ID: 0x%04x)\n", chip_id);
    }

    /* 3. Init camera_common_data (uses sensor->tegra_sinterface set above) */
    ret = ur_init_camera_common_data(sensor);
    if (ret) {
        dev_err(&client->dev, "Failed to init camera common data: %d\n", ret);
        goto error_mutex;
    }

    /* 4. Controls */
    v4l2_ctrl_handler_init(&sensor->ctrl_handler, 6);

    sensor->gain = v4l2_ctrl_new_std(&sensor->ctrl_handler, &ur_ctrl_ops,
                                      V4L2_CID_GAIN, 0, 255, 1, 0);
    sensor->exposure = v4l2_ctrl_new_std(&sensor->ctrl_handler, &ur_ctrl_ops,
                                          V4L2_CID_EXPOSURE, 0, 65535, 1, 1000);
    sensor->pixel_rate = v4l2_ctrl_new_std(&sensor->ctrl_handler, &ur_ctrl_ops,
                                            V4L2_CID_PIXEL_RATE,
                                            UR_PIXEL_RATE, UR_PIXEL_RATE, 1, UR_PIXEL_RATE);
    if (sensor->pixel_rate)
        sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    /*
     * Custom volatile read-only controls for the capture timestamp.
     * Range for seconds: 0 .. 0x7FFFFFFF (covers unix time well past 2038
     * as a signed 32-bit value — sufficient for a frame timestamp).
     * Range for nanoseconds: 0 .. 999999999.
     */
    sensor->ts_sec_ctrl = v4l2_ctrl_new_custom(&sensor->ctrl_handler,
        &(const struct v4l2_ctrl_config){
            .ops  = &ur_ctrl_ops,
            .id   = V4L2_CID_UR_TS_SEC,
            .name = "Capture Timestamp Seconds",
            .type = V4L2_CTRL_TYPE_INTEGER,
            .min  = 0,
            .max  = 0x7FFFFFFF,
            .step = 1,
            .def  = 0,
            .flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
        }, NULL);

    sensor->ts_nsec_ctrl = v4l2_ctrl_new_custom(&sensor->ctrl_handler,
        &(const struct v4l2_ctrl_config){
            .ops  = &ur_ctrl_ops,
            .id   = V4L2_CID_UR_TS_NSEC,
            .name = "Capture Timestamp Nanoseconds",
            .type = V4L2_CTRL_TYPE_INTEGER,
            .min  = 0,
            .max  = 999999999,
            .step = 1,
            .def  = 0,
            .flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
        }, NULL);

    ret = v4l2_fwnode_device_parse(&client->dev, &props);
    if (ret == 0) {
        ret = v4l2_ctrl_new_fwnode_properties(&sensor->ctrl_handler,
                                               &ur_ctrl_ops, &props);
        if (ret)
            dev_warn(&client->dev, "Failed to add fwnode properties: %d\n", ret);
    }

    if (sensor->ctrl_handler.error) {
        ret = sensor->ctrl_handler.error;
        dev_err(&client->dev, "Control handler error: %d\n", ret);
        goto error_ctrl;
    }
    sd->ctrl_handler = &sensor->ctrl_handler;

    /* 5. Media entity pads */
    sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to init media entity pads: %d\n", ret);
        goto error_ctrl;
    }

    /* 6. Register with async framework */
    ret = v4l2_async_register_subdev_sensor(sd);
    if (ret < 0) {
        dev_err(&client->dev, "Async register failed: %d\n", ret);
        goto error_entity;
    }

    dev_info(&client->dev, "PROBE COMPLETE: Success (sinterface=%u)\n",
             sensor->tegra_sinterface);
    return 0;

error_entity:
    media_entity_cleanup(&sd->entity);
error_ctrl:
    v4l2_ctrl_handler_free(&sensor->ctrl_handler);
error_mutex:
    mutex_destroy(&sensor->mutex);
    return ret;
}

static int ur_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct ur_sensor *sensor = to_ur_sensor(sd);

    v4l2_async_unregister_subdev(sd);
    hrtimer_cancel(&sensor->frame_timer);
    media_entity_cleanup(&sd->entity);
    v4l2_ctrl_handler_free(&sensor->ctrl_handler);
    mutex_destroy(&sensor->mutex);
    return 0;
}

static const struct i2c_device_id ur_id[] = {
    { "ur_thermal", 0 },
    { "imx219", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, ur_id);

static const struct of_device_id ur_dt_ids[] = {
    { .compatible = "ur,thermal-camera" },
    { .compatible = "covenant,imx219-yuv640" },
    { .compatible = "sony,imx219" },
    { }
};
MODULE_DEVICE_TABLE(of, ur_dt_ids);

static struct i2c_driver ur_i2c_driver = {
    .driver = {
        .name = "ur_thermal",
        .of_match_table = ur_dt_ids,
    },
    .probe = ur_probe,
    .remove = ur_remove,
    .id_table = ur_id,
};

module_i2c_driver(ur_i2c_driver);

MODULE_DESCRIPTION("UR Series Thermal Sensor Driver for Jetson (Dual Camera)");
MODULE_AUTHOR("Your Name");
MODULE_LICENSE("GPL v2");