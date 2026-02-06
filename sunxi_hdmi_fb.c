/*
 * Sunxi HDMI and Framebuffer Control Utility
 *
 * Unified utility for Allwinner SoCs supporting both:
 * - A20/A10 (sun7i/sun4i) - Display Engine 1.0 (DE1)
 * - H3/H5/A64 (sun8iw7/sun50iw1) - Display Engine 2.0 (DE2)
 *
 * Auto-detects the display driver version and uses appropriate ioctls.
 *
 * Based on analysis of:
 * - THEC64 Mini firmware
 * - Linux 3.4 kernel source for sun7i (A20) display driver
 * - Linux 3.4 kernel source for sun8iw7 (H3) display driver
 *
 * Compile with:
 *   arm-linux-gnueabihf-gcc -o sunxi_hdmi_fb sunxi_hdmi_fb.c
 *
 * Copyright (c) 2024
 * License: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <signal.h>
#include <linux/fb.h>
#include <linux/types.h>
#include <stdbool.h>

/* Device paths */
#define DISP_DEV    "/dev/disp"
#define FB_DEV      "/dev/fb0"
#define HDMI_STATE  "/sys/class/switch/hdmi/state"
#define CPUINFO     "/proc/cpuinfo"

/*
 * ============================================================================
 * Display Engine Version Detection
 * ============================================================================
 */
typedef enum {
    DE_VERSION_UNKNOWN = 0,
    DE_VERSION_1,   /* A10/A20 (sun4i/sun7i) - Display Engine 1.0 */
    DE_VERSION_2,   /* H3/H5/A64 (sun8iw7/sun50iw1) - Display Engine 2.0 */
} de_version_t;

static de_version_t g_de_version = DE_VERSION_UNKNOWN;

/*
 * ============================================================================
 * Common Type Definitions
 * ============================================================================
 */
typedef signed char __bool;

/* Common structures used by both DE1 and DE2 */
typedef struct { __s32 x; __s32 y; __u32 width; __u32 height; } disp_rect;
typedef struct { __u32 width; __u32 height; } disp_rectsz;
typedef struct { __u8 alpha; __u8 red; __u8 green; __u8 blue; } disp_color;

/* TV/HDMI modes - common between DE1 and DE2 */
typedef enum {
    DISP_TV_MOD_480I            = 0,
    DISP_TV_MOD_576I            = 1,
    DISP_TV_MOD_480P            = 2,
    DISP_TV_MOD_576P            = 3,
    DISP_TV_MOD_720P_50HZ       = 4,
    DISP_TV_MOD_720P_60HZ       = 5,
    DISP_TV_MOD_1080I_50HZ      = 6,
    DISP_TV_MOD_1080I_60HZ      = 7,
    DISP_TV_MOD_1080P_24HZ      = 8,
    DISP_TV_MOD_1080P_50HZ      = 9,
    DISP_TV_MOD_1080P_60HZ      = 0xa,
    DISP_TV_MOD_PAL             = 0xb,
    DISP_TV_MOD_PAL_SVIDEO      = 0xc,
    DISP_TV_MOD_NTSC            = 0xe,
    DISP_TV_MOD_NTSC_SVIDEO     = 0xf,
    DISP_TV_MOD_PAL_M           = 0x11,
    DISP_TV_MOD_PAL_M_SVIDEO    = 0x12,
    DISP_TV_MOD_PAL_NC          = 0x14,
    DISP_TV_MOD_PAL_NC_SVIDEO   = 0x15,
    DISP_TV_MOD_1080P_24HZ_3D_FP = 0x17,
    DISP_TV_MOD_720P_50HZ_3D_FP = 0x18,
    DISP_TV_MOD_720P_60HZ_3D_FP = 0x19,
    DISP_TV_MOD_1080P_25HZ      = 0x1a,
    DISP_TV_MOD_1080P_30HZ      = 0x1b,
    DISP_TV_MOD_3840_2160P_30HZ = 0x1c,  /* H3+ only */
    DISP_TV_MOD_3840_2160P_25HZ = 0x1d,  /* H3+ only */
    DISP_TV_MOD_3840_2160P_24HZ = 0x1e,  /* H3+ only */
    DISP_TV_MODE_NUM            = 0x1f,
} disp_tv_mode;

/* Output types */
typedef enum {
    DISP_OUTPUT_TYPE_NONE   = 0,
    DISP_OUTPUT_TYPE_LCD    = 1,
    DISP_OUTPUT_TYPE_TV     = 2,
    DISP_OUTPUT_TYPE_HDMI   = 4,
    DISP_OUTPUT_TYPE_VGA    = 8,
} disp_output_type;

/*
 * ============================================================================
 * DE1 (A20) Specific Definitions
 * ============================================================================
 */

/* DE1 ioctl commands */
#define DE1_CMD_SCN_GET_WIDTH       0x08
#define DE1_CMD_SCN_GET_HEIGHT      0x09
#define DE1_CMD_GET_OUTPUT_TYPE     0x0a
#define DE1_CMD_SET_SCREEN_SIZE     0x1f

#define DE1_CMD_LAYER_REQUEST       0x40
#define DE1_CMD_LAYER_RELEASE       0x41
#define DE1_CMD_LAYER_OPEN          0x42
#define DE1_CMD_LAYER_CLOSE         0x43
#define DE1_CMD_LAYER_SET_FB        0x44
#define DE1_CMD_LAYER_GET_FB        0x45
#define DE1_CMD_LAYER_SET_SRC_WIN   0x46
#define DE1_CMD_LAYER_GET_SRC_WIN   0x47
#define DE1_CMD_LAYER_SET_SCN_WIN   0x48
#define DE1_CMD_LAYER_GET_SCN_WIN   0x49
#define DE1_CMD_LAYER_SET_PARA      0x4a
#define DE1_CMD_LAYER_GET_PARA      0x4b

#define DE1_CMD_HDMI_ON             0x1c0
#define DE1_CMD_HDMI_OFF            0x1c1
#define DE1_CMD_HDMI_SET_MODE       0x1c2
#define DE1_CMD_HDMI_GET_MODE       0x1c3
#define DE1_CMD_HDMI_SUPPORT_MODE   0x1c4
#define DE1_CMD_HDMI_GET_HPD        0x1c5

#define DE1_CMD_FB_REQUEST          0x280
#define DE1_CMD_FB_RELEASE          0x281
#define DE1_CMD_FB_GET_PARA         0x282

/* DE1 pixel formats */
typedef enum {
    DE1_FORMAT_1BPP         = 0x0,
    DE1_FORMAT_2BPP         = 0x1,
    DE1_FORMAT_4BPP         = 0x2,
    DE1_FORMAT_8BPP         = 0x3,
    DE1_FORMAT_RGB655       = 0x4,
    DE1_FORMAT_RGB565       = 0x5,
    DE1_FORMAT_RGB556       = 0x6,
    DE1_FORMAT_ARGB1555     = 0x7,
    DE1_FORMAT_RGBA5551     = 0x8,
    DE1_FORMAT_ARGB888      = 0x9,
    DE1_FORMAT_ARGB8888     = 0xa,
    DE1_FORMAT_RGB888       = 0xb,
    DE1_FORMAT_ARGB4444     = 0xc,
} de1_pixel_fmt;

typedef enum {
    DE1_SEQ_ARGB    = 0x0,
    DE1_SEQ_BGRA    = 0x2,
    DE1_SEQ_P3210   = 0xf,
} de1_pixel_seq;

typedef enum {
    DE1_MOD_INTERLEAVED = 0x1,
} de1_pixel_mod;

typedef enum {
    DE1_BT601   = 0,
    DE1_BT709   = 1,
} de1_cs_mode;

typedef enum {
    DE1_3D_SRC_MODE_TB = 0x0,
} de1_3d_src_mode;

typedef enum {
    DE1_3D_OUT_MODE_TB = 0x0,
} de1_3d_out_mode;

typedef enum {
    DE1_LAYER_WORK_MODE_NORMAL  = 0,
    DE1_LAYER_WORK_MODE_SCALER  = 4,
} de1_layer_work_mode;

typedef enum {
    DE1_FB_MODE_SCREEN0 = 0,
    DE1_FB_MODE_SCREEN1 = 1,
} de1_fb_mode;

/* DE1 framebuffer structure */
typedef struct {
    __u32               addr[3];
    disp_rectsz         size;
    de1_pixel_fmt       format;
    de1_pixel_seq       seq;
    de1_pixel_mod       mode;
    __bool              br_swap;
    de1_cs_mode         cs_mode;
    __bool              b_trd_src;
    de1_3d_src_mode     trd_mode;
    __u32               trd_right_addr[3];
    __bool              pre_multiply;
} de1_fb_t;

/* DE1 layer info structure */
typedef struct {
    de1_layer_work_mode mode;
    __bool              b_from_screen;
    __u8                pipe;
    __u8                prio;
    __bool              alpha_en;
    __u16               alpha_val;
    __bool              ck_enable;
    disp_rect           src_win;
    disp_rect           scn_win;
    de1_fb_t            fb;
    __bool              b_trd_out;
    de1_3d_out_mode     out_trd_mode;
} de1_layer_info_t;

/* DE1 FB create parameters */
typedef struct {
    de1_fb_mode             fb_mode;
    de1_layer_work_mode     mode;
    __u32                   buffer_num;
    __u32                   width;
    __u32                   height;
    __u32                   output_width;
    __u32                   output_height;
    __u32                   primary_screen_id;
    __u32                   aux_output_width;
    __u32                   aux_output_height;
    __u32                   line_length;
    __u32                   smem_len;
    __u32                   ch1_offset;
    __u32                   ch2_offset;
} de1_fb_create_para_t;

/*
 * ============================================================================
 * DE2 (H3) Specific Definitions
 * ============================================================================
 */

/* DE2 ioctl commands */
#define DE2_CMD_SET_BKCOLOR         0x03
#define DE2_CMD_GET_SCN_WIDTH       0x07
#define DE2_CMD_GET_SCN_HEIGHT      0x08
#define DE2_CMD_GET_OUTPUT_TYPE     0x09
#define DE2_CMD_DEVICE_SWITCH       0x0f
#define DE2_CMD_GET_OUTPUT          0x10

#define DE2_CMD_LAYER_ENABLE        0x40
#define DE2_CMD_LAYER_DISABLE       0x41
#define DE2_CMD_LAYER_SET_INFO      0x42
#define DE2_CMD_LAYER_GET_INFO      0x43
#define DE2_CMD_LAYER_SET_CONFIG    0x47
#define DE2_CMD_LAYER_GET_CONFIG    0x48

#define DE2_CMD_HDMI_SUPPORT_MODE   0xc4
#define DE2_CMD_HDMI_GET_EDID       0xc6

#define DE2_CMD_FB_REQUEST          0x280
#define DE2_CMD_FB_RELEASE          0x281

/* DE2 pixel formats */
typedef enum {
    DE2_FORMAT_ARGB_8888    = 0x00,
    DE2_FORMAT_ABGR_8888    = 0x01,
    DE2_FORMAT_RGBA_8888    = 0x02,
    DE2_FORMAT_BGRA_8888    = 0x03,
    DE2_FORMAT_XRGB_8888    = 0x04,
    DE2_FORMAT_XBGR_8888    = 0x05,
    DE2_FORMAT_RGBX_8888    = 0x06,
    DE2_FORMAT_BGRX_8888    = 0x07,
    DE2_FORMAT_RGB_888      = 0x08,
    DE2_FORMAT_BGR_888      = 0x09,
    DE2_FORMAT_RGB_565      = 0x0a,
    DE2_FORMAT_BGR_565      = 0x0b,
} de2_pixel_format;

typedef enum {
    DE2_BT601   = 0,
    DE2_BT709   = 1,
    DE2_YCC     = 2,
} de2_color_space;

typedef enum {
    DE2_3D_OUT_MODE_TB  = 0x0,
    DE2_3D_OUT_MODE_FP  = 0x1,
    DE2_3D_OUT_MODE_SSF = 0x2,
    DE2_3D_OUT_MODE_SSH = 0x3,
    DE2_3D_OUT_MODE_LI  = 0x4,
} de2_3d_out_mode;

typedef enum {
    DE2_LAYER_MODE_BUFFER = 0,
    DE2_LAYER_MODE_COLOR  = 1,
} de2_layer_mode;

typedef enum {
    DE2_SCAN_PROGRESSIVE = 0,
} de2_scan_flags;

typedef enum {
    DE2_BF_NORMAL = 0,
} de2_buffer_flags;

/* DE2 64-bit rectangle for cropping */
typedef struct {
    long long x;
    long long y;
    long long width;
    long long height;
} de2_rect64;

/* DE2 framebuffer info */
typedef struct {
    unsigned long long  addr[3];
    disp_rectsz         size[3];
    unsigned int        align[3];
    de2_pixel_format    format;
    de2_color_space     color_space;
    unsigned int        trd_right_addr[3];
    bool                pre_multiply;
    de2_rect64          crop;
    de2_buffer_flags    flags;
    de2_scan_flags      scan;
} de2_fb_info;

/* DE2 layer info */
typedef struct {
    de2_layer_mode      mode;
    unsigned char       zorder;
    unsigned char       alpha_mode;
    unsigned char       alpha_value;
    disp_rect           screen_win;
    bool                b_trd_out;
    de2_3d_out_mode     out_trd_mode;
    union {
        unsigned int    color;
        de2_fb_info     fb;
    };
    unsigned int        id;
} de2_layer_info;

/* DE2 layer config */
typedef struct {
    de2_layer_info      info;
    bool                enable;
    unsigned int        channel;
    unsigned int        layer_id;
} de2_layer_config;

/* DE2 output info for device switch */
typedef struct {
    unsigned int        type;
    unsigned int        mode;
} de2_output;

/* DE2 FB create info */
typedef struct {
    unsigned int        fb_mode;
    de2_layer_mode      mode;
    unsigned int        buffer_num;
    unsigned int        width;
    unsigned int        height;
    unsigned int        output_width;
    unsigned int        output_height;
} de2_fb_create_info;

/*
 * ============================================================================
 * Mode Info Table
 * ============================================================================
 */
typedef struct {
    disp_tv_mode mode;
    const char *name;
    uint32_t width;
    uint32_t height;
    uint32_t refresh;
} mode_info_t;

static const mode_info_t mode_table[] = {
    { DISP_TV_MOD_480I,             "480i",     720,  480,  60 },
    { DISP_TV_MOD_576I,             "576i",     720,  576,  50 },
    { DISP_TV_MOD_480P,             "480p",     720,  480,  60 },
    { DISP_TV_MOD_576P,             "576p",     720,  576,  50 },
    { DISP_TV_MOD_720P_50HZ,        "720p50",  1280,  720,  50 },
    { DISP_TV_MOD_720P_60HZ,        "720p60",  1280,  720,  60 },
    { DISP_TV_MOD_1080I_50HZ,       "1080i50", 1920, 1080,  50 },
    { DISP_TV_MOD_1080I_60HZ,       "1080i60", 1920, 1080,  60 },
    { DISP_TV_MOD_1080P_24HZ,       "1080p24", 1920, 1080,  24 },
    { DISP_TV_MOD_1080P_50HZ,       "1080p50", 1920, 1080,  50 },
    { DISP_TV_MOD_1080P_60HZ,       "1080p60", 1920, 1080,  60 },
    { DISP_TV_MOD_1080P_25HZ,       "1080p25", 1920, 1080,  25 },
    { DISP_TV_MOD_1080P_30HZ,       "1080p30", 1920, 1080,  30 },
    { DISP_TV_MOD_3840_2160P_30HZ,  "2160p30", 3840, 2160,  30 },
    { DISP_TV_MOD_3840_2160P_25HZ,  "2160p25", 3840, 2160,  25 },
    { DISP_TV_MOD_3840_2160P_24HZ,  "2160p24", 3840, 2160,  24 },
    { (disp_tv_mode)-1,             NULL,         0,    0,   0 }
};

/*
 * ============================================================================
 * Global State
 * ============================================================================
 */
static int g_disp_fd = -1;
static int g_fb_fd = -1;
static uint32_t g_screen = 0;
static int g_verbose = 0;
static int g_force = 0;

/* Debug macro */
#define DEBUG(fmt, ...) do { \
    if (g_verbose) { \
        printf("[DEBUG] %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
        fflush(stdout); \
    } \
} while(0)

/*
 * ============================================================================
 * Signal Handlers
 * ============================================================================
 */
static void segfault_handler(int sig)
{
    fprintf(stderr, "\n*** SIGNAL %d ***\n", sig);
    fprintf(stderr, "This likely indicates a structure alignment mismatch.\n");
    fprintf(stderr, "Run with 'debug' command to check structure sizes.\n");
    fflush(stderr);
    _exit(128 + sig);
}

static void install_signal_handlers(void)
{
    struct sigaction sa;
    sa.sa_handler = segfault_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGSEGV, &sa, NULL);
    sigaction(SIGBUS, &sa, NULL);
    sigaction(SIGABRT, &sa, NULL);
}

/*
 * ============================================================================
 * Display Engine Version Detection
 * ============================================================================
 */

/* Detect SoC type from /proc/cpuinfo */
static de_version_t detect_soc_from_cpuinfo(void)
{
    FILE *f = fopen(CPUINFO, "r");
    if (!f) return DE_VERSION_UNKNOWN;

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, "Hardware")) {
            fclose(f);
            /* A20 variants */
            if (strstr(line, "sun7i") || strstr(line, "A20") ||
                strstr(line, "sun4i") || strstr(line, "A10")) {
                DEBUG("Detected A10/A20 (sun4i/sun7i) from cpuinfo");
                return DE_VERSION_1;
            }
            /* H3/H5/A64 variants */
            if (strstr(line, "sun8i") || strstr(line, "H3") ||
                strstr(line, "H5") || strstr(line, "sun50i") ||
                strstr(line, "A64")) {
                DEBUG("Detected H3/H5/A64 (sun8i/sun50i) from cpuinfo");
                return DE_VERSION_2;
            }
            return DE_VERSION_UNKNOWN;
        }
    }
    fclose(f);
    return DE_VERSION_UNKNOWN;
}

/* Detect by probing ioctl commands */
static de_version_t detect_by_ioctl_probe(int fd)
{
    unsigned long args[4] = {0, 0, 0, 0};
    int ret;

    /* Try DE1-specific command (HDMI_GET_HPD at 0x1c5) */
    ret = ioctl(fd, DE1_CMD_HDMI_GET_HPD, args);
    if (ret >= 0 || errno != ENOTTY) {
        DEBUG("DE1 HDMI_GET_HPD ioctl responded (ret=%d, errno=%d)", ret, errno);
        return DE_VERSION_1;
    }

    /* Try DE2-specific command (HDMI_SUPPORT_MODE at 0xc4) */
    args[0] = 0; args[1] = DISP_TV_MOD_720P_60HZ;
    ret = ioctl(fd, DE2_CMD_HDMI_SUPPORT_MODE, args);
    if (ret >= 0 || errno != ENOTTY) {
        DEBUG("DE2 HDMI_SUPPORT_MODE ioctl responded (ret=%d, errno=%d)", ret, errno);
        return DE_VERSION_2;
    }

    return DE_VERSION_UNKNOWN;
}

/* Main detection function */
static de_version_t detect_de_version(int fd)
{
    de_version_t ver;

    /* Try cpuinfo first */
    ver = detect_soc_from_cpuinfo();
    if (ver != DE_VERSION_UNKNOWN) {
        return ver;
    }

    /* Fall back to ioctl probing */
    ver = detect_by_ioctl_probe(fd);
    if (ver != DE_VERSION_UNKNOWN) {
        return ver;
    }

    /* Default to DE1 for older kernels without proper detection */
    DEBUG("Could not detect DE version, defaulting to DE1 (A20)");
    return DE_VERSION_1;
}

static const char* de_version_name(de_version_t ver)
{
    switch (ver) {
        case DE_VERSION_1: return "DE1 (A10/A20)";
        case DE_VERSION_2: return "DE2 (H3/H5/A64)";
        default: return "Unknown";
    }
}

/*
 * ============================================================================
 * Device Open/Close
 * ============================================================================
 */
static int disp_open(void)
{
    if (g_disp_fd >= 0) return 0;

    g_disp_fd = open(DISP_DEV, O_RDWR);
    if (g_disp_fd < 0) {
        perror("Failed to open " DISP_DEV);
        return -1;
    }

    /* Detect display engine version */
    g_de_version = detect_de_version(g_disp_fd);
    DEBUG("Display Engine: %s", de_version_name(g_de_version));

    return 0;
}

static void disp_close(void)
{
    if (g_disp_fd >= 0) {
        close(g_disp_fd);
        g_disp_fd = -1;
    }
}

/*
 * ============================================================================
 * Low-level ioctl wrapper
 * ============================================================================
 */
static int disp_ioctl(unsigned int cmd, unsigned long *args)
{
    if (g_disp_fd < 0) {
        fprintf(stderr, "Display device not open\n");
        return -1;
    }

    DEBUG("ioctl: cmd=0x%x args={%lu, %lu, 0x%lx, %lu}",
          cmd, args[0], args[1], args[2], args[3]);

    int ret = ioctl(g_disp_fd, cmd, args);
    int saved_errno = errno;

    DEBUG("ioctl: returned %d (errno=%d)", ret, saved_errno);

    errno = saved_errno;
    return ret;
}

/*
 * ============================================================================
 * Mode Lookup Functions
 * ============================================================================
 */
static const mode_info_t* find_mode_by_resolution(uint32_t w, uint32_t h, uint32_t refresh)
{
    for (int i = 0; mode_table[i].name != NULL; i++) {
        if (mode_table[i].width == w && mode_table[i].height == h) {
            if (refresh == 0 || mode_table[i].refresh == refresh)
                return &mode_table[i];
        }
    }
    return NULL;
}

static const mode_info_t* find_mode_by_name(const char *name)
{
    for (int i = 0; mode_table[i].name != NULL; i++) {
        if (strcasecmp(mode_table[i].name, name) == 0)
            return &mode_table[i];
    }
    return NULL;
}

static const mode_info_t* get_mode_info(disp_tv_mode mode)
{
    for (int i = 0; mode_table[i].name != NULL; i++) {
        if (mode_table[i].mode == mode)
            return &mode_table[i];
    }
    return NULL;
}

/*
 * ============================================================================
 * DE1 (A20) Implementation
 * ============================================================================
 */

static int de1_get_screen_size(uint32_t *width, uint32_t *height)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    int ret;

    ret = disp_ioctl(DE1_CMD_SCN_GET_WIDTH, args);
    if (ret < 0) return -1;
    *width = (uint32_t)ret;

    args[0] = g_screen;
    ret = disp_ioctl(DE1_CMD_SCN_GET_HEIGHT, args);
    if (ret < 0) return -1;
    *height = (uint32_t)ret;

    return 0;
}

static int de1_get_output_type(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    return disp_ioctl(DE1_CMD_GET_OUTPUT_TYPE, args);
}

static int de1_hdmi_get_hpd(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    return disp_ioctl(DE1_CMD_HDMI_GET_HPD, args);
}

static int de1_hdmi_mode_supported(disp_tv_mode mode)
{
    unsigned long args[4] = {g_screen, mode, 0, 0};
    int ret = disp_ioctl(DE1_CMD_HDMI_SUPPORT_MODE, args);
    return (ret > 0) ? 1 : 0;
}

static disp_tv_mode de1_hdmi_get_mode(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    int ret = disp_ioctl(DE1_CMD_HDMI_GET_MODE, args);
    return (ret < 0) ? (disp_tv_mode)-1 : (disp_tv_mode)ret;
}

/* Default HDMI mode when current mode is unknown/unsupported */
#define DEFAULT_HDMI_MODE DISP_TV_MOD_720P_50HZ  /* 720p50 = mode 4 */

static int de1_hdmi_off(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    return disp_ioctl(DE1_CMD_HDMI_OFF, args);
}

static int de1_hdmi_on(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    return disp_ioctl(DE1_CMD_HDMI_ON, args);
}

static int de1_hdmi_set_mode(disp_tv_mode mode)
{
    unsigned long args[4] = {g_screen, mode, 0, 0};
    return disp_ioctl(DE1_CMD_HDMI_SET_MODE, args);
}

static int de1_hdmi_init(disp_tv_mode mode)
{
    if (!g_force && !de1_hdmi_mode_supported(mode)) {
        fprintf(stderr, "HDMI mode %d not supported (use -f to force)\n", mode);
        return -1;
    }

    de1_hdmi_off();

    if (de1_hdmi_set_mode(mode) < 0)
        return -1;

    return de1_hdmi_on();
}

static int de1_fb_release(uint32_t fb_id)
{
    unsigned long args[4] = {fb_id, 0, 0, 0};
    return disp_ioctl(DE1_CMD_FB_RELEASE, args);
}

static int de1_fb_request(uint32_t fb_id, de1_fb_create_para_t *para)
{
    unsigned long args[4] = {fb_id, (unsigned long)para, 0, 0};
    return disp_ioctl(DE1_CMD_FB_REQUEST, args);
}

static int de1_setup_fb_with_scaling(uint32_t fb_id, uint32_t fb_w, uint32_t fb_h,
                                     uint32_t scn_w, uint32_t scn_h, int depth)
{
    de1_fb_create_para_t para;
    int needs_scaling = (fb_w != scn_w || fb_h != scn_h);

    DEBUG("DE1 setup: fb=%ux%u scn=%ux%u depth=%d scaling=%d",
          fb_w, fb_h, scn_w, scn_h, depth, needs_scaling);

    de1_fb_release(fb_id);

    memset(&para, 0, sizeof(para));
    para.fb_mode = DE1_FB_MODE_SCREEN0;
    para.mode = needs_scaling ? DE1_LAYER_WORK_MODE_SCALER : DE1_LAYER_WORK_MODE_NORMAL;
    para.buffer_num = 1;
    para.width = fb_w;
    para.height = fb_h;
    para.output_width = scn_w;
    para.output_height = scn_h;
    para.primary_screen_id = g_screen;

    if (de1_fb_request(fb_id, &para) < 0) {
        fprintf(stderr, "Failed to request framebuffer\n");
        return -1;
    }

    if (needs_scaling) {
        printf("Hardware scaling enabled: %dx%d -> %dx%d\n", fb_w, fb_h, scn_w, scn_h);
        printf("NOTE: Scaling mode is incompatible with Mali/EGL. Run 'noscale' first.\n");
    } else {
        printf("Framebuffer configured: %dx%d (no scaling)\n", fb_w, fb_h);
    }

    return 0;
}

/*
 * ============================================================================
 * DE2 (H3) Implementation
 * ============================================================================
 */

static int de2_get_screen_size(uint32_t *width, uint32_t *height)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    int ret;

    ret = disp_ioctl(DE2_CMD_GET_SCN_WIDTH, args);
    if (ret < 0) return -1;
    *width = (uint32_t)ret;

    args[0] = g_screen;
    ret = disp_ioctl(DE2_CMD_GET_SCN_HEIGHT, args);
    if (ret < 0) return -1;
    *height = (uint32_t)ret;

    return 0;
}

static int de2_get_output_type(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    return disp_ioctl(DE2_CMD_GET_OUTPUT_TYPE, args);
}

static int de2_hdmi_mode_supported(disp_tv_mode mode)
{
    unsigned long args[4] = {g_screen, mode, 0, 0};
    int ret = disp_ioctl(DE2_CMD_HDMI_SUPPORT_MODE, args);
    return (ret > 0) ? 1 : 0;
}

static int de2_hdmi_get_hpd(void)
{
    /* DE2 doesn't have a direct HPD ioctl - use sysfs */
    FILE *f = fopen(HDMI_STATE, "r");
    if (f) {
        int state = 0;
        if (fscanf(f, "%d", &state) == 1) {
            fclose(f);
            return state;
        }
        fclose(f);
    }
    return -1;
}

static disp_tv_mode de2_hdmi_get_mode(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    de2_output output;

    memset(&output, 0, sizeof(output));
    args[1] = (unsigned long)&output;
    int ret = disp_ioctl(DE2_CMD_GET_OUTPUT, args);
    if (ret < 0) return (disp_tv_mode)-1;

    DEBUG("de2_hdmi_get_mode: type=%u mode=%u", output.type, output.mode);
    return (disp_tv_mode)output.mode;
}

static int de2_device_switch(disp_output_type type, disp_tv_mode mode)
{
    unsigned long args[4] = {g_screen, type, mode, 0};
    DEBUG("de2_device_switch: screen=%lu type=%lu mode=%lu", args[0], args[1], args[2]);
    int ret = disp_ioctl(DE2_CMD_DEVICE_SWITCH, args);
    DEBUG("de2_device_switch: returned %d", ret);
    return ret;
}

static int de2_hdmi_init(disp_tv_mode mode)
{
    if (!g_force && !de2_hdmi_mode_supported(mode)) {
        fprintf(stderr, "HDMI mode %d not supported (use -f to force)\n", mode);
        return -1;
    }

    return de2_device_switch(DISP_OUTPUT_TYPE_HDMI, mode);
}

static int de2_hdmi_off(void)
{
    return de2_device_switch(DISP_OUTPUT_TYPE_NONE, 0);
}

/*
 * DE2 Scaling Notes:
 * Unlike DE1 (A20), DE2 (H3) handles scaling automatically via VSU/GSU.
 * There's no FB_REQUEST ioctl - we just change FB resolution via fbdev.
 * The DE2 automatically scales the framebuffer content to screen_win size.
 */

static int de2_setup_fb_with_scaling(uint32_t fb_id, uint32_t fb_w, uint32_t fb_h,
                                     uint32_t scn_w, uint32_t scn_h, int depth)
{
    struct fb_var_screeninfo vinfo;
    int fd;
    int needs_scaling = (fb_w != scn_w || fb_h != scn_h);

    (void)fb_id;  /* Not used on DE2 */

    DEBUG("DE2 setup: fb=%ux%u scn=%ux%u depth=%d scaling=%d",
          fb_w, fb_h, scn_w, scn_h, depth, needs_scaling);

    /*
     * On DE2, scaling is automatic. We just need to set the FB resolution
     * via standard fbdev. The display engine will scale to screen size.
     */
    fd = open(FB_DEV, O_RDWR);
    if (fd < 0) {
        perror("Failed to open " FB_DEV);
        return -1;
    }

    if (ioctl(fd, FBIOGET_VSCREENINFO, &vinfo) < 0) {
        perror("FBIOGET_VSCREENINFO failed");
        close(fd);
        return -1;
    }

    /* Only change if different from current */
    if (vinfo.xres != fb_w || vinfo.yres != fb_h ||
        vinfo.bits_per_pixel != (unsigned)depth) {

        vinfo.xres = fb_w;
        vinfo.yres = fb_h;
        vinfo.xres_virtual = fb_w;
        vinfo.yres_virtual = fb_h * 2;  /* Double buffer */
        vinfo.bits_per_pixel = depth;

        /* Set color format */
        if (depth == 16) {
            vinfo.red.offset = 11; vinfo.red.length = 5;
            vinfo.green.offset = 5; vinfo.green.length = 6;
            vinfo.blue.offset = 0; vinfo.blue.length = 5;
            vinfo.transp.offset = 0; vinfo.transp.length = 0;
        } else if (depth == 24 || depth == 32) {
            vinfo.red.offset = 16; vinfo.red.length = 8;
            vinfo.green.offset = 8; vinfo.green.length = 8;
            vinfo.blue.offset = 0; vinfo.blue.length = 8;
            vinfo.transp.offset = (depth == 32) ? 24 : 0;
            vinfo.transp.length = (depth == 32) ? 8 : 0;
        }

        if (ioctl(fd, FBIOPUT_VSCREENINFO, &vinfo) < 0) {
            perror("FBIOPUT_VSCREENINFO failed");
            close(fd);
            return -1;
        }

        printf("Framebuffer set to: %dx%d @ %dbpp\n", fb_w, fb_h, depth);
    } else {
        printf("Framebuffer already at: %dx%d @ %dbpp\n", fb_w, fb_h, depth);
    }

    close(fd);

    if (needs_scaling) {
        printf("DE2 auto-scaling: %dx%d -> %dx%d (handled by hardware)\n",
               fb_w, fb_h, scn_w, scn_h);
    } else {
        printf("No scaling needed (1:1)\n");
    }

    return 0;
}

/*
 * ============================================================================
 * Unified API (dispatches to DE1 or DE2)
 * ============================================================================
 */

static int get_screen_size(uint32_t *width, uint32_t *height)
{
    switch (g_de_version) {
        case DE_VERSION_1: return de1_get_screen_size(width, height);
        case DE_VERSION_2: return de2_get_screen_size(width, height);
        default: return -1;
    }
}

static int get_output_type(void)
{
    switch (g_de_version) {
        case DE_VERSION_1: return de1_get_output_type();
        case DE_VERSION_2: return de2_get_output_type();
        default: return -1;
    }
}

static int hdmi_get_hpd(void)
{
    /* Try sysfs first (works for both) */
    FILE *f = fopen(HDMI_STATE, "r");
    if (f) {
        int state = 0;
        if (fscanf(f, "%d", &state) == 1) {
            fclose(f);
            return state;
        }
        fclose(f);
    }

    switch (g_de_version) {
        case DE_VERSION_1: return de1_hdmi_get_hpd();
        case DE_VERSION_2: return de2_hdmi_get_hpd();
        default: return -1;
    }
}

static int hdmi_mode_supported(disp_tv_mode mode)
{
    switch (g_de_version) {
        case DE_VERSION_1: return de1_hdmi_mode_supported(mode);
        case DE_VERSION_2: return de2_hdmi_mode_supported(mode);
        default: return 0;
    }
}

static disp_tv_mode hdmi_get_mode(void)
{
    switch (g_de_version) {
        case DE_VERSION_1: return de1_hdmi_get_mode();
        case DE_VERSION_2: return de2_hdmi_get_mode();
        default: return (disp_tv_mode)-1;
    }
}

static int hdmi_init(disp_tv_mode mode)
{
    switch (g_de_version) {
        case DE_VERSION_1: return de1_hdmi_init(mode);
        case DE_VERSION_2: return de2_hdmi_init(mode);
        default: return -1;
    }
}

/* Default HDMI mode when current mode is unknown/unsupported */
static int hdmi_on(void)
{
    disp_tv_mode mode;
    int ret;
    int saved_force;

    switch (g_de_version) {
        case DE_VERSION_1:
            /* Try simple on first */
            ret = de1_hdmi_on();
            if (ret < 0) {
                /* Fall back to init with default mode, force it */
                DEBUG("Simple HDMI on failed, forcing default mode %d", DEFAULT_HDMI_MODE);
                saved_force = g_force;
                g_force = 1;  /* Force mode - can't check EDID when off */
                ret = de1_hdmi_init(DEFAULT_HDMI_MODE);
                g_force = saved_force;
            }
            return ret;
        case DE_VERSION_2:
            /* Get current mode, use default if invalid or unsupported (like 480i) */
            mode = hdmi_get_mode();
            DEBUG("hdmi_on DE2: current mode = %d", mode);
            if ((int)mode <= 0 || mode == DISP_TV_MOD_480I || mode == DISP_TV_MOD_576I) {
                DEBUG("Current mode %d invalid/unsuitable, using default %d", mode, DEFAULT_HDMI_MODE);
                mode = DEFAULT_HDMI_MODE;
            }
            DEBUG("hdmi_on DE2: will use mode %d", mode);
            /* Force the mode - can't reliably check EDID when HDMI is off */
            saved_force = g_force;
            g_force = 1;
            ret = de2_hdmi_init(mode);
            DEBUG("hdmi_on DE2: de2_hdmi_init returned %d", ret);
            g_force = saved_force;
            return ret;
        default:
            return -1;
    }
}

static int hdmi_off(void)
{
    switch (g_de_version) {
        case DE_VERSION_1: return de1_hdmi_off();
        case DE_VERSION_2: return de2_hdmi_off();
        default: return -1;
    }
}

static int setup_fb_with_scaling(uint32_t fb_id, uint32_t fb_w, uint32_t fb_h,
                                 uint32_t scn_w, uint32_t scn_h, int depth)
{
    switch (g_de_version) {
        case DE_VERSION_1:
            return de1_setup_fb_with_scaling(fb_id, fb_w, fb_h, scn_w, scn_h, depth);
        case DE_VERSION_2:
            return de2_setup_fb_with_scaling(fb_id, fb_w, fb_h, scn_w, scn_h, depth);
        default:
            return -1;
    }
}

/*
 * ============================================================================
 * Framebuffer Configuration via fbdev
 * ============================================================================
 */
static int fb_configure(uint32_t width, uint32_t height, int depth)
{
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;

    g_fb_fd = open(FB_DEV, O_RDWR);
    if (g_fb_fd < 0) {
        perror("Failed to open " FB_DEV);
        return -1;
    }

    if (ioctl(g_fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0) {
        perror("FBIOGET_VSCREENINFO failed");
        close(g_fb_fd);
        g_fb_fd = -1;
        return -1;
    }

    vinfo.xres = width;
    vinfo.yres = height;
    vinfo.xres_virtual = width;
    vinfo.yres_virtual = height;
    vinfo.bits_per_pixel = depth;

    if (depth == 16) {
        vinfo.red.offset = 11; vinfo.red.length = 5;
        vinfo.green.offset = 5; vinfo.green.length = 6;
        vinfo.blue.offset = 0; vinfo.blue.length = 5;
        vinfo.transp.offset = 0; vinfo.transp.length = 0;
    } else if (depth == 24 || depth == 32) {
        vinfo.red.offset = 16; vinfo.red.length = 8;
        vinfo.green.offset = 8; vinfo.green.length = 8;
        vinfo.blue.offset = 0; vinfo.blue.length = 8;
        vinfo.transp.offset = (depth == 32) ? 24 : 0;
        vinfo.transp.length = (depth == 32) ? 8 : 0;
    }

    if (ioctl(g_fb_fd, FBIOPUT_VSCREENINFO, &vinfo) < 0) {
        perror("FBIOPUT_VSCREENINFO failed");
        close(g_fb_fd);
        g_fb_fd = -1;
        return -1;
    }

    if (ioctl(g_fb_fd, FBIOGET_FSCREENINFO, &finfo) < 0) {
        perror("FBIOGET_FSCREENINFO failed");
        close(g_fb_fd);
        g_fb_fd = -1;
        return -1;
    }

    printf("Framebuffer configured: %dx%d @ %d bpp\n",
           vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
    printf("Line length: %d bytes, Total size: %d bytes\n",
           finfo.line_length, finfo.smem_len);

    return 0;
}

static int get_fb_info(struct fb_var_screeninfo *vinfo, struct fb_fix_screeninfo *finfo)
{
    int fd = open(FB_DEV, O_RDONLY);
    if (fd < 0) return -1;

    if (vinfo && ioctl(fd, FBIOGET_VSCREENINFO, vinfo) < 0) {
        close(fd);
        return -1;
    }

    if (finfo && ioctl(fd, FBIOGET_FSCREENINFO, finfo) < 0) {
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

/*
 * ============================================================================
 * Information Display
 * ============================================================================
 */
static void show_info(void)
{
    uint32_t width, height;
    disp_tv_mode mode;
    const mode_info_t *info;
    int hpd, output_type;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;

    printf("=== Sunxi Display Information ===\n\n");

    printf("Display Engine: %s\n", de_version_name(g_de_version));
    printf("Screen: %u\n", g_screen);

    output_type = get_output_type();
    printf("Output type: ");
    switch (output_type) {
        case DISP_OUTPUT_TYPE_NONE: printf("None\n"); break;
        case DISP_OUTPUT_TYPE_LCD:  printf("LCD\n"); break;
        case DISP_OUTPUT_TYPE_TV:   printf("TV\n"); break;
        case DISP_OUTPUT_TYPE_HDMI: printf("HDMI\n"); break;
        case DISP_OUTPUT_TYPE_VGA:  printf("VGA\n"); break;
        default: printf("Unknown (%d)\n", output_type); break;
    }

    hpd = hdmi_get_hpd();
    printf("HDMI Hot Plug: %s (raw: %d)\n",
           hpd > 0 ? "Connected" : (hpd == 0 ? "Disconnected" : "Error"), hpd);

    mode = hdmi_get_mode();
    info = get_mode_info(mode);
    printf("Current HDMI mode: %d", (int)mode);
    if (info) {
        printf(" = %s (%dx%d @ %dHz)\n",
               info->name, info->width, info->height, info->refresh);
    } else {
        printf(" (not in table)\n");
    }

    if (get_screen_size(&width, &height) == 0) {
        printf("Screen size: %ux%u\n", width, height);
    } else {
        printf("Screen size: failed to read\n");
    }

    printf("\n--- Framebuffer (" FB_DEV ") ---\n");
    if (get_fb_info(&vinfo, &finfo) == 0) {
        printf("Resolution: %ux%u", vinfo.xres, vinfo.yres);
        if (vinfo.xres_virtual != vinfo.xres || vinfo.yres_virtual != vinfo.yres) {
            printf(" (virtual: %ux%u)", vinfo.xres_virtual, vinfo.yres_virtual);
        }
        printf("\n");
        printf("Color depth: %u bpp\n", vinfo.bits_per_pixel);
        printf("Color format: R%u@%u G%u@%u B%u@%u",
               vinfo.red.length, vinfo.red.offset,
               vinfo.green.length, vinfo.green.offset,
               vinfo.blue.length, vinfo.blue.offset);
        if (vinfo.transp.length > 0) {
            printf(" A%u@%u", vinfo.transp.length, vinfo.transp.offset);
        }
        printf("\n");
        printf("Line length: %u bytes\n", finfo.line_length);
        printf("Memory size: %u bytes (%.2f MB)\n",
               finfo.smem_len, finfo.smem_len / (1024.0 * 1024.0));
        printf("Physical address: 0x%lx\n", finfo.smem_start);

        if (get_screen_size(&width, &height) == 0) {
            if (vinfo.xres != width || vinfo.yres != height) {
                printf("Scaling: %ux%u -> %ux%u (active%s)\n",
                       vinfo.xres, vinfo.yres, width, height,
                       g_de_version == DE_VERSION_2 ? ", auto by DE2" : "");
            } else {
                printf("Scaling: none (1:1)\n");
            }
        }
    } else {
        printf("Failed to read framebuffer info\n");
    }

    if (g_de_version == DE_VERSION_2) {
        printf("\nNote: DE2 handles scaling automatically via VSU/GSU hardware.\n");
        printf("      Change FB resolution with 'fb set' or 'scale' to adjust.\n");
    }

    printf("\n--- Supported HDMI modes ---\n");
    printf("  Mode  Name      Resolution   Supported\n");
    printf("  ----  --------  -----------  ---------\n");
    for (int i = 0; mode_table[i].name != NULL; i++) {
        /* Skip 4K modes for DE1 (A20 doesn't support them) */
        if (g_de_version == DE_VERSION_1 && mode_table[i].mode >= DISP_TV_MOD_3840_2160P_30HZ)
            continue;

        int supported = hdmi_mode_supported(mode_table[i].mode);
        printf("  %2d    %-8s  %4dx%-4d    %s\n",
               mode_table[i].mode, mode_table[i].name,
               mode_table[i].width, mode_table[i].height,
               supported > 0 ? "Yes" : "No");
    }
    printf("\nNote: Mode support detection requires HDMI cable connected.\n");
}

static void show_debug_info(void)
{
    printf("=== Structure Size Debug Info ===\n\n");

    printf("Display Engine: %s\n\n", de_version_name(g_de_version));

    printf("Basic types:\n");
    printf("  sizeof(__bool)     = %zu\n", sizeof(__bool));
    printf("  sizeof(__u8)       = %zu\n", sizeof(__u8));
    printf("  sizeof(__u16)      = %zu\n", sizeof(__u16));
    printf("  sizeof(__u32)      = %zu\n", sizeof(__u32));
    printf("  sizeof(__s32)      = %zu\n", sizeof(__s32));
    printf("  sizeof(bool)       = %zu\n", sizeof(bool));
    printf("  sizeof(enum)       = %zu\n", sizeof(disp_tv_mode));

    printf("\nCommon structures:\n");
    printf("  sizeof(disp_rect)    = %zu\n", sizeof(disp_rect));
    printf("  sizeof(disp_rectsz)  = %zu\n", sizeof(disp_rectsz));

    printf("\nDE1 structures:\n");
    printf("  sizeof(de1_fb_t)             = %zu (expected: 64)\n", sizeof(de1_fb_t));
    printf("  sizeof(de1_layer_info_t)     = %zu (expected: 116)\n", sizeof(de1_layer_info_t));
    printf("  sizeof(de1_fb_create_para_t) = %zu (expected: 56)\n", sizeof(de1_fb_create_para_t));

    printf("\nDE2 structures:\n");
    printf("  sizeof(de2_fb_info)       = %zu\n", sizeof(de2_fb_info));
    printf("  sizeof(de2_layer_info)    = %zu\n", sizeof(de2_layer_info));
    printf("  sizeof(de2_layer_config)  = %zu\n", sizeof(de2_layer_config));
    printf("  sizeof(de2_fb_create_info)= %zu\n", sizeof(de2_fb_create_info));
    printf("  sizeof(de2_rect64)        = %zu\n", sizeof(de2_rect64));
}

/*
 * ============================================================================
 * Argument Parsing
 * ============================================================================
 */
static int parse_resolution(const char *str, uint32_t *width, uint32_t *height, uint32_t *refresh)
{
    uint32_t dummy_refresh = 0;
    uint32_t *refresh_ptr = refresh ? refresh : &dummy_refresh;

    *refresh_ptr = 0;
    if (sscanf(str, "%ux%u@%u", width, height, refresh_ptr) >= 2) return 0;
    if (sscanf(str, "%ux%u", width, height) == 2) return 0;
    return -1;
}

static int parse_resolution_depth(const char *str, uint32_t *width, uint32_t *height, int *depth)
{
    if (sscanf(str, "%ux%ux%d", width, height, depth) == 3) return 0;
    return -1;
}

/*
 * ============================================================================
 * Usage
 * ============================================================================
 */
static void print_usage(const char *prog)
{
    printf("Sunxi HDMI and Framebuffer Control Utility\n");
    printf("Supports A10/A20 (DE1) and H3/H5/A64 (DE2)\n\n");
    printf("Usage: %s [-v] [-f] [-s screen] <command> [options]\n\n", prog);
    printf("Options:\n");
    printf("  -v                            Verbose output\n");
    printf("  -f                            Force mode (bypass EDID check)\n");
    printf("  -s <screen>                   Select screen (0 or 1)\n\n");
    printf("Commands:\n");
    printf("  info                          Show display and framebuffer info\n");
    printf("  debug                         Show structure sizes for debugging\n");
    printf("  hdmi on                       Enable HDMI output\n");
    printf("  hdmi off                      Disable HDMI output\n");
    printf("  hdmi mode <name|num>          Set HDMI mode\n");
    printf("  hdmi init <W>x<H>[@Hz]        Initialize HDMI with resolution\n");
    printf("  fb set <W>x<H>x<depth>        Set framebuffer resolution\n");
    printf("  scale <fbW>x<fbH> <scnW>x<scnH> <depth>  Setup scaling\n");
    printf("  autoscale [depth]             Scale current FB to screen\n");
    printf("  noscale [depth]               Disable scaling\n");
    printf("\nHDMI modes:\n");
    for (int i = 0; mode_table[i].name != NULL; i++) {
        printf("  %2d  %-8s  %4dx%d @%dHz\n",
               mode_table[i].mode, mode_table[i].name,
               mode_table[i].width, mode_table[i].height,
               mode_table[i].refresh);
    }
    printf("\nExamples:\n");
    printf("  %s info\n", prog);
    printf("  %s hdmi mode 720p60\n", prog);
    printf("  %s scale 640x480 1280x720 32\n", prog);
    printf("  %s autoscale\n", prog);
    printf("  %s noscale\n", prog);
}

/*
 * ============================================================================
 * Main
 * ============================================================================
 */
int main(int argc, char *argv[])
{
    int ret = 0;
    int arg_start = 1;
    int orig_argc = argc;

    install_signal_handlers();

    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    /* Parse options */
    while (arg_start < argc && argv[arg_start][0] == '-') {
        if (strcmp(argv[arg_start], "-v") == 0) {
            g_verbose = 1;
            arg_start++;
        }
        else if (strcmp(argv[arg_start], "-f") == 0) {
            g_force = 1;
            arg_start++;
        }
        else if (strcmp(argv[arg_start], "-s") == 0 && arg_start + 1 < argc) {
            g_screen = (uint32_t)atoi(argv[arg_start + 1]);
            if (g_screen > 1) {
                fprintf(stderr, "Invalid screen: %s\n", argv[arg_start + 1]);
                return 1;
            }
            arg_start += 2;
        }
        else if (strcmp(argv[arg_start], "-h") == 0 || strcmp(argv[arg_start], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        else {
            fprintf(stderr, "Unknown option: %s\n", argv[arg_start]);
            return 1;
        }
    }

    if (arg_start >= orig_argc) {
        print_usage(argv[0]);
        return 1;
    }

    if (disp_open() < 0) return 1;

    /* info command */
    if (strcmp(argv[arg_start], "info") == 0) {
        show_info();
    }
    /* debug command */
    else if (strcmp(argv[arg_start], "debug") == 0) {
        show_debug_info();
    }
    /* hdmi commands */
    else if (strcmp(argv[arg_start], "hdmi") == 0 && orig_argc >= arg_start + 2) {
        if (strcmp(argv[arg_start + 1], "on") == 0) {
            ret = hdmi_on();
            if (ret == 0) {
                /* Read back the actual mode that was set */
                disp_tv_mode on_mode = hdmi_get_mode();
                const mode_info_t *info = get_mode_info(on_mode);
                if (info) {
                    printf("HDMI enabled: %s (%dx%d @ %dHz)\n",
                           info->name, info->width, info->height, info->refresh);
                } else {
                    printf("HDMI enabled: mode %d\n", on_mode);
                }
            }
        }
        else if (strcmp(argv[arg_start + 1], "off") == 0) {
            ret = hdmi_off();
            if (ret == 0) printf("HDMI disabled\n");
        }
        else if (strcmp(argv[arg_start + 1], "mode") == 0 && orig_argc >= arg_start + 3) {
            const char *mode_arg = argv[arg_start + 2];
            const mode_info_t *info = NULL;

            char *endptr;
            long mode_num = strtol(mode_arg, &endptr, 10);
            if (*endptr == '\0' && mode_num >= 0 && mode_num < DISP_TV_MODE_NUM) {
                info = get_mode_info((disp_tv_mode)mode_num);
                if (!info) {
                    ret = hdmi_init((disp_tv_mode)mode_num);
                    if (ret == 0) printf("HDMI mode set to %ld\n", mode_num);
                }
            } else {
                info = find_mode_by_name(mode_arg);
            }

            if (info) {
                ret = hdmi_init(info->mode);
                if (ret == 0) {
                    printf("HDMI mode set to %s (%dx%d @ %dHz)\n",
                           info->name, info->width, info->height, info->refresh);
                }
            } else if (*endptr != '\0') {
                fprintf(stderr, "Unknown mode: %s\n", mode_arg);
                ret = 1;
            }
        }
        else if (strcmp(argv[arg_start + 1], "init") == 0 && orig_argc >= arg_start + 3) {
            uint32_t width, height, refresh;
            if (parse_resolution(argv[arg_start + 2], &width, &height, &refresh) == 0) {
                const mode_info_t *info = find_mode_by_resolution(width, height, refresh);
                if (info) {
                    ret = hdmi_init(info->mode);
                    if (ret == 0) {
                        printf("HDMI initialized: %s (%dx%d @ %dHz)\n",
                               info->name, info->width, info->height, info->refresh);
                    }
                } else {
                    fprintf(stderr, "No matching mode for %dx%d\n", width, height);
                    ret = 1;
                }
            } else {
                fprintf(stderr, "Invalid resolution: %s\n", argv[arg_start + 2]);
                ret = 1;
            }
        }
        else {
            print_usage(argv[0]);
            ret = 1;
        }
    }
    /* fb set command */
    else if (strcmp(argv[arg_start], "fb") == 0 && orig_argc >= arg_start + 3) {
        if (strcmp(argv[arg_start + 1], "set") == 0) {
            uint32_t width, height;
            int depth;
            if (parse_resolution_depth(argv[arg_start + 2], &width, &height, &depth) == 0) {
                ret = fb_configure(width, height, depth);
            } else {
                fprintf(stderr, "Invalid format. Use: WxHxDEPTH\n");
                ret = 1;
            }
        }
        else {
            print_usage(argv[0]);
            ret = 1;
        }
    }
    /* scale command */
    else if (strcmp(argv[arg_start], "scale") == 0 && orig_argc >= arg_start + 4) {
        uint32_t fb_width, fb_height, scn_width, scn_height;
        int depth;

        if (parse_resolution(argv[arg_start + 1], &fb_width, &fb_height, NULL) == 0 &&
            parse_resolution(argv[arg_start + 2], &scn_width, &scn_height, NULL) == 0) {
            depth = atoi(argv[arg_start + 3]);
            if (depth != 16 && depth != 24 && depth != 32) {
                fprintf(stderr, "Invalid depth. Use 16, 24, or 32\n");
                ret = 1;
            } else {
                ret = setup_fb_with_scaling(0, fb_width, fb_height,
                                           scn_width, scn_height, depth);
                if (ret == 0) {
                    printf("Framebuffer: %dx%d @ %dbpp\n", fb_width, fb_height, depth);
                    printf("Screen output: %dx%d\n", scn_width, scn_height);
                }
            }
        } else {
            fprintf(stderr, "Invalid resolution format\n");
            ret = 1;
        }
    }
    /* autoscale command */
    else if (strcmp(argv[arg_start], "autoscale") == 0) {
        struct fb_var_screeninfo vinfo;
        uint32_t scn_width, scn_height;
        int depth;

        if (get_fb_info(&vinfo, NULL) < 0) {
            fprintf(stderr, "Failed to read framebuffer settings\n");
            ret = 1;
        }
        else if (get_screen_size(&scn_width, &scn_height) < 0) {
            fprintf(stderr, "Failed to get screen size\n");
            ret = 1;
        }
        else {
            if (arg_start + 1 < orig_argc) {
                depth = atoi(argv[arg_start + 1]);
                if (depth != 16 && depth != 24 && depth != 32) {
                    fprintf(stderr, "Invalid depth. Use 16, 24, or 32\n");
                    ret = 1;
                    goto done;
                }
            } else {
                depth = vinfo.bits_per_pixel;
            }

            if (vinfo.xres == scn_width && vinfo.yres == scn_height) {
                printf("FB (%ux%u) already matches screen - no scaling needed\n",
                       vinfo.xres, vinfo.yres);
            } else {
                if (g_de_version == DE_VERSION_2) {
                    printf("DE2 auto-scaling already active: %ux%u -> %ux%u\n",
                           vinfo.xres, vinfo.yres, scn_width, scn_height);
                    printf("(DE2 handles scaling automatically - no action needed)\n");
                } else {
                    printf("Scaling: %ux%u -> %ux%u @ %dbpp\n",
                           vinfo.xres, vinfo.yres, scn_width, scn_height, depth);
                    ret = setup_fb_with_scaling(0, vinfo.xres, vinfo.yres,
                                               scn_width, scn_height, depth);
                }
            }
        }
    }
    /* noscale command */
    else if (strcmp(argv[arg_start], "noscale") == 0) {
        struct fb_var_screeninfo vinfo;
        uint32_t scn_width, scn_height;
        int depth;

        if (get_screen_size(&scn_width, &scn_height) < 0) {
            fprintf(stderr, "Failed to get screen size\n");
            ret = 1;
        }
        else {
            if (arg_start + 1 < orig_argc) {
                depth = atoi(argv[arg_start + 1]);
                if (depth != 16 && depth != 24 && depth != 32) {
                    fprintf(stderr, "Invalid depth. Use 16, 24, or 32\n");
                    ret = 1;
                    goto done;
                }
            } else {
                if (get_fb_info(&vinfo, NULL) == 0) {
                    depth = vinfo.bits_per_pixel;
                } else {
                    depth = 32;
                }
            }

            printf("Disabling scaling: FB -> %ux%u @ %dbpp\n",
                   scn_width, scn_height, depth);
            ret = setup_fb_with_scaling(0, scn_width, scn_height,
                                       scn_width, scn_height, depth);
        }
    }
    else {
        print_usage(argv[0]);
        ret = 1;
    }

done:
    if (g_fb_fd >= 0) close(g_fb_fd);
    disp_close();

    return ret;
}
