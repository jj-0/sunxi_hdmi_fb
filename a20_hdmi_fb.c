/*
 * A20 HDMI and Framebuffer Control Utility
 *
 * This utility initializes HDMI output with configurable resolution and
 * sets up framebuffer with hardware scaling support on Allwinner A20 SoC.
 *
 * Based on analysis of:
 * - THEC64 Mini firmware (the64 binary)
 * - Linux 3.4 kernel source for sun7i display driver
 * - a10_display.c reference implementation
 *
 * The A20 display pipeline:
 *   Framebuffer -> Display Engine (DE) -> Layer (with optional scaler) -> TCON -> HDMI
 *
 * Hardware scaling is achieved by configuring a layer in SCALER mode where
 * the source window (src_win) differs from the screen window (scn_win).
 *
 * Compile with:
 *   arm-linux-gnueabihf-gcc -I/path/to/linux-3.4/include -o a20_hdmi_fb a20_hdmi_fb.c
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
#include <stddef.h>  /* for offsetof() */
#include <errno.h>
#include <signal.h>
#include <linux/fb.h>
#include <linux/types.h>

/*
 * Include the kernel display driver header for correct structure definitions.
 * If not available, we define compatible structures below.
 */
#ifdef USE_KERNEL_HEADER
#include <linux/drv_display.h>
#else
/* Kernel-compatible type definitions */
typedef signed char __bool;
typedef struct {__u8  alpha;__u8 red;__u8 green; __u8 blue; } __disp_color_t;
typedef struct {__s32 x; __s32 y; __u32 width; __u32 height;} __disp_rect_t;
typedef struct {__u32 width;__u32 height;                   } __disp_rectsz_t;
#endif

/* Device paths */
#define DISP_DEV    "/dev/disp"
#define FB_DEV      "/dev/fb0"
#define HDMI_STATE  "/sys/class/switch/hdmi/state"

/* Display command base addresses (ioctl codes) */
#define DISP_CMD_BASE_LCD   0x140
#define DISP_CMD_BASE_TV    0x180
#define DISP_CMD_BASE_HDMI  0x1c0
#define DISP_CMD_BASE_VGA   0x200

/* Global display commands */
#define DISP_CMD_SCN_GET_WIDTH      0x08
#define DISP_CMD_SCN_GET_HEIGHT     0x09
#define DISP_CMD_GET_OUTPUT_TYPE    0x0a
#define DISP_CMD_SET_SCREEN_SIZE    0x1f

/* Layer commands */
#define DISP_CMD_LAYER_REQUEST      0x40
#define DISP_CMD_LAYER_RELEASE      0x41
#define DISP_CMD_LAYER_OPEN         0x42
#define DISP_CMD_LAYER_CLOSE        0x43
#define DISP_CMD_LAYER_SET_FB       0x44
#define DISP_CMD_LAYER_GET_FB       0x45
#define DISP_CMD_LAYER_SET_SRC_WIN  0x46
#define DISP_CMD_LAYER_GET_SRC_WIN  0x47
#define DISP_CMD_LAYER_SET_SCN_WIN  0x48
#define DISP_CMD_LAYER_GET_SCN_WIN  0x49
#define DISP_CMD_LAYER_SET_PARA     0x4a
#define DISP_CMD_LAYER_GET_PARA     0x4b

/* Scaler commands */
#define DISP_CMD_SCALER_REQUEST     0x80
#define DISP_CMD_SCALER_RELEASE     0x81
#define DISP_CMD_SCALER_EXECUTE     0x82

/* HDMI commands */
#define DISP_CMD_HDMI_ON            0x1c0
#define DISP_CMD_HDMI_OFF           0x1c1
#define DISP_CMD_HDMI_SET_MODE      0x1c2
#define DISP_CMD_HDMI_GET_MODE      0x1c3
#define DISP_CMD_HDMI_SUPPORT_MODE  0x1c4
#define DISP_CMD_HDMI_GET_HPD       0x1c5
#define DISP_CMD_HDMI_SET_SRC       0x1c6

/* Framebuffer commands */
#define DISP_CMD_FB_REQUEST         0x280
#define DISP_CMD_FB_RELEASE         0x281
#define DISP_CMD_FB_GET_PARA        0x282
#define DISP_CMD_GET_DISP_INIT_PARA 0x283

#ifndef USE_KERNEL_HEADER
/*
 * These definitions must match the kernel's drv_display.h exactly.
 * The __bool type is 'signed char' (1 byte), enums are 4 bytes.
 */

/* HDMI/TV modes */
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
    DISP_TV_MODE_NUM            = 0x1c,
} __disp_tv_mode_t;

/* Pixel formats */
typedef enum {
    DISP_FORMAT_1BPP        = 0x0,
    DISP_FORMAT_2BPP        = 0x1,
    DISP_FORMAT_4BPP        = 0x2,
    DISP_FORMAT_8BPP        = 0x3,
    DISP_FORMAT_RGB655      = 0x4,
    DISP_FORMAT_RGB565      = 0x5,
    DISP_FORMAT_RGB556      = 0x6,
    DISP_FORMAT_ARGB1555    = 0x7,
    DISP_FORMAT_RGBA5551    = 0x8,
    DISP_FORMAT_ARGB888     = 0x9,
    DISP_FORMAT_ARGB8888    = 0xa,
    DISP_FORMAT_RGB888      = 0xb,
    DISP_FORMAT_ARGB4444    = 0xc,
    DISP_FORMAT_YUV444      = 0x10,
    DISP_FORMAT_YUV422      = 0x11,
    DISP_FORMAT_YUV420      = 0x12,
    DISP_FORMAT_YUV411      = 0x13,
} __disp_pixel_fmt_t;

/* Pixel sequence */
typedef enum {
    DISP_SEQ_ARGB   = 0x0,
    DISP_SEQ_BGRA   = 0x2,
    DISP_SEQ_UYVY   = 0x3,
    DISP_SEQ_YUYV   = 0x4,
    DISP_SEQ_VYUY   = 0x5,
    DISP_SEQ_YVYU   = 0x6,
    DISP_SEQ_AYUV   = 0x7,
    DISP_SEQ_VUYA   = 0x8,
    DISP_SEQ_UVUV   = 0x9,
    DISP_SEQ_VUVU   = 0xa,
    DISP_SEQ_P10    = 0xd,
    DISP_SEQ_P01    = 0xe,
    DISP_SEQ_P3210  = 0xf,
    DISP_SEQ_P0123  = 0x10,
} __disp_pixel_seq_t;

/* Pixel mode */
typedef enum {
    DISP_MOD_NON_MB_PLANAR      = 0x0,
    DISP_MOD_INTERLEAVED        = 0x1,
    DISP_MOD_NON_MB_UV_COMBINED = 0x2,
    DISP_MOD_MB_PLANAR          = 0x4,
    DISP_MOD_MB_UV_COMBINED     = 0x6,
} __disp_pixel_mod_t;

/* Color space mode */
typedef enum {
    DISP_BT601  = 0,
    DISP_BT709  = 1,
    DISP_YCC    = 2,
    DISP_VXYCC  = 3,
} __disp_cs_mode_t;

/* 3D source mode */
typedef enum {
    DISP_3D_SRC_MODE_TB     = 0x0,
    DISP_3D_SRC_MODE_FP     = 0x1,
    DISP_3D_SRC_MODE_SSF    = 0x2,
    DISP_3D_SRC_MODE_SSH    = 0x3,
    DISP_3D_SRC_MODE_LI     = 0x4,
} __disp_3d_src_mode_t;

/* 3D output mode */
typedef enum {
    DISP_3D_OUT_MODE_TB     = 0x0,
    DISP_3D_OUT_MODE_FP     = 0x1,
    DISP_3D_OUT_MODE_SSF    = 0x2,
    DISP_3D_OUT_MODE_SSH    = 0x3,
    DISP_3D_OUT_MODE_LI     = 0x4,
    DISP_3D_OUT_MODE_CI_1   = 0x5,
    DISP_3D_OUT_MODE_CI_2   = 0x6,
    DISP_3D_OUT_MODE_CI_3   = 0x7,
    DISP_3D_OUT_MODE_CI_4   = 0x8,
    DISP_3D_OUT_MODE_LIRGB  = 0x9,
    DISP_3D_OUT_MODE_FA     = 0xa,
} __disp_3d_out_mode_t;

/* Layer work mode */
typedef enum {
    DISP_LAYER_WORK_MODE_NORMAL     = 0,
    DISP_LAYER_WORK_MODE_PALETTE    = 1,
    DISP_LAYER_WORK_MODE_INTER_BUF  = 2,
    DISP_LAYER_WORK_MODE_GAMMA      = 3,
    DISP_LAYER_WORK_MODE_SCALER     = 4,
} __disp_layer_work_mode_t;

/* Output type */
typedef enum {
    DISP_OUTPUT_TYPE_NONE   = 0,
    DISP_OUTPUT_TYPE_LCD    = 1,
    DISP_OUTPUT_TYPE_TV     = 2,
    DISP_OUTPUT_TYPE_HDMI   = 4,
    DISP_OUTPUT_TYPE_VGA    = 8,
} __disp_output_type_t;

/* FB mode */
typedef enum {
    FB_MODE_SCREEN0                         = 0,
    FB_MODE_SCREEN1                         = 1,
    FB_MODE_DUAL_SAME_SCREEN_TB             = 2,
    FB_MODE_DUAL_DIFF_SCREEN_SAME_CONTENTS  = 3,
} __fb_mode_t;

/* Framebuffer info structure - must match kernel exactly */
typedef struct {
    __u32                   addr[3];
    __disp_rectsz_t         size;
    __disp_pixel_fmt_t      format;
    __disp_pixel_seq_t      seq;
    __disp_pixel_mod_t      mode;
    __bool                  br_swap;
    __disp_cs_mode_t        cs_mode;
    __bool                  b_trd_src;
    __disp_3d_src_mode_t    trd_mode;
    __u32                   trd_right_addr[3];
    __bool                  pre_multiply;
} __disp_fb_t;

/* Layer info structure - must match kernel exactly */
typedef struct {
    __disp_layer_work_mode_t    mode;
    __bool                      b_from_screen;
    __u8                        pipe;
    __u8                        prio;
    __bool                      alpha_en;
    __u16                       alpha_val;
    __bool                      ck_enable;
    __disp_rect_t               src_win;
    __disp_rect_t               scn_win;
    __disp_fb_t                 fb;
    __bool                      b_trd_out;
    __disp_3d_out_mode_t        out_trd_mode;
} __disp_layer_info_t;

/* Framebuffer create parameters */
typedef struct {
    __fb_mode_t                 fb_mode;
    __disp_layer_work_mode_t    mode;
    __u32                       buffer_num;
    __u32                       width;
    __u32                       height;
    __u32                       output_width;
    __u32                       output_height;
    __u32                       primary_screen_id;
    __u32                       aux_output_width;
    __u32                       aux_output_height;
    __u32                       line_length;
    __u32                       smem_len;
    __u32                       ch1_offset;
    __u32                       ch2_offset;
} __disp_fb_create_para_t;

#endif /* USE_KERNEL_HEADER */

/* Convenience typedefs for cleaner code */
typedef __disp_tv_mode_t disp_tv_mode_t;
typedef __disp_layer_work_mode_t disp_layer_work_mode_t;
typedef __disp_layer_info_t disp_layer_info_t;
typedef __disp_fb_t disp_fb_t;
typedef __disp_rect_t disp_rect_t;
typedef __disp_rectsz_t disp_rectsz_t;
typedef __disp_fb_create_para_t disp_fb_create_para_t;
typedef __disp_pixel_fmt_t disp_pixel_fmt_t;
typedef __disp_pixel_seq_t disp_pixel_seq_t;
typedef __disp_pixel_mod_t disp_pixel_mod_t;

/* Mode info table */
typedef struct {
    disp_tv_mode_t mode;
    const char *name;
    uint32_t width;
    uint32_t height;
    uint32_t refresh;
} mode_info_t;

static const mode_info_t mode_table[] = {
    { DISP_TV_MOD_480I,         "480i",         720,  480,  60 },
    { DISP_TV_MOD_576I,         "576i",         720,  576,  50 },
    { DISP_TV_MOD_480P,         "480p",         720,  480,  60 },
    { DISP_TV_MOD_576P,         "576p",         720,  576,  50 },
    { DISP_TV_MOD_720P_50HZ,    "720p50",      1280,  720,  50 },
    { DISP_TV_MOD_720P_60HZ,    "720p60",      1280,  720,  60 },
    { DISP_TV_MOD_1080I_50HZ,   "1080i50",     1920, 1080,  50 },
    { DISP_TV_MOD_1080I_60HZ,   "1080i60",     1920, 1080,  60 },
    { DISP_TV_MOD_1080P_24HZ,   "1080p24",     1920, 1080,  24 },
    { DISP_TV_MOD_1080P_50HZ,   "1080p50",     1920, 1080,  50 },
    { DISP_TV_MOD_1080P_60HZ,   "1080p60",     1920, 1080,  60 },
    { DISP_TV_MOD_1080P_25HZ,   "1080p25",     1920, 1080,  25 },
    { DISP_TV_MOD_1080P_30HZ,   "1080p30",     1920, 1080,  30 },
    { (disp_tv_mode_t)-1,       NULL,             0,    0,   0 }
};

/* Global state */
static int g_disp_fd = -1;
static int g_fb_fd = -1;
static uint32_t g_screen = 0;  /* Screen 0 by default */
static int g_verbose = 0;      /* Verbose debug output (off by default) */
static int g_force = 0;        /* Force mode setting, bypass EDID check */

/* Debug macro for scaling operations - only prints when -v is passed */
#define SCALE_DEBUG(fmt, ...) do { \
    if (g_verbose) { \
        printf("[DEBUG] %s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
        fflush(stdout); \
    } \
} while(0)

/* Signal handler for debugging segfaults */
static void segfault_handler(int sig)
{
    fprintf(stderr, "\n*** SEGMENTATION FAULT (signal %d) ***\n", sig);
    fprintf(stderr, "This likely indicates a structure alignment mismatch\n");
    fprintf(stderr, "between user-space and kernel-space definitions.\n");
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

/* Check structure sizes at startup and warn if they don't match expected values */
static int check_structure_alignment(void)
{
    int errors = 0;

    SCALE_DEBUG("Checking structure alignment...");

    /* Check basic sizes */
    if (sizeof(__disp_rect_t) != 16) {
        fprintf(stderr, "WARNING: sizeof(__disp_rect_t) = %zu, expected 16\n",
                sizeof(__disp_rect_t));
        errors++;
    }

    if (sizeof(__disp_rectsz_t) != 8) {
        fprintf(stderr, "WARNING: sizeof(__disp_rectsz_t) = %zu, expected 8\n",
                sizeof(__disp_rectsz_t));
        errors++;
    }

    if (sizeof(__disp_fb_t) != 64) {
        fprintf(stderr, "WARNING: sizeof(__disp_fb_t) = %zu, expected 64\n",
                sizeof(__disp_fb_t));
        fprintf(stderr, "  This is a CRITICAL mismatch - scaling WILL crash!\n");
        errors++;
    }

    if (sizeof(__disp_layer_info_t) != 116) {
        fprintf(stderr, "WARNING: sizeof(__disp_layer_info_t) = %zu, expected 116\n",
                sizeof(__disp_layer_info_t));
        fprintf(stderr, "  This is a CRITICAL mismatch - scaling WILL crash!\n");
        errors++;
    }

    if (sizeof(__disp_fb_create_para_t) != 56) {
        fprintf(stderr, "WARNING: sizeof(__disp_fb_create_para_t) = %zu, expected 56\n",
                sizeof(__disp_fb_create_para_t));
        fprintf(stderr, "  This is a CRITICAL mismatch - scaling WILL crash!\n");
        errors++;
    }

    /* Check critical field offsets in __disp_layer_info_t */
    if (offsetof(__disp_layer_info_t, src_win) != 12) {
        fprintf(stderr, "WARNING: offsetof(__disp_layer_info_t, src_win) = %zu, expected 12\n",
                offsetof(__disp_layer_info_t, src_win));
        errors++;
    }

    if (offsetof(__disp_layer_info_t, scn_win) != 28) {
        fprintf(stderr, "WARNING: offsetof(__disp_layer_info_t, scn_win) = %zu, expected 28\n",
                offsetof(__disp_layer_info_t, scn_win));
        errors++;
    }

    if (offsetof(__disp_layer_info_t, fb) != 44) {
        fprintf(stderr, "WARNING: offsetof(__disp_layer_info_t, fb) = %zu, expected 44\n",
                offsetof(__disp_layer_info_t, fb));
        errors++;
    }

    if (errors > 0) {
        fprintf(stderr, "\n*** STRUCTURE ALIGNMENT MISMATCH DETECTED ***\n");
        fprintf(stderr, "The structure definitions in this program do not match\n");
        fprintf(stderr, "the kernel's expected layout. This WILL cause crashes\n");
        fprintf(stderr, "when using scaling features.\n\n");
        fprintf(stderr, "Run '%s debug' to see detailed structure information.\n\n",
                "a20_hdmi_fb");
        fprintf(stderr, "Possible causes:\n");
        fprintf(stderr, "  1. Different compiler or compiler flags\n");
        fprintf(stderr, "  2. Different kernel version with modified structures\n");
        fprintf(stderr, "  3. Missing #pragma pack directives\n\n");
    } else {
        SCALE_DEBUG("Structure alignment check PASSED");
    }

    return errors;
}

/* Hexdump utility for debugging */
static void hexdump(const char *desc, const void *addr, size_t len)
{
    const unsigned char *pc = (const unsigned char*)addr;
    size_t i;

    printf("%s (addr=%p, len=%zu):\n", desc, addr, len);
    for (i = 0; i < len; i++) {
        if ((i % 16) == 0) {
            printf("  %04zx: ", i);
        }
        printf("%02x ", pc[i]);
        if ((i % 16) == 15 || i == len - 1) {
            printf("\n");
        }
    }
    fflush(stdout);
}

/* Utility functions */
static int disp_open(void)
{
    if (g_disp_fd >= 0)
        return 0;

    g_disp_fd = open(DISP_DEV, O_RDWR);
    if (g_disp_fd < 0) {
        perror("Failed to open " DISP_DEV);
        return -1;
    }
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
 * The sun7i display driver ioctl interface:
 * - args is a pointer to unsigned long[4]: {screen_id, param1, param2, param3}
 * - The return value of ioctl() contains the result for "get" operations
 * - For commands < 0x280, args[0] must be screen id (0 or 1)
 */
static int disp_ioctl(unsigned int cmd, unsigned long *args)
{
    int ret;
    int saved_errno;

    if (g_disp_fd < 0) {
        fprintf(stderr, "Display device not open\n");
        return -1;
    }

    /* Debug output for scaling-related ioctls */
    if (cmd == DISP_CMD_LAYER_REQUEST || cmd == DISP_CMD_LAYER_SET_PARA ||
        cmd == DISP_CMD_LAYER_OPEN || cmd == DISP_CMD_FB_REQUEST ||
        cmd == DISP_CMD_FB_RELEASE) {
        SCALE_DEBUG("disp_ioctl: cmd=0x%x fd=%d args=%p", cmd, g_disp_fd, (void*)args);
        SCALE_DEBUG("  args[0]=%lu args[1]=%lu args[2]=0x%lx args[3]=%lu",
                    args[0], args[1], args[2], args[3]);
        SCALE_DEBUG("  About to call ioctl()...");
        fflush(stdout);
        fflush(stderr);
    }

    ret = ioctl(g_disp_fd, cmd, args);
    saved_errno = errno;

    /* Debug output for scaling-related ioctls */
    if (cmd == DISP_CMD_LAYER_REQUEST || cmd == DISP_CMD_LAYER_SET_PARA ||
        cmd == DISP_CMD_LAYER_OPEN || cmd == DISP_CMD_FB_REQUEST ||
        cmd == DISP_CMD_FB_RELEASE) {
        SCALE_DEBUG("disp_ioctl: returned %d (errno=%d: %s)", ret, saved_errno, strerror(saved_errno));
    }

    errno = saved_errno;
    return ret;
}

/* Find mode by resolution */
static const mode_info_t* find_mode_by_resolution(uint32_t width, uint32_t height, uint32_t refresh)
{
    for (int i = 0; mode_table[i].name != NULL; i++) {
        if (mode_table[i].width == width && mode_table[i].height == height) {
            if (refresh == 0 || mode_table[i].refresh == refresh) {
                return &mode_table[i];
            }
        }
    }
    return NULL;
}

/* Find mode by name */
static const mode_info_t* find_mode_by_name(const char *name)
{
    for (int i = 0; mode_table[i].name != NULL; i++) {
        if (strcasecmp(mode_table[i].name, name) == 0) {
            return &mode_table[i];
        }
    }
    return NULL;
}

/* Get mode info by mode value */
static const mode_info_t* get_mode_info(disp_tv_mode_t mode)
{
    for (int i = 0; mode_table[i].name != NULL; i++) {
        if (mode_table[i].mode == mode) {
            return &mode_table[i];
        }
    }
    return NULL;
}

/* Convert color depth to format */
static __disp_pixel_fmt_t depth_to_format(int depth)
{
    switch (depth) {
        case 16: return DISP_FORMAT_RGB565;
        case 24: return DISP_FORMAT_RGB888;
        case 32: return DISP_FORMAT_ARGB8888;
        default: return DISP_FORMAT_ARGB8888;
    }
}

/* Get bytes per pixel from format */
static int format_to_bpp(__disp_pixel_fmt_t format)
{
    switch (format) {
        case DISP_FORMAT_RGB565:
        case DISP_FORMAT_ARGB1555:
        case DISP_FORMAT_RGBA5551:
        case DISP_FORMAT_ARGB4444:
            return 2;
        case DISP_FORMAT_RGB888:
            return 3;
        case DISP_FORMAT_ARGB888:
        case DISP_FORMAT_ARGB8888:
            return 4;
        default:
            return 4;
    }
}

/* Check if HDMI is connected */
int hdmi_get_hpd_status(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    int ret;

    /* Try sysfs first */
    FILE *f = fopen(HDMI_STATE, "r");
    if (f) {
        int state = 0;
        if (fscanf(f, "%d", &state) == 1) {
            fclose(f);
            return state;
        }
        fclose(f);
    }

    /* Fall back to ioctl - return value contains the HPD status */
    ret = disp_ioctl(DISP_CMD_HDMI_GET_HPD, args);
    return ret;  /* 1 = connected, 0 = disconnected */
}

/* Check if HDMI mode is supported
 * Note: The driver returns 0 if HPD not ready, or the support value from EDID
 * Returns > 0 if supported, 0 if not supported or HPD not ready
 */
int hdmi_mode_supported(disp_tv_mode_t mode)
{
    unsigned long args[4] = {g_screen, mode, 0, 0};
    int ret = disp_ioctl(DISP_CMD_HDMI_SUPPORT_MODE, args);
    /* Return value > 0 means supported */
    return (ret > 0) ? 1 : 0;
}

/* Get current HDMI mode
 * The ioctl return value contains the current mode number
 */
disp_tv_mode_t hdmi_get_mode(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    int ret = disp_ioctl(DISP_CMD_HDMI_GET_MODE, args);
    if (ret < 0) {
        return (disp_tv_mode_t)-1;
    }
    /* Return value is the mode */
    return (disp_tv_mode_t)ret;
}

/* Disable HDMI */
int hdmi_off(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    int ret = disp_ioctl(DISP_CMD_HDMI_OFF, args);
    if (ret < 0) {
        fprintf(stderr, "hdmi: disable hdmi failed: %d\n", ret);
    }
    return ret;
}

/* Set HDMI mode */
int hdmi_set_mode(disp_tv_mode_t mode)
{
    unsigned long args[4] = {g_screen, mode, 0, 0};
    int ret = disp_ioctl(DISP_CMD_HDMI_SET_MODE, args);
    if (ret < 0) {
        fprintf(stderr, "hdmi: set hdmi output mode failed: %d\n", ret);
    }
    return ret;
}

/* Enable HDMI */
int hdmi_on(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    int ret = disp_ioctl(DISP_CMD_HDMI_ON, args);
    if (ret < 0) {
        fprintf(stderr, "hdmi: enable hdmi failed: %d\n", ret);
    }
    return ret;
}

/* Full HDMI initialization with mode setting */
int hdmi_init(disp_tv_mode_t mode)
{
    int ret;

    /* Check if mode is supported (skip if force flag is set) */
    if (!g_force && !hdmi_mode_supported(mode)) {
        fprintf(stderr, "HDMI mode %d not supported by display (use -f to force)\n", mode);
        return -1;
    }

    /* Disable HDMI first */
    ret = hdmi_off();
    if (ret < 0) {
        /* Continue anyway - might not be on */
    }

    /* Set mode */
    ret = hdmi_set_mode(mode);
    if (ret < 0) {
        return ret;
    }

    /* Enable HDMI */
    ret = hdmi_on();
    if (ret < 0) {
        return ret;
    }

    return 0;
}

/* Get screen dimensions
 * The ioctl return value contains the width/height
 */
int get_screen_size(uint32_t *width, uint32_t *height)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    int ret;

    ret = disp_ioctl(DISP_CMD_SCN_GET_WIDTH, args);
    if (ret < 0) {
        return -1;
    }
    *width = (uint32_t)ret;

    args[0] = g_screen;
    ret = disp_ioctl(DISP_CMD_SCN_GET_HEIGHT, args);
    if (ret < 0) {
        return -1;
    }
    *height = (uint32_t)ret;

    return 0;
}

/* Set screen size (for off-screen rendering) */
int set_screen_size(uint32_t width, uint32_t height)
{
    unsigned long args[4] = {g_screen, width, height, 0};
    return disp_ioctl(DISP_CMD_SET_SCREEN_SIZE, args);
}

/* Get current output type
 * Returns: DISP_OUTPUT_TYPE_NONE/LCD/TV/HDMI/VGA
 */
int get_output_type(void)
{
    unsigned long args[4] = {g_screen, 0, 0, 0};
    return disp_ioctl(DISP_CMD_GET_OUTPUT_TYPE, args);
}

/* Request a layer
 * args = {screen_id, work_mode, 0, 0}
 * returns: layer handle (>= 0) on success, < 0 on error
 */
int layer_request(disp_layer_work_mode_t mode)
{
    unsigned long args[4] = {g_screen, (unsigned long)mode, 0, 0};
    int ret;

    SCALE_DEBUG("layer_request: screen=%lu, mode=%lu", args[0], args[1]);
    SCALE_DEBUG("  ioctl args array at %p: {%lu, %lu, %lu, %lu}",
                (void*)args, args[0], args[1], args[2], args[3]);

    ret = disp_ioctl(DISP_CMD_LAYER_REQUEST, args);

    SCALE_DEBUG("layer_request: ioctl returned %d (errno=%d)", ret, errno);

    if (ret < 0) {
        perror("layer request failed");
        return -1;
    }
    SCALE_DEBUG("layer_request: got layer handle %d", ret);
    return ret;  /* Returns layer handle */
}

/* Release a layer
 * args = {screen_id, layer_hdl, 0, 0}
 */
int layer_release(int layer_hdl)
{
    unsigned long args[4] = {g_screen, (unsigned long)layer_hdl, 0, 0};
    return disp_ioctl(DISP_CMD_LAYER_RELEASE, args);
}

/* Open (enable) a layer
 * args = {screen_id, layer_hdl, 0, 0}
 */
int layer_open(int layer_hdl)
{
    unsigned long args[4] = {g_screen, (unsigned long)layer_hdl, 0, 0};
    int ret;

    SCALE_DEBUG("layer_open: screen=%lu, hdl=%lu", args[0], args[1]);
    SCALE_DEBUG("  Calling ioctl 0x%x NOW...", DISP_CMD_LAYER_OPEN);

    ret = disp_ioctl(DISP_CMD_LAYER_OPEN, args);

    SCALE_DEBUG("layer_open: ioctl returned %d (errno=%d)", ret, errno);
    return ret;
}

/* Close (disable) a layer
 * args = {screen_id, layer_hdl, 0, 0}
 */
int layer_close(int layer_hdl)
{
    unsigned long args[4] = {g_screen, (unsigned long)layer_hdl, 0, 0};
    return disp_ioctl(DISP_CMD_LAYER_CLOSE, args);
}

/* Set layer parameters
 * args = {screen_id, layer_hdl, info_ptr, 0}
 */
int layer_set_para(int layer_hdl, disp_layer_info_t *info)
{
    unsigned long args[4] = {g_screen, (unsigned long)layer_hdl, (unsigned long)info, 0};
    int ret;

    SCALE_DEBUG("layer_set_para: screen=%lu, hdl=%lu, info=%p",
                args[0], args[1], (void*)args[2]);
    SCALE_DEBUG("  ioctl args array at %p", (void*)args);

    if (info == NULL) {
        SCALE_DEBUG("ERROR: layer_set_para called with NULL info pointer!");
        return -1;
    }

    SCALE_DEBUG("  info->mode = %d (offset %zu)", info->mode, offsetof(disp_layer_info_t, mode));
    SCALE_DEBUG("  Calling ioctl 0x%x NOW...", DISP_CMD_LAYER_SET_PARA);

    ret = disp_ioctl(DISP_CMD_LAYER_SET_PARA, args);

    SCALE_DEBUG("layer_set_para: ioctl returned %d (errno=%d)", ret, errno);
    return ret;
}

/* Get layer parameters
 * args = {screen_id, layer_hdl, info_ptr, 0}
 */
int layer_get_para(int layer_hdl, disp_layer_info_t *info)
{
    unsigned long args[4] = {g_screen, (unsigned long)layer_hdl, (unsigned long)info, 0};
    return disp_ioctl(DISP_CMD_LAYER_GET_PARA, args);
}

/* Set source window (framebuffer region)
 * args = {screen_id, layer_hdl, rect_ptr, 0}
 */
int layer_set_src_window(int layer_hdl, disp_rect_t *rect)
{
    unsigned long args[4] = {g_screen, (unsigned long)layer_hdl, (unsigned long)rect, 0};
    return disp_ioctl(DISP_CMD_LAYER_SET_SRC_WIN, args);
}

/* Set screen window (output region)
 * args = {screen_id, layer_hdl, rect_ptr, 0}
 */
int layer_set_scn_window(int layer_hdl, disp_rect_t *rect)
{
    unsigned long args[4] = {g_screen, (unsigned long)layer_hdl, (unsigned long)rect, 0};
    return disp_ioctl(DISP_CMD_LAYER_SET_SCN_WIN, args);
}

/* Set layer framebuffer
 * args = {screen_id, layer_hdl, fb_ptr, 0}
 */
int layer_set_fb(int layer_hdl, disp_fb_t *fb)
{
    unsigned long args[4] = {g_screen, (unsigned long)layer_hdl, (unsigned long)fb, 0};
    return disp_ioctl(DISP_CMD_LAYER_SET_FB, args);
}

/* Request framebuffer through display driver */
int fb_request(uint32_t fb_id, disp_fb_create_para_t *para)
{
    unsigned long args[4] = {fb_id, (unsigned long)para, 0, 0};
    int ret;

    SCALE_DEBUG("fb_request: fb_id=%u para=%p", fb_id, (void*)para);
    SCALE_DEBUG("  args array at %p: {%lu, 0x%lx, 0, 0}", (void*)args, args[0], args[1]);

    if (para == NULL) {
        SCALE_DEBUG("ERROR: fb_request called with NULL para pointer!");
        return -1;
    }

    SCALE_DEBUG("  para->fb_mode = %d", para->fb_mode);
    SCALE_DEBUG("  para->mode = %d (0=NORMAL, 4=SCALER)", para->mode);
    SCALE_DEBUG("  para->buffer_num = %u", para->buffer_num);
    SCALE_DEBUG("  para->width = %u, para->height = %u", para->width, para->height);
    SCALE_DEBUG("  para->output_width = %u, para->output_height = %u",
                para->output_width, para->output_height);
    SCALE_DEBUG("  para->primary_screen_id = %u", para->primary_screen_id);

    SCALE_DEBUG("Calling disp_ioctl(0x%x, args)...", DISP_CMD_FB_REQUEST);

    ret = disp_ioctl(DISP_CMD_FB_REQUEST, args);

    SCALE_DEBUG("fb_request: disp_ioctl returned %d (errno=%d: %s)",
                ret, errno, strerror(errno));

    return ret;
}

/* Release framebuffer */
int fb_release(uint32_t fb_id)
{
    unsigned long args[4] = {fb_id, 0, 0, 0};
    int ret;

    SCALE_DEBUG("fb_release: fb_id=%u", fb_id);
    SCALE_DEBUG("  args array at %p: {%lu, 0, 0, 0}", (void*)args, args[0]);
    SCALE_DEBUG("Calling disp_ioctl(0x%x, args)...", DISP_CMD_FB_RELEASE);

    ret = disp_ioctl(DISP_CMD_FB_RELEASE, args);

    SCALE_DEBUG("fb_release: disp_ioctl returned %d (errno=%d: %s)",
                ret, errno, strerror(errno));

    return ret;
}

/* Get framebuffer parameters */
int fb_get_para(uint32_t fb_id, disp_fb_create_para_t *para)
{
    unsigned long args[4] = {fb_id, (unsigned long)para, 0, 0};
    return disp_ioctl(DISP_CMD_FB_GET_PARA, args);
}

/* Configure framebuffer through /dev/fb0 */
int fb_configure(uint32_t width, uint32_t height, int depth)
{
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;

    g_fb_fd = open(FB_DEV, O_RDWR);
    if (g_fb_fd < 0) {
        perror("Failed to open " FB_DEV);
        return -1;
    }

    /* Get current settings */
    if (ioctl(g_fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0) {
        perror("FBIOGET_VSCREENINFO failed");
        close(g_fb_fd);
        g_fb_fd = -1;
        return -1;
    }

    /* Set new resolution and depth */
    vinfo.xres = width;
    vinfo.yres = height;
    vinfo.xres_virtual = width;
    vinfo.yres_virtual = height;
    vinfo.bits_per_pixel = depth;

    /* Set color format based on depth */
    if (depth == 16) {
        vinfo.red.offset = 11;
        vinfo.red.length = 5;
        vinfo.green.offset = 5;
        vinfo.green.length = 6;
        vinfo.blue.offset = 0;
        vinfo.blue.length = 5;
        vinfo.transp.offset = 0;
        vinfo.transp.length = 0;
    } else if (depth == 24 || depth == 32) {
        vinfo.red.offset = 16;
        vinfo.red.length = 8;
        vinfo.green.offset = 8;
        vinfo.green.length = 8;
        vinfo.blue.offset = 0;
        vinfo.blue.length = 8;
        vinfo.transp.offset = (depth == 32) ? 24 : 0;
        vinfo.transp.length = (depth == 32) ? 8 : 0;
    }

    if (ioctl(g_fb_fd, FBIOPUT_VSCREENINFO, &vinfo) < 0) {
        perror("FBIOPUT_VSCREENINFO failed");
        close(g_fb_fd);
        g_fb_fd = -1;
        return -1;
    }

    /* Re-read to confirm */
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

/*
 * Setup framebuffer with hardware scaling using DISP_CMD_FB_REQUEST.
 * This is the proper way to set up scaling on sun7i - it creates a
 * framebuffer with an associated layer configured for scaling.
 *
 * The key is using mode=DISP_LAYER_WORK_MODE_SCALER with different
 * width/height vs output_width/output_height values.
 */
int setup_fb_with_scaling(uint32_t fb_id, uint32_t fb_width, uint32_t fb_height,
                          uint32_t scn_width, uint32_t scn_height, int depth)
{
    disp_fb_create_para_t fb_para;
    int ret;
    int needs_scaling = (fb_width != scn_width || fb_height != scn_height);

    SCALE_DEBUG("=== ENTRY ===");
    SCALE_DEBUG("fb_id=%u fb=%ux%u scn=%ux%u depth=%d",
                fb_id, fb_width, fb_height, scn_width, scn_height, depth);
    SCALE_DEBUG("needs_scaling=%d", needs_scaling);

    SCALE_DEBUG("sizeof(disp_fb_create_para_t) = %zu (expected: 56)", sizeof(disp_fb_create_para_t));
    SCALE_DEBUG("g_disp_fd = %d", g_disp_fd);

    if (g_disp_fd < 0) {
        SCALE_DEBUG("ERROR: Display device not open!");
        return -1;
    }

    /* Validate input parameters */
    SCALE_DEBUG("Validating input parameters...");
    if (fb_width == 0 || fb_height == 0) {
        SCALE_DEBUG("ERROR: Invalid framebuffer dimensions: %ux%u", fb_width, fb_height);
        return -1;
    }
    if (scn_width == 0 || scn_height == 0) {
        SCALE_DEBUG("ERROR: Invalid screen dimensions: %ux%u", scn_width, scn_height);
        return -1;
    }
    if (depth != 16 && depth != 24 && depth != 32) {
        SCALE_DEBUG("ERROR: Invalid depth: %d", depth);
        return -1;
    }
    SCALE_DEBUG("Input parameters validated OK");

    /* First release the existing framebuffer if any */
    SCALE_DEBUG("Releasing existing fb%u...", fb_id);
    ret = fb_release(fb_id);
    SCALE_DEBUG("fb_release returned: %d (may fail if not allocated, that's OK)", ret);

    /* Set up framebuffer creation parameters */
    SCALE_DEBUG("Initializing fb_para structure at %p...", (void*)&fb_para);
    SCALE_DEBUG("About to call memset...");
    memset(&fb_para, 0, sizeof(fb_para));
    SCALE_DEBUG("memset completed");

    SCALE_DEBUG("Setting fb_para.fb_mode = %d (FB_MODE_SCREEN0)", FB_MODE_SCREEN0);
    fb_para.fb_mode = FB_MODE_SCREEN0;  /* fb for screen 0 */

    SCALE_DEBUG("Setting fb_para.mode = %d (%s)",
                needs_scaling ? DISP_LAYER_WORK_MODE_SCALER : DISP_LAYER_WORK_MODE_NORMAL,
                needs_scaling ? "SCALER" : "NORMAL");
    fb_para.mode = needs_scaling ? DISP_LAYER_WORK_MODE_SCALER :
                                   DISP_LAYER_WORK_MODE_NORMAL;

    SCALE_DEBUG("Setting fb_para.buffer_num = 1");
    fb_para.buffer_num = 1;             /* Single buffer */

    SCALE_DEBUG("Setting fb_para.width = %u", fb_width);
    fb_para.width = fb_width;           /* Framebuffer dimensions */

    SCALE_DEBUG("Setting fb_para.height = %u", fb_height);
    fb_para.height = fb_height;

    SCALE_DEBUG("Setting fb_para.output_width = %u", scn_width);
    fb_para.output_width = scn_width;   /* Output (screen) dimensions */

    SCALE_DEBUG("Setting fb_para.output_height = %u", scn_height);
    fb_para.output_height = scn_height;

    SCALE_DEBUG("Setting fb_para.primary_screen_id = %u", g_screen);
    fb_para.primary_screen_id = g_screen;

    SCALE_DEBUG("fb_para structure filled:");
    SCALE_DEBUG("  fb_mode=%d mode=%d buffer_num=%u",
                fb_para.fb_mode, fb_para.mode, fb_para.buffer_num);
    SCALE_DEBUG("  width=%u height=%u output_width=%u output_height=%u",
                fb_para.width, fb_para.height, fb_para.output_width, fb_para.output_height);
    SCALE_DEBUG("  primary_screen_id=%u", fb_para.primary_screen_id);
    SCALE_DEBUG("  aux_output_width=%u aux_output_height=%u",
                fb_para.aux_output_width, fb_para.aux_output_height);
    SCALE_DEBUG("  line_length=%u smem_len=%u",
                fb_para.line_length, fb_para.smem_len);
    SCALE_DEBUG("  ch1_offset=%u ch2_offset=%u",
                fb_para.ch1_offset, fb_para.ch2_offset);

    /* Hexdump the structure for debugging */
    hexdump("fb_para raw data", &fb_para, sizeof(fb_para));

    SCALE_DEBUG("About to call fb_request (ioctl 0x%x) with:", DISP_CMD_FB_REQUEST);
    SCALE_DEBUG("  fb_id=%u para=%p", fb_id, (void*)&fb_para);
    SCALE_DEBUG("Calling fb_request NOW...");

    ret = fb_request(fb_id, &fb_para);

    SCALE_DEBUG("fb_request returned: %d (errno=%d: %s)", ret, errno, strerror(errno));

    if (ret < 0) {
        SCALE_DEBUG("ERROR: fb_request failed!");
        fprintf(stderr, "Failed to request framebuffer with scaling (errno=%d: %s)\n",
                errno, strerror(errno));
        return -1;
    }

    SCALE_DEBUG("fb_request succeeded!");

    /* Read back the parameters to see what the kernel set */
    SCALE_DEBUG("Reading back fb_para after request...");
    SCALE_DEBUG("  line_length=%u smem_len=%u (may be filled by kernel)",
                fb_para.line_length, fb_para.smem_len);

    if (needs_scaling) {
        SCALE_DEBUG("Hardware scaling enabled: %dx%d -> %dx%d",
                    fb_width, fb_height, scn_width, scn_height);
        printf("Hardware scaling enabled: %dx%d -> %dx%d\n",
               fb_width, fb_height, scn_width, scn_height);
        printf("NOTE: Scaling mode is incompatible with Mali/EGL apps. Run 'noscale' first.\n");
    } else {
        SCALE_DEBUG("Framebuffer configured: %dx%d (no scaling)", fb_width, fb_height);
        printf("Framebuffer configured: %dx%d (no scaling)\n", fb_width, fb_height);
    }

    SCALE_DEBUG("=== EXIT (success) ===");
    return 0;
}

/*
 * Alternative approach: Setup scaling by directly configuring the layer
 * associated with the existing framebuffer. This works by:
 * 1. First configuring fb via standard fbdev
 * 2. Then using layer ioctls to modify the display parameters
 *
 * Note: This approach may cause crashes if the kernel structure layout
 * doesn't match. Use setup_fb_with_scaling() as the preferred method.
 */
int setup_scaling_layer(uint32_t fb_width, uint32_t fb_height,
                        uint32_t scn_width, uint32_t scn_height,
                        int depth, uint32_t fb_phys_addr)
{
    int layer_hdl;
    disp_layer_info_t layer_info;
    int ret;
    int needs_scaling = (fb_width != scn_width || fb_height != scn_height);

    SCALE_DEBUG("=== ENTRY ===");
    SCALE_DEBUG("fb=%ux%u scn=%ux%u depth=%d phys=0x%08x",
                fb_width, fb_height, scn_width, scn_height, depth, fb_phys_addr);
    SCALE_DEBUG("needs_scaling=%d", needs_scaling);

    SCALE_DEBUG("Structure sizes:");
    SCALE_DEBUG("  sizeof(disp_layer_info_t) = %zu (expected: 116)", sizeof(disp_layer_info_t));
    SCALE_DEBUG("  sizeof(disp_fb_t) = %zu (expected: 64)", sizeof(disp_fb_t));
    SCALE_DEBUG("  sizeof(disp_rect_t) = %zu (expected: 16)", sizeof(disp_rect_t));
    SCALE_DEBUG("  sizeof(disp_rectsz_t) = %zu (expected: 8)", sizeof(disp_rectsz_t));

    SCALE_DEBUG("g_disp_fd = %d", g_disp_fd);

    if (g_disp_fd < 0) {
        SCALE_DEBUG("ERROR: Display device not open!");
        return -1;
    }

    /* Validate input parameters */
    SCALE_DEBUG("Validating input parameters...");
    if (fb_phys_addr == 0) {
        SCALE_DEBUG("ERROR: framebuffer physical address is 0");
        fprintf(stderr, "Error: framebuffer physical address is 0\n");
        return -1;
    }
    if (fb_width == 0 || fb_height == 0) {
        SCALE_DEBUG("ERROR: Invalid framebuffer dimensions: %ux%u", fb_width, fb_height);
        return -1;
    }
    if (scn_width == 0 || scn_height == 0) {
        SCALE_DEBUG("ERROR: Invalid screen dimensions: %ux%u", scn_width, scn_height);
        return -1;
    }
    SCALE_DEBUG("Input parameters validated OK");

    /* Zero the layer_info structure */
    SCALE_DEBUG("layer_info at stack address %p", (void*)&layer_info);
    SCALE_DEBUG("About to memset layer_info (%zu bytes)...", sizeof(layer_info));
    memset(&layer_info, 0, sizeof(layer_info));
    SCALE_DEBUG("layer_info zeroed");

    /* Request layer - use scaler mode if scaling needed */
    SCALE_DEBUG("About to request layer:");
    SCALE_DEBUG("  ioctl cmd = 0x%x (DISP_CMD_LAYER_REQUEST)", DISP_CMD_LAYER_REQUEST);
    SCALE_DEBUG("  work_mode = %d (%s)",
                needs_scaling ? DISP_LAYER_WORK_MODE_SCALER : DISP_LAYER_WORK_MODE_NORMAL,
                needs_scaling ? "SCALER" : "NORMAL");
    SCALE_DEBUG("Calling layer_request NOW...");

    layer_hdl = layer_request(needs_scaling ? DISP_LAYER_WORK_MODE_SCALER :
                                              DISP_LAYER_WORK_MODE_NORMAL);

    SCALE_DEBUG("layer_request returned: %d (errno=%d)", layer_hdl, errno);

    if (layer_hdl < 0) {
        SCALE_DEBUG("ERROR: Failed to request layer");
        fprintf(stderr, "Failed to request layer (errno=%d: %s)\n", errno, strerror(errno));
        return -1;
    }
    SCALE_DEBUG("Layer handle obtained: %d", layer_hdl);

    /* Configure layer - each field individually with debug */
    SCALE_DEBUG("Configuring layer_info fields...");

    SCALE_DEBUG("Setting layer_info.mode = %d", needs_scaling ? DISP_LAYER_WORK_MODE_SCALER : DISP_LAYER_WORK_MODE_NORMAL);
    layer_info.mode = needs_scaling ? DISP_LAYER_WORK_MODE_SCALER :
                                      DISP_LAYER_WORK_MODE_NORMAL;

    SCALE_DEBUG("Setting layer_info.b_from_screen = 0");
    layer_info.b_from_screen = 0;

    SCALE_DEBUG("Setting layer_info.pipe = 0");
    layer_info.pipe = 0;

    SCALE_DEBUG("Setting layer_info.prio = 0");
    layer_info.prio = 0;

    SCALE_DEBUG("Setting layer_info.alpha_en = 0");
    layer_info.alpha_en = 0;

    SCALE_DEBUG("Setting layer_info.alpha_val = 0xff");
    layer_info.alpha_val = 0xff;

    SCALE_DEBUG("Setting layer_info.ck_enable = 0");
    layer_info.ck_enable = 0;

    /* Source window (framebuffer content) */
    SCALE_DEBUG("Setting layer_info.src_win (framebuffer region):");
    SCALE_DEBUG("  x=0, y=0, width=%u, height=%u", fb_width, fb_height);
    layer_info.src_win.x = 0;
    layer_info.src_win.y = 0;
    layer_info.src_win.width = fb_width;
    layer_info.src_win.height = fb_height;

    /* Screen window (output position/size) */
    SCALE_DEBUG("Setting layer_info.scn_win (screen output region):");
    SCALE_DEBUG("  x=0, y=0, width=%u, height=%u", scn_width, scn_height);
    layer_info.scn_win.x = 0;
    layer_info.scn_win.y = 0;
    layer_info.scn_win.width = scn_width;
    layer_info.scn_win.height = scn_height;

    /* Framebuffer settings */
    SCALE_DEBUG("Setting layer_info.fb fields:");

    SCALE_DEBUG("  fb.addr[0] = 0x%08x (physical address)", fb_phys_addr);
    layer_info.fb.addr[0] = fb_phys_addr;
    layer_info.fb.addr[1] = 0;
    layer_info.fb.addr[2] = 0;

    SCALE_DEBUG("  fb.size.width = %u, fb.size.height = %u", fb_width, fb_height);
    layer_info.fb.size.width = fb_width;
    layer_info.fb.size.height = fb_height;

    SCALE_DEBUG("  fb.format = %d (from depth_to_format(%d))", depth_to_format(depth), depth);
    layer_info.fb.format = depth_to_format(depth);

    SCALE_DEBUG("  fb.seq = %d (%s)", (depth == 32) ? DISP_SEQ_ARGB : DISP_SEQ_P3210,
                (depth == 32) ? "DISP_SEQ_ARGB" : "DISP_SEQ_P3210");
    layer_info.fb.seq = (depth == 32) ? DISP_SEQ_ARGB : DISP_SEQ_P3210;

    SCALE_DEBUG("  fb.mode = %d (DISP_MOD_INTERLEAVED)", DISP_MOD_INTERLEAVED);
    layer_info.fb.mode = DISP_MOD_INTERLEAVED;

    SCALE_DEBUG("  fb.br_swap = 0");
    layer_info.fb.br_swap = 0;

    SCALE_DEBUG("  fb.cs_mode = %d (DISP_BT601)", DISP_BT601);
    layer_info.fb.cs_mode = DISP_BT601;

    SCALE_DEBUG("  fb.b_trd_src = 0");
    layer_info.fb.b_trd_src = 0;

    SCALE_DEBUG("  fb.trd_mode = %d (DISP_3D_SRC_MODE_TB)", DISP_3D_SRC_MODE_TB);
    layer_info.fb.trd_mode = DISP_3D_SRC_MODE_TB;

    SCALE_DEBUG("  fb.trd_right_addr[0,1,2] = 0");
    layer_info.fb.trd_right_addr[0] = 0;
    layer_info.fb.trd_right_addr[1] = 0;
    layer_info.fb.trd_right_addr[2] = 0;

    SCALE_DEBUG("  fb.pre_multiply = 0");
    layer_info.fb.pre_multiply = 0;

    SCALE_DEBUG("Setting layer_info 3D output fields:");
    SCALE_DEBUG("  b_trd_out = 0");
    layer_info.b_trd_out = 0;

    SCALE_DEBUG("  out_trd_mode = %d (DISP_3D_OUT_MODE_TB)", DISP_3D_OUT_MODE_TB);
    layer_info.out_trd_mode = DISP_3D_OUT_MODE_TB;

    SCALE_DEBUG("layer_info structure filled completely");

    /* Hexdump the structure for debugging */
    hexdump("layer_info raw data", &layer_info, sizeof(layer_info));

    SCALE_DEBUG("About to call layer_set_para:");
    SCALE_DEBUG("  ioctl cmd = 0x%x (DISP_CMD_LAYER_SET_PARA)", DISP_CMD_LAYER_SET_PARA);
    SCALE_DEBUG("  layer_hdl = %d", layer_hdl);
    SCALE_DEBUG("  layer_info ptr = %p", (void*)&layer_info);

    SCALE_DEBUG("Summary of key layer_info values:");
    SCALE_DEBUG("  mode = %d", layer_info.mode);
    SCALE_DEBUG("  src_win = (%d,%d) %ux%u",
                layer_info.src_win.x, layer_info.src_win.y,
                layer_info.src_win.width, layer_info.src_win.height);
    SCALE_DEBUG("  scn_win = (%d,%d) %ux%u",
                layer_info.scn_win.x, layer_info.scn_win.y,
                layer_info.scn_win.width, layer_info.scn_win.height);
    SCALE_DEBUG("  fb.addr[0] = 0x%08x", layer_info.fb.addr[0]);
    SCALE_DEBUG("  fb.size = %ux%u", layer_info.fb.size.width, layer_info.fb.size.height);
    SCALE_DEBUG("  fb.format = %d, fb.seq = %d, fb.mode = %d",
                layer_info.fb.format, layer_info.fb.seq, layer_info.fb.mode);

    SCALE_DEBUG("Calling layer_set_para NOW...");
    ret = layer_set_para(layer_hdl, &layer_info);
    SCALE_DEBUG("layer_set_para returned: %d (errno=%d: %s)", ret, errno, strerror(errno));

    if (ret < 0) {
        SCALE_DEBUG("ERROR: Failed to set layer parameters");
        fprintf(stderr, "Failed to set layer parameters (errno=%d: %s)\n",
                errno, strerror(errno));
        SCALE_DEBUG("Releasing layer %d...", layer_hdl);
        layer_release(layer_hdl);
        return -1;
    }
    SCALE_DEBUG("layer_set_para succeeded");

    /* Open (enable) the layer */
    SCALE_DEBUG("About to call layer_open:");
    SCALE_DEBUG("  ioctl cmd = 0x%x (DISP_CMD_LAYER_OPEN)", DISP_CMD_LAYER_OPEN);
    SCALE_DEBUG("  layer_hdl = %d", layer_hdl);

    SCALE_DEBUG("Calling layer_open NOW...");
    ret = layer_open(layer_hdl);
    SCALE_DEBUG("layer_open returned: %d (errno=%d: %s)", ret, errno, strerror(errno));

    if (ret < 0) {
        SCALE_DEBUG("ERROR: Failed to open layer");
        fprintf(stderr, "Failed to open layer (errno=%d: %s)\n", errno, strerror(errno));
        SCALE_DEBUG("Releasing layer %d...", layer_hdl);
        layer_release(layer_hdl);
        return -1;
    }
    SCALE_DEBUG("layer_open succeeded");

    if (needs_scaling) {
        SCALE_DEBUG("Hardware scaling enabled: %dx%d -> %dx%d",
                    fb_width, fb_height, scn_width, scn_height);
        printf("Hardware scaling enabled: %dx%d -> %dx%d\n",
               fb_width, fb_height, scn_width, scn_height);
        printf("NOTE: Scaling mode is incompatible with Mali/EGL apps. Run 'noscale' first.\n");
    }

    SCALE_DEBUG("=== EXIT (success, layer_hdl=%d) ===", layer_hdl);
    return layer_hdl;
}

/* Print usage */
void print_usage(const char *prog)
{
    printf("A20 HDMI and Framebuffer Control Utility\n\n");
    printf("Usage: %s [-v] [-f] [-s screen] <command> [options]\n\n", prog);
    printf("Options:\n");
    printf("  -v                            Verbose output (show debug messages)\n");
    printf("  -f                            Force mode setting (bypass EDID check)\n");
    printf("  -s <screen>                   Select screen (0 or 1, default: 0)\n\n");
    printf("Commands:\n");
    printf("  info                          Show current display and framebuffer info\n");
    printf("  debug                         Show structure sizes for debugging\n");
    printf("  hdmi on                       Enable HDMI output\n");
    printf("  hdmi off                      Disable HDMI output\n");
    printf("  hdmi mode <name|num>          Set HDMI mode by name or number\n");
    printf("  hdmi init <W>x<H>[@Hz]        Initialize HDMI with resolution\n");
    printf("  fb set <W>x<H>x<depth>        Set framebuffer resolution and depth\n");
    printf("  scale <fbW>x<fbH> <scnW>x<scnH> <depth>  Setup framebuffer with scaling\n");
    printf("  scale2 <fbW>x<fbH> <scnW>x<scnH> <depth> Alternative method (for testing)\n");
    printf("  autoscale [depth]             Scale current FB to current screen size\n");
    printf("  noscale [depth]               Disable scaling (set FB to screen size)\n");
    printf("\nHDMI mode numbers (for 'hdmi mode <num>'):\n");
    printf("  Num  Name      Resolution\n");
    printf("  ---  --------  -----------\n");
    for (int i = 0; mode_table[i].name != NULL; i++) {
        printf("  %2d   %-8s  %4dx%d @%dHz\n",
               mode_table[i].mode, mode_table[i].name,
               mode_table[i].width, mode_table[i].height,
               mode_table[i].refresh);
    }
    printf("\nSupported color depths: 16, 24, 32\n");
    printf("\nExamples:\n");
    printf("  %s info                      # Show display and FB info\n", prog);
    printf("  %s hdmi mode 720p60         # Set HDMI to 720p60 by name\n", prog);
    printf("  %s -f hdmi mode 576p        # Force 576p even if EDID says unsupported\n", prog);
    printf("  %s hdmi init 1280x720@60    # Init HDMI at 720p60\n", prog);
    printf("  %s fb set 640x480x32        # Set FB to 640x480 32bpp\n", prog);
    printf("  %s scale 640x480 1280x720 32 # FB 640x480 scaled to 720p\n", prog);
    printf("  %s autoscale                # Scale current FB to screen (keep depth)\n", prog);
    printf("  %s autoscale 32             # Scale current FB to screen at 32bpp\n", prog);
    printf("  %s noscale                  # Disable scaling, set FB to screen size\n", prog);
    printf("  %s -v scale 640x480 1280x720 32 # Verbose scaling with debug output\n", prog);
    printf("\nNotes:\n");
    printf("  - 'scale' uses FB_REQUEST ioctl (recommended method)\n");
    printf("  - 'scale2' uses manual layer setup (alternative method)\n");
    printf("  - 'autoscale' reads current FB size and scales to current screen size\n");
    printf("  - 'noscale' sets FB resolution to match screen (1:1, no scaling)\n");
    printf("\nWARNING: Scaling mode (DISP_LAYER_WORK_MODE_SCALER) is incompatible with\n");
    printf("  Mali GPU / EGL applications. Run 'noscale' before starting EGL apps.\n");
}

/* Show structure sizes for debugging */
void show_debug_info(void)
{
    printf("=== Structure Size Debug Info ===\n\n");
    printf("Basic types:\n");
    printf("  sizeof(__bool)           = %zu (expected: 1)\n", sizeof(__bool));
    printf("  sizeof(__u8)             = %zu (expected: 1)\n", sizeof(__u8));
    printf("  sizeof(__u16)            = %zu (expected: 2)\n", sizeof(__u16));
    printf("  sizeof(__u32)            = %zu (expected: 4)\n", sizeof(__u32));
    printf("  sizeof(__s32)            = %zu (expected: 4)\n", sizeof(__s32));
    printf("  sizeof(enum)             = %zu (expected: 4)\n", sizeof(__disp_pixel_fmt_t));
    printf("\nDisplay structures:\n");
    printf("  sizeof(__disp_rect_t)       = %zu (expected: 16)\n", sizeof(__disp_rect_t));
    printf("  sizeof(__disp_rectsz_t)     = %zu (expected: 8)\n", sizeof(__disp_rectsz_t));
    printf("  sizeof(__disp_fb_t)         = %zu (expected: 64)\n", sizeof(__disp_fb_t));
    printf("  sizeof(__disp_layer_info_t) = %zu (expected: 116)\n", sizeof(__disp_layer_info_t));
    printf("  sizeof(__disp_fb_create_para_t) = %zu (expected: 56)\n", sizeof(__disp_fb_create_para_t));

    printf("\n__disp_fb_t field offsets:\n");
    printf("  offsetof(addr)           = %zu (expected: 0)\n", offsetof(__disp_fb_t, addr));
    printf("  offsetof(size)           = %zu (expected: 12)\n", offsetof(__disp_fb_t, size));
    printf("  offsetof(format)         = %zu (expected: 20)\n", offsetof(__disp_fb_t, format));
    printf("  offsetof(seq)            = %zu (expected: 24)\n", offsetof(__disp_fb_t, seq));
    printf("  offsetof(mode)           = %zu (expected: 28)\n", offsetof(__disp_fb_t, mode));
    printf("  offsetof(br_swap)        = %zu (expected: 32)\n", offsetof(__disp_fb_t, br_swap));
    printf("  offsetof(cs_mode)        = %zu (expected: 36)\n", offsetof(__disp_fb_t, cs_mode));
    printf("  offsetof(b_trd_src)      = %zu (expected: 40)\n", offsetof(__disp_fb_t, b_trd_src));
    printf("  offsetof(trd_mode)       = %zu (expected: 44)\n", offsetof(__disp_fb_t, trd_mode));
    printf("  offsetof(trd_right_addr) = %zu (expected: 48)\n", offsetof(__disp_fb_t, trd_right_addr));
    printf("  offsetof(pre_multiply)   = %zu (expected: 60)\n", offsetof(__disp_fb_t, pre_multiply));

    printf("\n__disp_layer_info_t field offsets:\n");
    printf("  offsetof(mode)           = %zu (expected: 0)\n", offsetof(__disp_layer_info_t, mode));
    printf("  offsetof(b_from_screen)  = %zu (expected: 4)\n", offsetof(__disp_layer_info_t, b_from_screen));
    printf("  offsetof(pipe)           = %zu (expected: 5)\n", offsetof(__disp_layer_info_t, pipe));
    printf("  offsetof(prio)           = %zu (expected: 6)\n", offsetof(__disp_layer_info_t, prio));
    printf("  offsetof(alpha_en)       = %zu (expected: 7)\n", offsetof(__disp_layer_info_t, alpha_en));
    printf("  offsetof(alpha_val)      = %zu (expected: 8)\n", offsetof(__disp_layer_info_t, alpha_val));
    printf("  offsetof(ck_enable)      = %zu (expected: 10)\n", offsetof(__disp_layer_info_t, ck_enable));
    printf("  offsetof(src_win)        = %zu (expected: 12)\n", offsetof(__disp_layer_info_t, src_win));
    printf("  offsetof(scn_win)        = %zu (expected: 28)\n", offsetof(__disp_layer_info_t, scn_win));
    printf("  offsetof(fb)             = %zu (expected: 44)\n", offsetof(__disp_layer_info_t, fb));
    printf("  offsetof(b_trd_out)      = %zu (expected: 108)\n", offsetof(__disp_layer_info_t, b_trd_out));
    printf("  offsetof(out_trd_mode)   = %zu (expected: 112)\n", offsetof(__disp_layer_info_t, out_trd_mode));

    printf("\n__disp_fb_create_para_t field offsets:\n");
    printf("  offsetof(fb_mode)            = %zu (expected: 0)\n", offsetof(__disp_fb_create_para_t, fb_mode));
    printf("  offsetof(mode)               = %zu (expected: 4)\n", offsetof(__disp_fb_create_para_t, mode));
    printf("  offsetof(buffer_num)         = %zu (expected: 8)\n", offsetof(__disp_fb_create_para_t, buffer_num));
    printf("  offsetof(width)              = %zu (expected: 12)\n", offsetof(__disp_fb_create_para_t, width));
    printf("  offsetof(height)             = %zu (expected: 16)\n", offsetof(__disp_fb_create_para_t, height));
    printf("  offsetof(output_width)       = %zu (expected: 20)\n", offsetof(__disp_fb_create_para_t, output_width));
    printf("  offsetof(output_height)      = %zu (expected: 24)\n", offsetof(__disp_fb_create_para_t, output_height));
    printf("  offsetof(primary_screen_id)  = %zu (expected: 28)\n", offsetof(__disp_fb_create_para_t, primary_screen_id));
    printf("  offsetof(aux_output_width)   = %zu (expected: 32)\n", offsetof(__disp_fb_create_para_t, aux_output_width));
    printf("  offsetof(aux_output_height)  = %zu (expected: 36)\n", offsetof(__disp_fb_create_para_t, aux_output_height));
    printf("  offsetof(line_length)        = %zu (expected: 40)\n", offsetof(__disp_fb_create_para_t, line_length));
    printf("  offsetof(smem_len)           = %zu (expected: 44)\n", offsetof(__disp_fb_create_para_t, smem_len));
    printf("  offsetof(ch1_offset)         = %zu (expected: 48)\n", offsetof(__disp_fb_create_para_t, ch1_offset));
    printf("  offsetof(ch2_offset)         = %zu (expected: 52)\n", offsetof(__disp_fb_create_para_t, ch2_offset));

    printf("\nIf sizes/offsets don't match expected values, structure alignment is wrong.\n");
    printf("This can cause crashes when passing structures to kernel ioctls.\n");
}

/* Show display info */
/* Get framebuffer info and store in provided structures */
int get_fb_info(struct fb_var_screeninfo *vinfo, struct fb_fix_screeninfo *finfo)
{
    int fd = open(FB_DEV, O_RDONLY);
    if (fd < 0) {
        return -1;
    }

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

void show_info(void)
{
    uint32_t width, height;
    disp_tv_mode_t mode;
    const mode_info_t *info;
    int hpd, output_type;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;

    printf("=== A20 Display Information ===\n\n");

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

    hpd = hdmi_get_hpd_status();
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

    /* Framebuffer information */
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

        /* Check if scaling might be active */
        if (get_screen_size(&width, &height) == 0) {
            if (vinfo.xres != width || vinfo.yres != height) {
                printf("Scaling: %ux%u -> %ux%u (active)\n",
                       vinfo.xres, vinfo.yres, width, height);
            } else {
                printf("Scaling: none (1:1)\n");
            }
        }
    } else {
        printf("Failed to read framebuffer info\n");
    }

    printf("\n--- Supported HDMI modes (from EDID) ---\n");
    printf("  Mode  Name      Resolution   Supported\n");
    printf("  ----  --------  -----------  ---------\n");
    for (int i = 0; mode_table[i].name != NULL; i++) {
        int supported = hdmi_mode_supported(mode_table[i].mode);
        printf("  %2d    %-8s  %4dx%-4d    %s\n",
               mode_table[i].mode, mode_table[i].name,
               mode_table[i].width, mode_table[i].height,
               supported > 0 ? "Yes" : "No");
    }
    printf("\nNote: Mode support detection requires HDMI cable connected\n");
    printf("      and EDID to be parsed by the driver.\n");
}

/* Parse resolution string like "1280x720" or "1280x720@60" */
int parse_resolution(const char *str, uint32_t *width, uint32_t *height, uint32_t *refresh)
{
    uint32_t dummy_refresh = 0;
    uint32_t *refresh_ptr = refresh ? refresh : &dummy_refresh;

    SCALE_DEBUG("parse_resolution: str='%s' refresh=%p", str, (void*)refresh);

    *refresh_ptr = 0;
    if (sscanf(str, "%ux%u@%u", width, height, refresh_ptr) >= 2) {
        SCALE_DEBUG("parse_resolution: parsed %ux%u@%u", *width, *height, *refresh_ptr);
        return 0;
    }
    if (sscanf(str, "%ux%u", width, height) == 2) {
        SCALE_DEBUG("parse_resolution: parsed %ux%u (no refresh)", *width, *height);
        return 0;
    }
    SCALE_DEBUG("parse_resolution: FAILED to parse");
    return -1;
}

/* Parse resolution with depth like "640x480x32" */
int parse_resolution_depth(const char *str, uint32_t *width, uint32_t *height, int *depth)
{
    if (sscanf(str, "%ux%ux%d", width, height, depth) == 3) {
        return 0;
    }
    return -1;
}

int main(int argc, char *argv[])
{
    int ret = 0;
    int arg_start = 1;
    int orig_argc = argc;

    /* Install signal handlers for debugging crashes */
    install_signal_handlers();

    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    /* Parse options: -v (verbose) and -s <screen> */
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
                fprintf(stderr, "Invalid screen number: %s (must be 0 or 1)\n", argv[arg_start + 1]);
                return 1;
            }
            arg_start += 2;
        }
        else if (strcmp(argv[arg_start], "--help") == 0 || strcmp(argv[arg_start], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        else {
            fprintf(stderr, "Unknown option: %s\n", argv[arg_start]);
            print_usage(argv[0]);
            return 1;
        }
    }

    SCALE_DEBUG("Signal handlers installed");
    SCALE_DEBUG("Verbose mode enabled");
    SCALE_DEBUG("Force mode: %s", g_force ? "yes" : "no");
    SCALE_DEBUG("Screen: %u", g_screen);

    /* Check if we have a command after options */
    if (arg_start >= orig_argc) {
        print_usage(argv[0]);
        return 1;
    }

    if (disp_open() < 0) {
        return 1;
    }

    if (strcmp(argv[arg_start], "info") == 0) {
        show_info();
    }
    else if (strcmp(argv[arg_start], "debug") == 0) {
        show_debug_info();
    }
    else if (strcmp(argv[arg_start], "hdmi") == 0 && orig_argc >= arg_start + 2) {
        if (strcmp(argv[arg_start + 1], "on") == 0) {
            ret = hdmi_on();
            if (ret == 0) printf("HDMI enabled\n");
        }
        else if (strcmp(argv[arg_start + 1], "off") == 0) {
            ret = hdmi_off();
            if (ret == 0) printf("HDMI disabled\n");
        }
        else if (strcmp(argv[arg_start + 1], "mode") == 0 && orig_argc >= arg_start + 3) {
            const char *mode_arg = argv[arg_start + 2];
            const mode_info_t *info = NULL;

            /* Try numeric mode first */
            char *endptr;
            long mode_num = strtol(mode_arg, &endptr, 10);
            if (*endptr == '\0' && mode_num >= 0 && mode_num < DISP_TV_MODE_NUM) {
                /* Numeric mode specified */
                info = get_mode_info((disp_tv_mode_t)mode_num);
                if (!info) {
                    /* Mode not in table but still valid, use it directly */
                    printf("Setting HDMI mode %ld (not in mode table)\n", mode_num);
                    ret = hdmi_init((disp_tv_mode_t)mode_num);
                    if (ret == 0) {
                        printf("HDMI mode set to %ld\n", mode_num);
                    }
                }
            } else {
                /* Try mode name */
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
                    fprintf(stderr, "No matching HDMI mode for %dx%d\n", width, height);
                    ret = 1;
                }
            } else {
                fprintf(stderr, "Invalid resolution format: %s\n", argv[arg_start + 2]);
                ret = 1;
            }
        }
        else {
            print_usage(argv[0]);
            ret = 1;
        }
    }
    else if (strcmp(argv[arg_start], "fb") == 0 && orig_argc >= arg_start + 3) {
        if (strcmp(argv[arg_start + 1], "set") == 0) {
            uint32_t width, height;
            int depth;
            if (parse_resolution_depth(argv[arg_start + 2], &width, &height, &depth) == 0) {
                ret = fb_configure(width, height, depth);
            } else {
                fprintf(stderr, "Invalid format. Use: WxHxDEPTH (e.g., 640x480x32)\n");
                ret = 1;
            }
        }
        else {
            print_usage(argv[0]);
            ret = 1;
        }
    }
    else if (strcmp(argv[arg_start], "scale") == 0 && orig_argc >= arg_start + 4) {
        uint32_t fb_width, fb_height, scn_width, scn_height;
        int depth;

        SCALE_DEBUG("=== 'scale' command ===");

        /* Check structure alignment before attempting scaling */
        if (check_structure_alignment() > 0) {
            fprintf(stderr, "Continuing anyway (may crash)...\n\n");
        }

        SCALE_DEBUG("Parsing arguments: fb='%s' scn='%s' depth='%s'",
                    argv[arg_start + 1], argv[arg_start + 2], argv[arg_start + 3]);

        if (parse_resolution(argv[arg_start + 1], &fb_width, &fb_height, NULL) == 0 &&
            parse_resolution(argv[arg_start + 2], &scn_width, &scn_height, NULL) == 0) {
            depth = atoi(argv[arg_start + 3]);
            SCALE_DEBUG("Parsed: fb=%ux%u scn=%ux%u depth=%d",
                        fb_width, fb_height, scn_width, scn_height, depth);

            if (depth != 16 && depth != 24 && depth != 32) {
                fprintf(stderr, "Invalid depth. Use 16, 24, or 32\n");
                ret = 1;
            } else {
                /*
                 * Use DISP_CMD_FB_REQUEST to create framebuffer with scaling.
                 * This is the proper sun7i way - it creates a framebuffer with
                 * an associated layer configured for hardware scaling.
                 */
                SCALE_DEBUG("Calling setup_fb_with_scaling()...");
                ret = setup_fb_with_scaling(0, fb_width, fb_height,
                                            scn_width, scn_height, depth);
                SCALE_DEBUG("setup_fb_with_scaling returned: %d", ret);
                if (ret == 0) {
                    printf("Framebuffer: %dx%d @ %dbpp\n", fb_width, fb_height, depth);
                    printf("Screen output: %dx%d\n", scn_width, scn_height);
                }
            }
        } else {
            SCALE_DEBUG("Failed to parse resolution arguments");
            fprintf(stderr, "Invalid resolution format\n");
            ret = 1;
        }
    }
    else if (strcmp(argv[arg_start], "scale2") == 0 && orig_argc >= arg_start + 4) {
        /* Alternative method using manual layer setup (for testing) */
        uint32_t fb_width, fb_height, scn_width, scn_height;
        int depth;

        SCALE_DEBUG("=== 'scale2' command ===");

        /* Check structure alignment before attempting scaling */
        if (check_structure_alignment() > 0) {
            fprintf(stderr, "Continuing anyway (may crash)...\n\n");
        }

        SCALE_DEBUG("Parsing arguments: fb='%s' scn='%s' depth='%s'",
                    argv[arg_start + 1], argv[arg_start + 2], argv[arg_start + 3]);

        if (parse_resolution(argv[arg_start + 1], &fb_width, &fb_height, NULL) == 0 &&
            parse_resolution(argv[arg_start + 2], &scn_width, &scn_height, NULL) == 0) {
            depth = atoi(argv[arg_start + 3]);
            SCALE_DEBUG("Parsed: fb=%ux%u scn=%ux%u depth=%d",
                        fb_width, fb_height, scn_width, scn_height, depth);

            if (depth != 16 && depth != 24 && depth != 32) {
                fprintf(stderr, "Invalid depth. Use 16, 24, or 32\n");
                ret = 1;
            } else {
                /* First configure the framebuffer via standard fbdev */
                SCALE_DEBUG("Calling fb_configure(%u, %u, %d)...",
                            fb_width, fb_height, depth);
                ret = fb_configure(fb_width, fb_height, depth);
                SCALE_DEBUG("fb_configure returned: %d", ret);

                if (ret == 0) {
                    /* Get physical address from fb driver */
                    struct fb_fix_screeninfo finfo;
                    SCALE_DEBUG("g_fb_fd = %d", g_fb_fd);

                    if (g_fb_fd >= 0) {
                        SCALE_DEBUG("Calling FBIOGET_FSCREENINFO ioctl...");
                        if (ioctl(g_fb_fd, FBIOGET_FSCREENINFO, &finfo) == 0) {
                            SCALE_DEBUG("FBIOGET_FSCREENINFO succeeded");
                            SCALE_DEBUG("  finfo.smem_start = 0x%lx", finfo.smem_start);
                            SCALE_DEBUG("  finfo.smem_len = %u", finfo.smem_len);
                            SCALE_DEBUG("  finfo.line_length = %u", finfo.line_length);
                            printf("FB physical address: 0x%lx\n", finfo.smem_start);

                            SCALE_DEBUG("Calling setup_scaling_layer()...");
                            int layer = setup_scaling_layer(fb_width, fb_height,
                                                            scn_width, scn_height,
                                                            depth, finfo.smem_start);
                            SCALE_DEBUG("setup_scaling_layer returned: %d", layer);

                            if (layer >= 0) {
                                printf("Scaling layer created (handle: %d)\n", layer);
                                printf("Framebuffer: %dx%d @ %dbpp\n", fb_width, fb_height, depth);
                                printf("Screen output: %dx%d\n", scn_width, scn_height);
                            } else {
                                ret = 1;
                            }
                        } else {
                            SCALE_DEBUG("FBIOGET_FSCREENINFO failed (errno=%d: %s)",
                                        errno, strerror(errno));
                            fprintf(stderr, "Failed to get framebuffer info\n");
                            ret = 1;
                        }
                    } else {
                        SCALE_DEBUG("ERROR: g_fb_fd < 0 after fb_configure!");
                        fprintf(stderr, "Failed to get framebuffer info\n");
                        ret = 1;
                    }
                }
            }
        } else {
            SCALE_DEBUG("Failed to parse resolution arguments");
            fprintf(stderr, "Invalid resolution format\n");
            ret = 1;
        }
    }
    else if (strcmp(argv[arg_start], "autoscale") == 0) {
        /* Autoscale: scale current FB resolution to current screen resolution */
        struct fb_var_screeninfo vinfo;
        uint32_t scn_width, scn_height;
        int depth;

        SCALE_DEBUG("=== 'autoscale' command ===");

        /* Check structure alignment before attempting scaling */
        if (check_structure_alignment() > 0) {
            fprintf(stderr, "Continuing anyway (may crash)...\n\n");
        }

        /* Get current framebuffer info */
        if (get_fb_info(&vinfo, NULL) < 0) {
            fprintf(stderr, "Failed to read current framebuffer settings\n");
            ret = 1;
        }
        /* Get current screen size */
        else if (get_screen_size(&scn_width, &scn_height) < 0) {
            fprintf(stderr, "Failed to get current screen size\n");
            ret = 1;
        }
        else {
            /* Use depth from command line if provided, otherwise use current FB depth */
            if (arg_start + 1 < orig_argc) {
                depth = atoi(argv[arg_start + 1]);
                if (depth != 16 && depth != 24 && depth != 32) {
                    fprintf(stderr, "Invalid depth. Use 16, 24, or 32\n");
                    ret = 1;
                    goto autoscale_done;
                }
                SCALE_DEBUG("Using depth from command line: %d", depth);
            } else {
                depth = vinfo.bits_per_pixel;
                SCALE_DEBUG("Using current FB depth: %d", depth);
            }

            SCALE_DEBUG("Current FB: %ux%u @ %u bpp", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
            SCALE_DEBUG("Current screen: %ux%u", scn_width, scn_height);
            SCALE_DEBUG("Target depth: %d", depth);

            if (vinfo.xres == scn_width && vinfo.yres == scn_height) {
                printf("Framebuffer (%ux%u) already matches screen size - no scaling needed\n",
                       vinfo.xres, vinfo.yres);
            } else {
                printf("Scaling: %ux%u -> %ux%u @ %dbpp\n",
                       vinfo.xres, vinfo.yres, scn_width, scn_height, depth);

                SCALE_DEBUG("Calling setup_fb_with_scaling()...");
                ret = setup_fb_with_scaling(0, vinfo.xres, vinfo.yres,
                                            scn_width, scn_height, depth);
                SCALE_DEBUG("setup_fb_with_scaling returned: %d", ret);

                if (ret == 0) {
                    printf("Autoscale complete: %ux%u framebuffer scaled to %ux%u screen\n",
                           vinfo.xres, vinfo.yres, scn_width, scn_height);
                }
            }
        }
        autoscale_done: ;
    }
    else if (strcmp(argv[arg_start], "noscale") == 0) {
        /* Noscale: disable scaling by setting FB to match screen resolution */
        struct fb_var_screeninfo vinfo;
        uint32_t scn_width, scn_height;
        int depth;

        SCALE_DEBUG("=== 'noscale' command ===");

        /* Get current screen size */
        if (get_screen_size(&scn_width, &scn_height) < 0) {
            fprintf(stderr, "Failed to get current screen size\n");
            ret = 1;
        }
        else {
            /* Use depth from command line if provided, otherwise get from current FB */
            if (arg_start + 1 < orig_argc) {
                depth = atoi(argv[arg_start + 1]);
                if (depth != 16 && depth != 24 && depth != 32) {
                    fprintf(stderr, "Invalid depth. Use 16, 24, or 32\n");
                    ret = 1;
                    goto noscale_done;
                }
                SCALE_DEBUG("Using depth from command line: %d", depth);
            } else {
                /* Try to get current FB depth */
                if (get_fb_info(&vinfo, NULL) == 0) {
                    depth = vinfo.bits_per_pixel;
                    SCALE_DEBUG("Using current FB depth: %d", depth);
                } else {
                    depth = 32;  /* Default to 32bpp */
                    SCALE_DEBUG("Could not read FB, defaulting to depth: %d", depth);
                }
            }

            SCALE_DEBUG("Target: %ux%u @ %d bpp (no scaling)", scn_width, scn_height, depth);

            printf("Disabling scaling: setting FB to %ux%u @ %dbpp\n",
                   scn_width, scn_height, depth);

            /*
             * Call setup_fb_with_scaling with identical fb and screen dimensions.
             * This will use DISP_LAYER_WORK_MODE_NORMAL instead of SCALER.
             */
            SCALE_DEBUG("Calling setup_fb_with_scaling with identical dimensions...");
            ret = setup_fb_with_scaling(0, scn_width, scn_height,
                                        scn_width, scn_height, depth);
            SCALE_DEBUG("setup_fb_with_scaling returned: %d", ret);

            if (ret == 0) {
                printf("Scaling disabled: FB now %ux%u (1:1 with screen)\n",
                       scn_width, scn_height);
            }
        }
        noscale_done: ;
    }
    else {
        print_usage(argv[0]);
        ret = 1;
    }

    if (g_fb_fd >= 0) {
        close(g_fb_fd);
    }
    disp_close();

    return ret;
}
