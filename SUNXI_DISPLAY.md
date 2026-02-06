# Sunxi Display Engine Documentation

This document describes the Allwinner (sunxi) Display Engine architecture and the `sunxi_hdmi_fb` utility for controlling HDMI output and framebuffer scaling.

## Display Engine Versions

Allwinner SoCs use two generations of Display Engine:

| Version | SoCs | Kernel Naming | Architecture |
|---------|------|---------------|--------------|
| **DE1** | A10, A20 | sun4i, sun7i | Display Engine 1.0 |
| **DE2** | H3, H5, A64 | sun8iw7, sun50iw1 | Display Engine 2.0 |

The `sunxi_hdmi_fb` utility auto-detects the Display Engine version and uses the appropriate API.

## Hardware Architecture

### DE1 (A10/A20) Pipeline

```
Framebuffer -> Display Engine -> Layer -> TCON -> HDMI
                                  |
                              [Scaler]
                           (optional mode)
```

- **Layers**: 2 layers per screen (image0, image1)
- **Scaling**: Explicit `SCALER` layer mode required
- **Mali/EGL**: Incompatible with SCALER mode

### DE2 (H3/H5/A64) Pipeline

```
Framebuffer -> Display Engine 2.0 -> Mixer -> VSU/GSU -> TCON -> HDMI
                                       |         |
                                   [Layers]  [Scaler]
                                   (4+ per    (always
                                   channel)   available)
```

- **Layers**: 4+ layers per channel with z-ordering
- **Scaling**: Automatic via VSU (Video) / GSU (Graphics) scaler units
- **Mali/EGL**: Compatible (scaling is transparent)

## ioctl Command Differences

### Screen Information

| Function | DE1 Command | DE2 Command |
|----------|-------------|-------------|
| Get screen width | `0x08` | `0x07` |
| Get screen height | `0x09` | `0x08` |
| Get output type | `0x0a` | `0x09` |

### HDMI Control

| Function | DE1 Command | DE2 Command |
|----------|-------------|-------------|
| HDMI on | `0x1c0` | N/A (use DEVICE_SWITCH) |
| HDMI off | `0x1c1` | N/A (use DEVICE_SWITCH) |
| Set mode | `0x1c2` | N/A (use DEVICE_SWITCH) |
| Get mode | `0x1c3` | `0x10` (GET_OUTPUT) |
| Check mode support | `0x1c4` | `0xc4` |
| Get HPD status | `0x1c5` | sysfs only |
| Device switch | N/A | `0x0f` |

### Layer Control

| Function | DE1 Command | DE2 Command |
|----------|-------------|-------------|
| Request layer | `0x40` | N/A |
| Release layer | `0x41` | N/A |
| Enable layer | `0x42` | `0x40` |
| Disable layer | `0x43` | `0x41` |
| Set layer info | `0x4a` | `0x42` / `0x47` |
| Get layer info | `0x4b` | `0x43` / `0x48` |

### Framebuffer

| Function | DE1 Command | DE2 Command |
|----------|-------------|-------------|
| FB request | `0x280` | N/A (use fbdev) |
| FB release | `0x281` | N/A (use fbdev) |

## Scaling Behavior

### DE1 (A20) Scaling

Scaling requires explicit setup:

1. Release existing framebuffer
2. Request new framebuffer with `mode = DISP_LAYER_WORK_MODE_SCALER`
3. Set `width/height` (framebuffer) different from `output_width/output_height` (screen)

```c
// DE1 FB request structure
struct {
    fb_mode_t           fb_mode;           // FB_MODE_SCREEN0
    layer_work_mode_t   mode;              // NORMAL=0, SCALER=4
    uint32_t            buffer_num;
    uint32_t            width, height;     // Framebuffer size
    uint32_t            output_width;      // Screen size
    uint32_t            output_height;
    uint32_t            primary_screen_id;
    // ...
};
```

**Limitations:**
- SCALER mode is incompatible with Mali GPU / EGL
- Must run `noscale` before starting EGL applications
- Layer mode must be explicitly changed

### DE2 (H3) Scaling

Scaling is automatic and transparent:

1. Set framebuffer resolution via standard fbdev (`FBIOPUT_VSCREENINFO`)
2. DE2 automatically scales to screen size via VSU/GSU

```c
// Just use standard fbdev
struct fb_var_screeninfo vinfo;
vinfo.xres = 640;           // Framebuffer width
vinfo.yres = 480;           // Framebuffer height
ioctl(fd, FBIOPUT_VSCREENINFO, &vinfo);
// DE2 automatically scales 640x480 -> screen resolution
```

**Advantages:**
- No special mode required
- Mali GPU / EGL compatible
- Always active when FB size differs from screen size

## HDMI Mode Values

| Value | Mode | Resolution | Refresh |
|-------|------|------------|---------|
| 0 | 480i | 720x480 | 60 Hz |
| 1 | 576i | 720x576 | 50 Hz |
| 2 | 480p | 720x480 | 60 Hz |
| 3 | 576p | 720x576 | 50 Hz |
| 4 | 720p50 | 1280x720 | 50 Hz |
| 5 | 720p60 | 1280x720 | 60 Hz |
| 6 | 1080i50 | 1920x1080 | 50 Hz |
| 7 | 1080i60 | 1920x1080 | 60 Hz |
| 8 | 1080p24 | 1920x1080 | 24 Hz |
| 9 | 1080p50 | 1920x1080 | 50 Hz |
| 10 | 1080p60 | 1920x1080 | 60 Hz |
| 26 | 1080p25 | 1920x1080 | 25 Hz |
| 27 | 1080p30 | 1920x1080 | 30 Hz |
| 28 | 2160p30 | 3840x2160 | 30 Hz |
| 29 | 2160p25 | 3840x2160 | 25 Hz |
| 30 | 2160p24 | 3840x2160 | 24 Hz |

Note: 4K modes (28-30) are only available on DE2 (H3/H5/A64).

## sunxi_hdmi_fb Usage

### Basic Commands

```bash
# Show display information
sunxi_hdmi_fb info

# Show structure sizes (for debugging)
sunxi_hdmi_fb debug

# HDMI control
sunxi_hdmi_fb hdmi on              # Enable HDMI (uses 720p50 default)
sunxi_hdmi_fb hdmi off             # Disable HDMI
sunxi_hdmi_fb hdmi mode 720p60     # Set mode by name
sunxi_hdmi_fb hdmi mode 5          # Set mode by number
sunxi_hdmi_fb hdmi init 1920x1080  # Set mode by resolution

# Framebuffer control
sunxi_hdmi_fb fb set 640x480x32    # Set FB resolution and depth

# Scaling (behavior differs by DE version)
sunxi_hdmi_fb scale 640x480 1280x720 32   # FB 640x480 scaled to 720p
sunxi_hdmi_fb autoscale                    # Scale current FB to screen
sunxi_hdmi_fb noscale                      # Set FB to match screen (1:1)
```

### Command-line Options

| Option | Description |
|--------|-------------|
| `-v` | Verbose output (show debug messages) |
| `-f` | Force mode setting (bypass EDID check) |
| `-s <n>` | Select screen (0 or 1) |

### Examples

```bash
# Set HDMI to 1080p60 and scale 720p content
sunxi_hdmi_fb hdmi mode 1080p60
sunxi_hdmi_fb scale 1280x720 1920x1080 32

# Force a mode that EDID reports as unsupported
sunxi_hdmi_fb -f hdmi mode 576p

# Debug scaling issues
sunxi_hdmi_fb -v scale 640x480 1280x720 32

# Disable scaling for EGL apps (DE1 only - DE2 doesn't need this)
sunxi_hdmi_fb noscale
```

## Structure Definitions

### DE1 Layer Info (116 bytes)

```c
typedef struct {
    layer_work_mode_t   mode;           // 0=NORMAL, 4=SCALER
    bool                b_from_screen;
    uint8_t             pipe;
    uint8_t             prio;
    bool                alpha_en;
    uint16_t            alpha_val;
    bool                ck_enable;
    disp_rect           src_win;        // Source (FB) window
    disp_rect           scn_win;        // Screen (output) window
    disp_fb_t           fb;             // Framebuffer info
    bool                b_trd_out;
    uint32_t            out_trd_mode;
} de1_layer_info_t;
```

### DE2 Layer Config

```c
typedef struct {
    layer_mode_t        mode;           // BUFFER=0, COLOR=1
    uint8_t             zorder;
    uint8_t             alpha_mode;
    uint8_t             alpha_value;
    disp_rect           screen_win;     // Output window
    bool                b_trd_out;
    uint32_t            out_trd_mode;
    union {
        uint32_t        color;
        disp_fb_info    fb;             // Includes crop rectangle
    };
    uint32_t            id;
} de2_layer_info;

typedef struct {
    de2_layer_info      info;
    bool                enable;
    uint32_t            channel;
    uint32_t            layer_id;
} de2_layer_config;
```

## Troubleshooting

### "Mode not supported" error

The display driver checks EDID to determine supported modes. If HDMI is off or the display doesn't report support:

```bash
# Force the mode anyway
sunxi_hdmi_fb -f hdmi mode 720p60
```

### Segmentation fault during scaling (DE1)

Structure alignment mismatch between userspace and kernel:

```bash
# Check structure sizes
sunxi_hdmi_fb debug

# Expected sizes for DE1:
#   de1_fb_t:             64 bytes
#   de1_layer_info_t:    116 bytes
#   de1_fb_create_para_t: 56 bytes
```

### EGL/Mali crashes after scaling (DE1 only)

DE1 SCALER mode is incompatible with Mali GPU:

```bash
# Disable scaling before running EGL apps
sunxi_hdmi_fb noscale

# Then start your EGL application
./my_egl_app
```

### Scaling not working (DE2)

On DE2, scaling is automatic. Just set the framebuffer size:

```bash
# This automatically scales to screen size
sunxi_hdmi_fb fb set 640x480x32
```

## References

- Linux kernel source: `drivers/video/sunxi/disp/` (DE1)
- Linux kernel source: `drivers/video/sunxi/disp2/` (DE2)
- Kernel headers: `include/video/drv_display.h` (DE1)
- Kernel headers: `include/video/sunxi_display2.h` (DE2)
