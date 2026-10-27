#ifdef BUILD_SCREEN_SIM
// Host preview build (simulator) - stands in for the PROS and robot APIs so
// this exact file can be rendered on a Mac.
#include "sim_preview.h"
#include <cstdlib>
#include <cstring>
// The image descriptor only needs LVGL types, so the real header is fine here.
#include "subsystems/field_img.h"
#else
#include "main.h"
#include "globals.h"
#include "subsystems/cascade.h"
#include "subsystems/drivetrain.h"
#include "subsystems/screen.h"
#include "subsystems/field_img.h"
#include "routines/autons.h"
#include "routines/skills.h"
#include "liblvgl/lvgl.h"
#include <cstring>
#endif

// =============================================================================
// 936C brain UI
//
// Design rules followed here (dark-mode dashboard practice):
//   - #121212 base, not pure black, so surfaces have room to step up
//   - depth comes from LIGHTER SURFACE LAYERS, never from drop shadows
//   - colours are desaturated on dark backgrounds; no neon
//   - text is off-white (#E6E8EC), never pure #FFFFFF
//   - one accent hue, used sparingly, so it always means "this matters"
//   - hierarchy from scale: 18 > 16 > 14 > 12, one hero number per panel
//   - consistent 8 px rhythm for padding and gaps
//
// Structure
//   header   title left, live cascade position right
//   body     one of three pages
//   nav      three segmented buttons
// =============================================================================

#define UI_VARIANT_DEFAULT 1

// ===== Geometry =====
//
// The layout adapts to whatever the display actually reports. The V5's LVGL
// drawable height is smaller than the physical panel (LV_VER_RES_MAX is 240,
// against a 272 px panel), so hard-coding 272 pushes the bottom of the UI off
// the screen. Everything vertical is scaled from the real height instead.
static const int SCR_W = 480;
static const int SCR_H = 272;   // design height, used only as the scale base

static int disp_w = SCR_W;
static int disp_h = SCR_H;

// Vertical metrics, computed from the real display height in screen_init()
static int PAD   = 8;
static int HDR_H = 36;
static int NAV_H = 40;
static int PG_GAP = 6;
static int BOTTOM_MARGIN = 14;
static int NAV_Y = SCR_H - NAV_H - BOTTOM_MARGIN;

// Motors page block size, set by compute_layout() so the grid always fits.
static const int MB_GAP_Y = 3;
static int MB_H = 91;
static int MB_STRIP_H = 26;

// Field geometry, set by compute_layout() from the real display height.
static int FLD_H = 156;
static int FLD_W = 156;        // the field is square
static int FLD_X = 8;
static int FLD_Y = 8;

// Scale a design-space vertical measurement to the real display height.
static int vscale(int v) {
    long r = (long)v * (long)disp_h / (long)SCR_H;
    return (int)r;
}

static void compute_layout() {
    PAD           = vscale(8);
    HDR_H         = vscale(36);
    NAV_H         = vscale(40);
    PG_GAP        = vscale(6);
    BOTTOM_MARGIN = vscale(14);
    NAV_Y         = disp_h - NAV_H - BOTTOM_MARGIN;

    // The field image is square. Size it to whatever the AUTON page can give
    // it, so it always fits exactly.
    int page_h = NAV_Y - HDR_H - PAD - PG_GAP;
    FLD_H = page_h;
    if (FLD_H < 80) FLD_H = 80;
    FLD_W = FLD_H;

    // Motors grid: 2 rows of blocks plus a summary strip must fit the page.
    MB_STRIP_H = vscale(26);
    // 5 groups in 2 rows of 3, so 2 rows. (GROUP_COUNT is defined below.)
    const int MOTOR_ROWS = 2;
    int grid_h = page_h - MB_STRIP_H - PAD;        // leave room for the strip
    MB_H = grid_h / MOTOR_ROWS - MB_GAP_Y;
    if (MB_H < 40) MB_H = 40;
}

// =============================================================================
// Palette
//
// Three surface steps above the base. Each panel sits one step above the page
// behind it, which is what reads as depth on a dark screen.
// =============================================================================

struct Palette {
    uint32_t bg;         // page
    uint32_t surface;    // panel
    uint32_t surface2;   // nested / inset
    uint32_t surface3;   // hover / selected fill
    uint32_t border;     // hairline
    uint32_t text;       // primary
    uint32_t dim;        // secondary
    uint32_t faint;      // tertiary
    uint32_t accent;     // the single accent hue
    uint32_t on_accent;
    uint32_t ok;
    uint32_t warn;
    uint32_t hot;
    uint32_t red_side;
    uint32_t blue_side;
    uint32_t neutral;
    int      radius;
};

// Deep slate. Accent is a desaturated sky blue, dimmed so it does not glare.
static const Palette SLATE = {
    /* bg */        0x121417,
    /* surface */   0x1A1D21,
    /* surface2 */  0x22262B,
    /* surface3 */  0x2C3138,
    /* border */    0x31363D,
    /* text */      0xE6E8EC,
    /* dim */       0x9BA3AE,
    /* faint */     0x6B737E,
    /* accent */    0x4A9FE0,
    /* on_accent */ 0x0B1620,
    /* ok */        0x4CAF87,
    /* warn */      0xD9A544,
    /* hot */       0xD96A5C,
    /* red_side */  0xB5524C,
    /* blue_side */ 0x4C74B5,
    /* neutral */   0x7A828D,
    /* radius */    10,
};

// Warm charcoal, amber accent. Squarer.
static const Palette AMBER = {
    /* bg */        0x14120F,
    /* surface */   0x1E1B16,
    /* surface2 */  0x272319,
    /* surface3 */  0x332D20,
    /* border */    0x3A3428,
    /* text */      0xEDE9E1,
    /* dim */       0xA79E8C,
    /* faint */     0x736B5C,
    /* accent */    0xD9A03C,
    /* on_accent */ 0x1A1408,
    /* ok */        0x63A86B,
    /* warn */      0xD9A03C,
    /* hot */       0xCC6252,
    /* red_side */  0xB0564A,
    /* blue_side */ 0x4E76A8,
    /* neutral */   0x7E776A,
    /* radius */    4,
};

static const Palette* P = &SLATE;

// =============================================================================
// Handles
// =============================================================================
static lv_obj_t* header_pos  = nullptr;
static lv_obj_t* nav_btn[3]  = { nullptr, nullptr, nullptr };
static lv_obj_t* nav_lbl[3]  = { nullptr, nullptr, nullptr };
static lv_obj_t* page[3]     = { nullptr, nullptr, nullptr };
static int       active_tab  = 0;

// Set when opcontrol() starts. PROS exposes no match clock, so elapsed driver
// time is measured from here. The driver period is 1:45.
static uint32_t match_start_ms = 0;
static const uint32_t DRIVER_PERIOD_MS = 105000;   // 1:45

// AUTON
static lv_obj_t* robot_body = nullptr;
static lv_obj_t* robot_arrow = nullptr;
static lv_obj_t* robot_coord = nullptr;

// ===== Motor groups shown on the MOTORS page =====
struct MotorGroupEntry {
    const char* name;
    pros::MotorGroup* group;
};

static MotorGroupEntry groups[] = {
    { "DRIVE L", &left_motor_group },
    { "DRIVE R", &right_motor_group },
    { "INTAKE",  &intake_motor },
    { "ROLLER",  &intake_motor2 },
    { "CASCADE", &cascade_motor },
};
static const int GROUP_COUNT = sizeof(groups) / sizeof(groups[0]);
static const int MAX_PER_GROUP = 6;

// MOTORS
// Motors page: one block per group, containing a vertical temperature bar per
// physical motor plus that motor's live numbers.
static const int MB_BAR_W   = 8;    // bar width
static const int MB_BAR_GAP = 2;
static const int MB_BAR_MAX = 42;   // full-height bar in px
static const int MB_CELL_W  = 23;   // one motor cell in a block
static lv_obj_t* mb_bar[GROUP_COUNT * MAX_PER_GROUP];
static lv_obj_t* mb_seg[GROUP_COUNT * MAX_PER_GROUP];
static lv_obj_t* mb_temp[GROUP_COUNT * MAX_PER_GROUP];
static lv_obj_t* mb_eff[GROUP_COUNT * MAX_PER_GROUP];
static lv_obj_t* mb_head[GROUP_COUNT];   // group name
static lv_obj_t* mb_peak[GROUP_COUNT];   // group peak temperature
static lv_obj_t* mb_verdict = nullptr;
static lv_obj_t* mb_worst = nullptr;

// STATUS
static lv_obj_t* batt_fill = nullptr;
static lv_obj_t* batt_val = nullptr;
static lv_obj_t* batt_volt = nullptr;
static lv_obj_t* casc_fill = nullptr;
static lv_obj_t* casc_val = nullptr;
static lv_obj_t* casc_state = nullptr;
static lv_obj_t* casc_per_motor = nullptr;
static lv_obj_t* casc_raw = nullptr;
static lv_obj_t* ui_tick = nullptr;
static lv_obj_t* st_pose = nullptr;
static lv_obj_t* st_theta = nullptr;
static lv_obj_t* st_uptime = nullptr;
static lv_obj_t* st_eff = nullptr;
static lv_obj_t* st_timer = nullptr;
static lv_obj_t* mstat_dot[GROUP_COUNT];
static lv_obj_t* mstat_val[GROUP_COUNT];

// =============================================================================
// Primitives
// =============================================================================

// lv_font_montserrat_18 is the largest face enabled in lv_conf, so it is the
// hero size. 16 subhead, 14 body, 12 caption.
static const lv_font_t* FONT_HERO = &lv_font_montserrat_18;
static const lv_font_t* FONT_HEAD = &lv_font_montserrat_16;
static const lv_font_t* FONT_BODY = &lv_font_montserrat_14;
static const lv_font_t* FONT_CAP  = &lv_font_montserrat_12;

static lv_obj_t* label(lv_obj_t* parent, const char* text, uint32_t colour,
                       const lv_font_t* font) {
    lv_obj_t* l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(colour), 0);
    return l;
}

static lv_obj_t* box(lv_obj_t* parent, int x, int y, int w, int h,
                     uint32_t fill, int radius) {
    lv_obj_t* b = lv_obj_create(parent);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, w, h);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_style_bg_color(b, lv_color_hex(fill), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(b, radius, 0);
    return b;
}

// A panel: one surface step above the page, with a soft hairline.
// No shadow - on dark surfaces a lighter fill reads as elevation.
static lv_obj_t* panel(lv_obj_t* parent, int x, int y, int w, int h) {
    lv_obj_t* p = box(parent, x, y, w, h, P->surface, P->radius);
    lv_obj_set_style_border_width(p, 1, 0);
    lv_obj_set_style_border_color(p, lv_color_hex(P->border), 0);
    lv_obj_set_style_border_opa(p, LV_OPA_COVER, 0);
    return p;
}

// A translucent "pill" used for tags
static lv_obj_t* pill(lv_obj_t* parent, const char* text, uint32_t fg,
                      uint32_t bg) {
    lv_obj_t* p = box(parent, 0, 0, 10, 10, bg, 4);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(p, 6, 0);
    lv_obj_set_style_pad_ver(p, 2, 0);
    lv_obj_set_style_min_width(p, 0, 0);
    lv_obj_set_style_min_height(p, 0, 0);

    lv_obj_t* l = label(p, text, fg, FONT_CAP);
    lv_obj_center(l);
    lv_obj_set_width(p, LV_SIZE_CONTENT);
    lv_obj_set_height(p, LV_SIZE_CONTENT);
    return p;
}

// V5 motors start cutting out around 55 C, so warn before that.
static const double TEMP_WARN = 50.0;
static const double TEMP_HOT  = 55.0;

static uint32_t temp_colour(double t) {
    if (t >= TEMP_HOT)  return P->hot;
    if (t >= TEMP_WARN) return P->warn;
    return P->ok;
}

static const char* temp_word(double t) {
    if (t >= TEMP_HOT)  return "HOT";
    if (t >= TEMP_WARN) return "WARM";
    return "OK";
}

// =============================================================================
// Tabs
// =============================================================================

static void show_tab(int idx) {
    if (idx < 0 || idx > 2) return;
    active_tab = idx;

    for (int i = 0; i < 3; i++) {
        bool on = (i == idx);
        if (on) {
            lv_obj_remove_flag(page[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(nav_btn[i], lv_color_hex(P->surface3), 0);
            lv_obj_set_style_border_color(nav_btn[i],
                                          lv_color_hex(P->accent), 0);
            lv_obj_set_style_text_color(nav_lbl[i],
                                        lv_color_hex(P->text), 0);
        } else {
            lv_obj_add_flag(page[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(nav_btn[i],
                                      lv_color_hex(P->surface), 0);
            lv_obj_set_style_border_color(nav_btn[i],
                                          lv_color_hex(P->border), 0);
            lv_obj_set_style_text_color(nav_lbl[i], lv_color_hex(P->dim), 0);
        }
    }
}

static void nav_clicked(lv_event_t* e) {
    show_tab((int)(intptr_t)lv_event_get_user_data(e));
}

// =============================================================================
// AUTON page - the real Override field
//
// Geometry taken from the standardised orthographic render published at
// field-rendering.jerryio.com (CC BY 4.0), so this matches the physical field
// rather than an estimate.
//
// Drawn in path.jerry coordinates, the same ones the auton paths use:
//   origin at field centre, X right, Y toward the LEFT wall, so the field
//   spans -72..+72 on both axes. Screen Y is flipped.
//
// Heading convention (confirmed from the existing auton patches):
//   0 = +Y, 90 = -X, 180 = -Y, 270 = +X
//
// Real layout notes:
//   - The Midfield is a ROTATED SQUARE (diamond) in the centre. Its four
//     vertices sit on the wall centre-lines at +-72 and its edges run at 45
//     degrees. The edges are white truss, drawn as parallel lines.
//   - Neutral Goals are large circular Stacks with yellow rings: one at the
//     centre (the tall Goal) and four around it in a plus pattern.
//   - Alliance Goals are star-shaped truss structures in opposite corners,
//     with red/blue plates on the diagonals.
//   - Toggles are small red/blue markers at the Midfield vertices.
// =============================================================================

// The field fills the AUTON page. A square field on a 480x272 screen means
// the scale is limited by height, and the leftover width is used for the
// routine picker column.
static double fld_px(double inches) {
    return inches * ((double)FLD_W / 144.0);
}

static void fld_to_page(double x_in, double y_in, int* px, int* py) {
    *px = FLD_X + (int)(fld_px(x_in + 72.0));
    *py = FLD_Y + FLD_H - (int)(fld_px(y_in + 72.0));
}

// A Neutral Goal: a circular Stack with a ring, as seen from above
static void draw_neutral_goal(lv_obj_t* parent, double x_in, double y_in,
                             int ring, uint32_t ring_colour) {
    int cx, cy;
    fld_to_page(x_in, y_in, &cx, &cy);

    lv_obj_t* outer = box(parent, cx - ring, cy - ring, ring * 2, ring * 2,
                          ring_colour, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(outer, LV_OPA_COVER, 0);

    int inner = ring - 3;
    lv_obj_t* hole = box(parent, cx - inner, cy - inner, inner * 2, inner * 2,
                         P->surface, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(hole, LV_OPA_COVER, 0);
}

// A star-shaped Alliance Goal in a corner. Drawn as three plates on the
// diagonals plus a centre, which is what the real structure looks like from
// directly above.
static void draw_alliance_goal(lv_obj_t* parent, double x_in, double y_in,
                               uint32_t colour) {
    int cx, cy;
    fld_to_page(x_in, y_in, &cx, &cy);

    const int arm = 13;
    const int w = 7;

    lv_obj_t* c = box(parent, cx - 4, cy - 4, 8, 8, colour, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);

    // three arms at 120 degree spacing, matching the real three-sided star
    for (int i = 0; i < 3; i++) {
        double ang = (90.0 + i * 120.0) * 3.14159265 / 180.0;
        int ax = cx + (int)(cos(ang) * arm);
        int ay = cy - (int)(sin(ang) * arm);
        lv_obj_t* p = box(parent, ax - w / 2, ay - w / 2, w, w, colour, 2);
        lv_obj_set_style_bg_opa(p, LV_OPA_80, 0);
    }
}

// A small alliance-coloured Toggle marker
static void draw_toggle(lv_obj_t* parent, double x_in, double y_in,
                        uint32_t colour) {
    int cx, cy;
    fld_to_page(x_in, y_in, &cx, &cy);
    lv_obj_t* t = box(parent, cx - 3, cy - 3, 6, 6, colour, LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(t, LV_OPA_COVER, 0);
}

// Draw a line between two field points, in field inches
static void draw_field_line(lv_obj_t* parent, double x1, double y1,
                            double x2, double y2, int width,
                            uint32_t colour, lv_opa_t opa) {
    int ax, ay, bx, by;
    fld_to_page(x1, y1, &ax, &ay);
    fld_to_page(x2, y2, &bx, &by);

    lv_obj_t* ln = lv_line_create(parent);
    lv_obj_set_style_line_width(ln, width, 0);
    lv_obj_set_style_line_color(ln, lv_color_hex(colour), 0);
    lv_obj_set_style_line_opa(ln, opa, 0);
    lv_obj_set_style_line_rounded(ln, true, 0);
    lv_obj_set_pos(ln, 0, 0);

    static lv_point_precise_t pts[16][2];
    static int slot = 0;
    int s = slot++ % 16;
    pts[s][0].x = ax - FLD_X;
    pts[s][0].y = ay - FLD_Y;
    pts[s][1].x = bx - FLD_X;
    pts[s][1].y = by - FLD_Y;
    lv_line_set_points(ln, pts[s], 2);
}

static void build_field(lv_obj_t* pg) {
    const double px_per_in = (double)FLD_W / 144.0;

    // ---- Field background: the real field render ----
    // Drawn from the embedded image so the geometry is exact, rather than a
    // hand-drawn approximation. The robot marker is overlaid on top.
    lv_obj_t* field = lv_image_create(pg);
    lv_image_set_src(field, &field_img);
    lv_obj_set_pos(field, FLD_X, FLD_Y);
    lv_obj_remove_flag(field, LV_OBJ_FLAG_SCROLLABLE);

    // Draw the field at exactly the size the layout gave it, so it always
    // fits the page regardless of the display height we were handed.
    lv_obj_set_size(field, FLD_W, FLD_H);
    lv_image_set_inner_align(field, LV_IMAGE_ALIGN_STRETCH);

    // ---- Robot: body square plus a heading line, overlaid on the field ----
    const int body = (int)(15.0 * px_per_in);
    robot_body = box(pg, 0, 0, body, body, P->text, 3);
    lv_obj_set_style_border_width(robot_body, 2, 0);
    lv_obj_set_style_border_color(robot_body, lv_color_hex(P->accent), 0);
    lv_obj_set_style_border_opa(robot_body, LV_OPA_COVER, 0);

    robot_arrow = lv_line_create(pg);
    lv_obj_set_style_line_width(robot_arrow, 3, 0);
    lv_obj_set_style_line_color(robot_arrow, lv_color_hex(P->accent), 0);
    lv_obj_set_style_line_rounded(robot_arrow, true, 0);
    lv_obj_set_pos(robot_arrow, 0, 0);
}

static void update_robot_marker() {
    lemlib::Pose p = chassis.getPose();

    int cx, cy;
    fld_to_page(p.x, p.y, &cx, &cy);

    const int body = 18;
    lv_obj_set_pos(robot_body, cx - body / 2, cy - body / 2);

    // Heading follows the standard LemLib / path.jerry convention, confirmed
    // against the existing auton paths:
    //   0 = +Y (up), 90 = +X (right), 180 = -Y (down), 270 = -X (left)
    // So the unit vector for heading h is (sin h, cos h) in field axes, and
    // screen Y is flipped when drawing.
    double rad = p.theta * 3.14159265 / 180.0;
    int len = 17;
    int ax = cx + (int)(sin(rad) * len);
    int ay = cy - (int)(cos(rad) * len);

    static lv_point_precise_t pts[2];
    pts[0].x = cx - FLD_X;
    pts[0].y = cy - FLD_Y;
    pts[1].x = ax - FLD_X;
    pts[1].y = ay - FLD_Y;
    lv_line_set_points(robot_arrow, pts, 2);

    if (robot_coord) {
        // Shown in the header: "x , y @ heading"
        char buf[48];
        snprintf(buf, sizeof(buf), "%.0f, %.0f  @ %.0f", p.x, p.y, p.theta);
        lv_label_set_text(robot_coord, buf);
    }
}

// Draw the selected routine's route on the field, as a plain polyline.
// No headings - just the points, as requested.
static void build_auton_page(lv_obj_t* pg) {
    // The auton selector is gone: the routine is chosen in autonomous() in
    // main.cpp. This page shows the field and the live robot position.
    build_field(pg);
}

// =============================================================================
// MOTORS page - segmented heat bars
// =============================================================================

// Layout: up to 3 group blocks across, up to 2 rows.
static const int MB_W      = 150;
static const int MB_GAP_X  = 8;
static const int MB_HEAD_H = 19;
static const int MB_GRID_H = 44;   // bar grid + numbers

static void build_motor_page(lv_obj_t* pg) {
    for (int g = 0; g < GROUP_COUNT; g++) {
        int col = g % 3;
        int row = g / 3;
        int bx = col * (MB_W + MB_GAP_X);
        int by = row * (MB_H + MB_GAP_Y);

        lv_obj_t* blk = panel(pg, bx, by, MB_W, MB_H);
        lv_obj_set_style_pad_all(blk, 0, 0);

        // Header: group name on the left, peak temperature on the right
        mb_head[g] = label(blk, groups[g].name, P->text, FONT_CAP);
        lv_obj_set_width(mb_head[g], MB_W - 46);
        lv_obj_set_style_text_align(mb_head[g], LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_align(mb_head[g], LV_ALIGN_TOP_LEFT, 7, 4);

        mb_peak[g] = label(blk, "--", P->dim, FONT_CAP);
        lv_obj_align(mb_peak[g], LV_ALIGN_TOP_RIGHT, -7, 4);

        // Thin rule under the header, so each block reads as a card
        box(blk, 6, MB_HEAD_H - 3, MB_W - 12, 1, P->border, 0);

        // One cell per physical motor: bar on top, number and efficiency below
        for (int k = 0; k < MAX_PER_GROUP; k++) {
            int idx = g * MAX_PER_GROUP + k;
            int cx = 6 + k * MB_CELL_W;

            // Bar track, centred in the cell
            int bx = cx + (MB_CELL_W - MB_BAR_W) / 2;
            box(blk, bx, MB_HEAD_H, MB_BAR_W, MB_BAR_MAX,
                P->surface2, 2);

            // Filled portion, grown upward from the bottom in refresh()
            lv_obj_t* fill = box(blk, bx, MB_HEAD_H, MB_BAR_W, 4, P->ok, 2);
            lv_obj_set_style_bg_opa(fill, LV_OPA_COVER, 0);
            mb_bar[idx] = fill;

            // Temperature, right aligned in the cell
            mb_temp[idx] = label(blk, "", P->dim, FONT_CAP);
            lv_obj_set_width(mb_temp[idx], MB_CELL_W - 1);
            lv_obj_set_style_text_align(mb_temp[idx], LV_TEXT_ALIGN_RIGHT, 0);
            lv_obj_align(mb_temp[idx], LV_ALIGN_TOP_LEFT,
                         cx + 1, MB_HEAD_H + MB_BAR_MAX + 3);

            lv_obj_add_flag(mb_bar[idx], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(mb_temp[idx], LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Summary line, placed directly under the blocks grid
    int rows = (GROUP_COUNT + 2) / 3;
    int sy = rows * (MB_H + MB_GAP_Y) + 2;
    lv_obj_t* strip = panel(pg, 0, sy, SCR_W - 2 * PAD, MB_STRIP_H);
    lv_obj_set_style_pad_left(strip, 11, 0);
    lv_obj_set_style_pad_right(strip, 11, 0);
    mb_worst = label(strip, "", P->dim, FONT_CAP);
    lv_obj_align(mb_worst, LV_ALIGN_LEFT_MID, 0, 0);
    mb_verdict = nullptr;
}

static void refresh_motors() {
    double hottest = 0;
    bool hot = false, warn = false, any_over = false;

    for (int g = 0; g < GROUP_COUNT; g++) {
        std::vector<double> temps = groups[g].group->get_temperature_all();
        std::vector<double> effs  = groups[g].group->get_efficiency_all();

        double peak = 0;

        for (int k = 0; k < MAX_PER_GROUP; k++) {
            int idx = g * MAX_PER_GROUP + k;

            if (k >= (int)temps.size()) {
                lv_obj_add_flag(mb_bar[idx], LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(mb_temp[idx], LV_OBJ_FLAG_HIDDEN);
                continue;
            }

            double t = temps[k];
            uint32_t col = temp_colour(t);

            // Bar height: 25 C is nearly empty, TEMP_HOT is full
            double frac = (t - 25.0) / (TEMP_HOT - 25.0);
            if (frac < 0.0) frac = 0.0;
            if (frac > 1.0) frac = 1.0;
            int h = (int)(frac * MB_BAR_MAX);
            if (h < 4) h = 4;

            lv_obj_remove_flag(mb_bar[idx], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_height(mb_bar[idx], h);
            // Bars grow upward from the bottom of their track. The x position
            // must match how the track was built, or the bar drifts away from
            // it on the first refresh.
            int bx = 6 + k * MB_CELL_W + (MB_CELL_W - MB_BAR_W) / 2;
            lv_obj_align(mb_bar[idx], LV_ALIGN_TOP_LEFT, bx,
                         MB_HEAD_H + (MB_BAR_MAX - h));
            lv_obj_set_style_bg_color(mb_bar[idx], lv_color_hex(col), 0);
            lv_obj_set_style_bg_opa(mb_bar[idx], LV_OPA_COVER, 0);

            // Temperature
            lv_obj_remove_flag(mb_temp[idx], LV_OBJ_FLAG_HIDDEN);
            char buf[16];
            snprintf(buf, sizeof(buf), "%.0f", t);
            lv_label_set_text(mb_temp[idx], buf);
            lv_obj_set_style_text_color(mb_temp[idx], lv_color_hex(col), 0);

            if (t > peak)    peak = t;
            if (t > hottest) hottest = t;
            if (t >= TEMP_HOT)  hot = true;
            if (t >= TEMP_WARN) warn = true;
        }

        // Group peak in the header
        char buf[16];
        snprintf(buf, sizeof(buf), "%.0f", peak);
        lv_label_set_text(mb_peak[g], buf);
        lv_obj_set_style_text_color(mb_peak[g], lv_color_hex(temp_colour(peak)), 0);

        if (groups[g].group->is_over_temp() != 0) {
            any_over = true;
            hot = true;
        }
    }

    // Summary line on the motors page itself
    if (mb_worst) {
        char head[64];
        const char* word = any_over ? "OVER TEMP"
                         : hot      ? "TOO HOT"
                         : warn     ? "WARM"
                                    : "ALL OK";
        snprintf(head, sizeof(head), "hottest %.0f C   %s", hottest, word);
        lv_label_set_text(mb_worst, head);
        lv_obj_set_style_text_color(mb_worst, lv_color_hex(temp_colour(hottest)), 0);
    }
}

// =============================================================================
// STATUS page - live diagnostics
//
// Everything here is for debugging at the field:
//   - The CASCADE position is the raw encoder value in the SAME UNITS that
//     cascade_go_to() takes, so it can be read off the screen and typed
//     straight into an auton. No conversion, no mental maths.
//   - HOMED / HOMING / FAILED tells you whether go-to will even be accepted;
//     motion is refused unless the lift has homed.
//   - Battery voltage is shown next to the percentage, because voltage is the
//     honest number.
//   - Match timer counts from the start of the driver period, and for the last
//     10 s shows how long until the <SG12> centre-goal lockout.
// =============================================================================

// The bar scales against this, which is a realistic full-travel encoder count.
static const double CASC_FULL_SCALE = 2000.0;

static lv_obj_t* make_gauge(lv_obj_t* pg, int x, int y, int w, int h,
                            const char* cap, lv_obj_t** out_fill,
                            uint32_t fill_colour, bool hero,
                            lv_obj_t** out_card = nullptr) {
    lv_obj_t* card = panel(pg, x, y, w, h);
    if (out_card) *out_card = card;

    lv_obj_t* c = label(card, cap, P->faint, FONT_CAP);
    lv_obj_align(c, LV_ALIGN_TOP_LEFT, 11, 7);

    lv_obj_t* v = label(card, "--", P->text, hero ? FONT_HERO : FONT_HEAD);
    lv_obj_align(v, LV_ALIGN_TOP_RIGHT, -11, hero ? 4 : 6);

    int track_y = h - 10;
    box(card, 11, track_y, w - 22, 7, P->surface2, 4);
    *out_fill = box(card, 11, track_y, 4, 7, fill_colour, 4);

    return v;
}

static void build_status_page(lv_obj_t* pg) {
    const int gap = PAD;
    const int col_w = (SCR_W - 2 * PAD - gap) / 2;

    // ---- Cascade: the headline for debugging ----
    // Its value is directly usable as the argument to cascade_go_to().
    lv_obj_t* casc_card = nullptr;
    casc_val = make_gauge(pg, PAD, PAD, col_w, 74, "CASCADE POS",
                          &casc_fill, P->accent, true, &casc_card);

    // Parented to the card and positioned in its own coordinates, so it sits
    // above the track instead of landing on the page.
    casc_state = label(casc_card, "--", P->dim, FONT_CAP);
    lv_obj_set_pos(casc_state, 11, 28);

    // Per-motor encoder values. If the averaged position ever looks wrong or
    // stuck, this shows which of the three motors is not reporting.
    casc_per_motor = label(casc_card, "", P->faint, FONT_CAP);
    lv_obj_set_pos(casc_per_motor, 11, 40);

    // Raw motor-0 reading and the motor count. Together with the averaged
    // value above, these say whether the encoder is silent or the code is
    // misreading it.
    casc_raw = label(casc_card, "", P->faint, FONT_CAP);
    lv_obj_set_pos(casc_raw, 11, 52);

    // ---- Battery ----
    lv_obj_t* batt_card = nullptr;
    batt_val = make_gauge(pg, PAD + col_w + gap, PAD, col_w, 74, "BATTERY",
                          &batt_fill, P->ok, true, &batt_card);

    batt_volt = label(batt_card, "--", P->dim, FONT_CAP);
    lv_obj_set_pos(batt_volt, 11, 38);

    // ---- Motor health strip ----
    // You are on this page instead of the motors page, so it has to tell you
    // whether anything is hot.
    lv_obj_t* sys = panel(pg, PAD, PAD + 74 + gap, SCR_W - 2 * PAD, 36);
    lv_obj_t* sys_cap = label(sys, "MOTORS", P->faint, FONT_CAP);
    lv_obj_align(sys_cap, LV_ALIGN_TOP_LEFT, 11, 6);

    for (int g = 0; g < GROUP_COUNT; g++) {
        int cx = 11 + g * 88;

        mstat_dot[g] = box(sys, cx, 22, 9, 9, P->ok, LV_RADIUS_CIRCLE);
        lv_obj_set_style_bg_opa(mstat_dot[g], LV_OPA_COVER, 0);

        char buf[16];
        snprintf(buf, sizeof(buf), "%.0f", 0.0);
        mstat_val[g] = label(sys, "--", P->dim, FONT_CAP);
        lv_obj_align(mstat_val[g], LV_ALIGN_TOP_LEFT, cx + 14, 20);
        (void)buf;
    }

    // ---- Footer: pose, heading, timer, uptime ----
    lv_obj_t* foot = panel(pg, PAD, PAD + 74 + gap + 36 + gap,
                           SCR_W - 2 * PAD, 22);
    lv_obj_set_style_pad_left(foot, 11, 0);
    lv_obj_set_style_pad_right(foot, 11, 0);

    st_pose = label(foot, "--", P->text, FONT_CAP);
    lv_obj_align(st_pose, LV_ALIGN_LEFT_MID, 0, 0);

    st_timer = label(foot, "t --", P->dim, FONT_CAP);
    lv_obj_align(st_timer, LV_ALIGN_CENTER, 0, 0);

    st_uptime = label(foot, "up --", P->faint, FONT_CAP);
    lv_obj_align(st_uptime, LV_ALIGN_RIGHT_MID, 0, 0);
}

// Format the elapsed driver time as m:ss
static void format_clock(uint32_t ms, char* out, size_t n) {
    uint32_t total_s = ms / 1000;
    snprintf(out, n, "%lu:%02lu", (unsigned long)(total_s / 60),
             (unsigned long)(total_s % 60));
}

static void refresh_status() {
    lemlib::Pose p = chassis.getPose();
    char buf[64];

    // ---- Cascade position: raw encoder units, identical to the value
    // cascade_go_to() expects. Rounded, because you type a whole number.
    double cp = cascade_position();
    snprintf(buf, sizeof(buf), "%.0f", cp);
    lv_label_set_text(casc_val, buf);

    double frac = cp / CASC_FULL_SCALE;
    if (frac < 0.0) frac = 0.0;
    if (frac > 1.0) frac = 1.0;
    const int col_w = (SCR_W - 2 * PAD - PAD) / 2;
    int cw = (int)((col_w - 22) * frac);
    if (cw < 4) cw = 4;
    lv_obj_set_width(casc_fill, cw);

    // Cascade state: until it has homed, recorded go-to positions are
    // relative to an unset zero, so this matters when debugging.
    if (cascade_homing()) {
        lv_label_set_text(casc_state, "HOMING");
        lv_obj_set_style_text_color(casc_state, lv_color_hex(P->warn), 0);
    } else if (cascade_moving()) {
        lv_label_set_text(casc_state, "GOING TO TARGET");
        lv_obj_set_style_text_color(casc_state, lv_color_hex(P->accent), 0);
    } else if (cascade_has_homed()) {
        lv_label_set_text(casc_state, "HOMED");
        lv_obj_set_style_text_color(casc_state, lv_color_hex(P->ok), 0);
    } else {
        lv_label_set_text(casc_state, "NOT HOMED");
        lv_obj_set_style_text_color(casc_state, lv_color_hex(P->dim), 0);
    }

    // Per-motor encoder values, so a single bad motor is visible
    double mp[4] = {0};
    int mn = cascade_motor_positions(mp, 4);
    char per[64] = "";
    for (int i = 0; i < mn; i++) {
        char one[16];
        snprintf(one, sizeof(one), "%s%.0f", i ? " " : "", mp[i]);
        strncat(per, one, sizeof(per) - strlen(per) - 1);
    }
    if (casc_per_motor) lv_label_set_text(casc_per_motor, per);

    // Raw motor-0 read, and how many motors the group claims
    snprintf(buf, sizeof(buf), "raw %.0f   motors %d",
             cascade_position_raw(), cascade_motor_count());
    if (casc_raw) lv_label_set_text(casc_raw, buf);

    // ---- Battery: percentage plus the honest voltage ----
    double cap = pros::battery::get_capacity();
    snprintf(buf, sizeof(buf), "%.0f%%", cap);
    lv_label_set_text(batt_val, buf);

    uint32_t bc = P->ok;
    if (cap < 20.0)      bc = P->hot;
    else if (cap < 50.0) bc = P->warn;
    lv_obj_set_style_text_color(batt_val, lv_color_hex(bc), 0);
    lv_obj_set_style_bg_color(batt_fill, lv_color_hex(bc), 0);

    int bw = (int)((col_w - 22) * (cap / 100.0));
    if (bw < 4) bw = 4;
    if (bw > col_w - 22) bw = col_w - 22;
    lv_obj_set_width(batt_fill, bw);

    int32_t mv = pros::battery::get_voltage();
    snprintf(buf, sizeof(buf), "%.2f V", mv / 1000.0);
    lv_label_set_text(batt_volt, buf);

    // ---- Motor strip: worst reading per group ----
    for (int g = 0; g < GROUP_COUNT; g++) {
        std::vector<double> temps = groups[g].group->get_temperature_all();

        double worst = 0;
        for (size_t i = 0; i < temps.size(); i++) {
            if (temps[i] > worst) worst = temps[i];
        }

        uint32_t col = temp_colour(worst);
        lv_obj_set_style_bg_color(mstat_dot[g], lv_color_hex(col), 0);

        snprintf(buf, sizeof(buf), "%s %.0f", groups[g].name, worst);
        lv_label_set_text(mstat_val[g], buf);
        lv_obj_set_style_text_color(mstat_val[g], lv_color_hex(col), 0);
    }

    // ---- Footer ----
    snprintf(buf, sizeof(buf), "%.1f, %.1f  %.0f deg", p.x, p.y, p.theta);
    lv_label_set_text(st_pose, buf);

    snprintf(buf, sizeof(buf), "up %lu s", (unsigned long)(pros::millis() / 1000));
    lv_label_set_text(st_uptime, buf);

    // ---- Match clock ----
    // PROS has no match clock, so this counts from the start of opcontrol().
    // In the last 10 s it warns about the <SG12> centre-goal lockout.
    if (match_start_ms != 0) {
        uint32_t elapsed = pros::millis() - match_start_ms;
        uint32_t left = (elapsed < DRIVER_PERIOD_MS)
                        ? (DRIVER_PERIOD_MS - elapsed) / 1000 : 0;

        format_clock(elapsed, buf, sizeof(buf));
        lv_label_set_text(st_timer, buf);

        if (left <= 10) {
            lv_obj_set_style_text_color(st_timer, lv_color_hex(P->hot), 0);
        } else if (left <= 30) {
            lv_obj_set_style_text_color(st_timer, lv_color_hex(P->warn), 0);
        } else {
            lv_obj_set_style_text_color(st_timer, lv_color_hex(P->dim), 0);
        }
    } else {
        lv_label_set_text(st_timer, "t --");
    }
}

// Called from opcontrol() so the clock starts at the right moment
void screen_match_start() {
    match_start_ms = pros::millis();
}

// =============================================================================
// Build
// =============================================================================

void screen_init() {
#ifdef BUILD_SCREEN_SIM
    const char* env = getenv("UI_VARIANT");
    if (env && env[0] == '2') P = &AMBER;
#else
    #if UI_VARIANT_DEFAULT == 2
        P = &AMBER;
    #else
        P = &SLATE;
    #endif
#endif

    // Ask the display how big it really is, then lay out to that.
    {
        lv_display_t* d = lv_display_get_default();
        if (d) {
            int w = lv_display_get_horizontal_resolution(d);
            int h = lv_display_get_vertical_resolution(d);
            if (w > 0) disp_w = w;
            if (h > 0) disp_h = h;
        }
    }
    compute_layout();

    lv_obj_t* root = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(root, lv_color_hex(P->bg), 0);
    lv_obj_set_style_pad_all(root, 0, 0);
    lv_obj_remove_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    lv_screen_load(root);

    // ---- Header ----
    box(root, 0, 0, SCR_W, HDR_H, P->surface, 0);
    box(root, 0, HDR_H - 1, SCR_W, 1, P->border, 0);

    lv_obj_t* title = label(root, "936C", P->accent, FONT_HEAD);
    lv_obj_set_pos(title, PAD + 4, 1);

    // Live pose sits in the header so it is visible on every tab, like the
    // cascade position on the right.
    robot_coord = label(root, "--", P->dim, FONT_CAP);
    lv_obj_set_style_text_font(robot_coord, &lv_font_montserrat_12, 0);
    lv_obj_set_width(robot_coord, 96);
    lv_obj_set_style_text_align(robot_coord, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_pos(robot_coord, PAD + 4, 17);

    // (The motors summary lives on its own page now, so the header holds only
    // the cascade position on the right and the pose on the left.)

    // Cascade position, live, in the header so it is visible on every tab
    // without touching anything. This is the value cascade_go_to() takes.
    header_pos = label(root, "pos 0", P->accent, FONT_HERO);
    lv_obj_align(header_pos, LV_ALIGN_TOP_RIGHT, -(PAD + 6), 2);

    // ---- Pages ----
    for (int i = 0; i < 3; i++) {
        page[i] = box(root, PAD, HDR_H + PAD, SCR_W - 2 * PAD,
                      NAV_Y - HDR_H - PAD - PG_GAP, P->bg, 0);
    }

    build_auton_page(page[0]);
    build_motor_page(page[1]);
    build_status_page(page[2]);

    // ---- Nav ----
    box(root, 0, NAV_Y, SCR_W, NAV_H, P->surface, 0);
    box(root, 0, NAV_Y, SCR_W, 1, P->border, 0);

    const char* names[3] = { "FIELD", "MOTORS", "STATUS" };
    const int gap = PAD;
    const int bw = (SCR_W - 2 * PAD - 2 * gap) / 3;
    const int bh = NAV_H - 2 * PAD;
    const int by = NAV_Y + PAD;

    for (int i = 0; i < 3; i++) {
        lv_obj_t* b = box(root, PAD + i * (bw + gap), by, bw, bh, P->surface,
                          P->radius - 2);
        lv_obj_set_style_border_width(b, 1, 0);
        lv_obj_set_style_border_color(b, lv_color_hex(P->border), 0);
        lv_obj_set_style_border_opa(b, LV_OPA_COVER, 0);
        lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(b, nav_clicked, LV_EVENT_CLICKED,
                            (void*)(intptr_t)i);

        nav_lbl[i] = label(b, names[i], P->dim, FONT_CAP);
        lv_obj_center(nav_lbl[i]);
        nav_btn[i] = b;
    }

    show_tab(0);
    refresh_motors();
    refresh_status();
}

// =============================================================================
// Refresh loop
// =============================================================================

static void screen_step(bool force) {
    static uint32_t last_motor = 0;
    static uint32_t last_status = 0;
    static uint32_t last_telem = 0;

    uint32_t now = pros::millis();

    // LVGL is already driven by the PROS LVGL port's own display/touch timer
    // task. Running a second handler here from a PROS task meant two runners
    // mutating the same objects unlocked, which froze the UI.
#ifdef BUILD_SCREEN_SIM
    lv_timer_handler();
#endif

    char buf[32];
    snprintf(buf, sizeof(buf), "pos %.0f", cascade_position());
    lv_label_set_text(header_pos, buf);

    // Disabled: this now runs inside an LVGL timer callback, and blocking
    // telemetry logging here would delay touch handling. Move it to its own
    // task if it is needed again.
    // if (force || (now - last_telem) >= 50) {
    //     last_telem = now;
    //     lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
    // }

    if (force || (now - last_motor) >= 500) {
        last_motor = now;
        if (active_tab == 1 || force) refresh_motors();
    }

    if (force || (now - last_status) >= 100) {
        last_status = now;
        if (active_tab == 2 || force) refresh_status();
    }

    // The robot marker tracks the live pose whenever the field is on screen
    if (active_tab == 0 || force) update_robot_marker();
}

void screen_task() {
    lv_timer_create([](lv_timer_t*) {
        screen_step(false);
    }, 20, nullptr);
}

#ifdef BUILD_SCREEN_SIM
void sim_show_tab(int idx) {
    show_tab(idx);
}

void sim_refresh() {
    screen_step(true);
}
#endif
