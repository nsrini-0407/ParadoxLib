#include "gui/gui.hpp"
#include "liblvgl/lvgl.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <cstdio>

namespace gui {

//Logo
static const lv_image_dsc_t* g_logo_dsc = nullptr;
void setLogoImage(const void* img_dsc) { g_logo_dsc = static_cast<const lv_image_dsc_t*>(img_dsc); }

static std::vector<AutonRoutine> g_routines = {
    {"Left WP",   []() { printf("Running: Left WP\n"); }},
    {"Right WP",  []() { printf("Running: Right WP\n"); }},
    {"Skills",    []() { printf("Running: Skills\n"); }},
    {"Do Nothing",[]() { printf("Running: Do Nothing\n"); }},
};

static int g_selected = -1;
static std::vector<WatchedMotor> g_motors;
static std::vector<lv_obj_t*> g_auton_btns;
static bool g_locked = false;

int getSelectedAutonIndex() { return g_selected; }
std::string getSelectedAutonName() {
    return (g_selected >= 0 && g_selected < (int)g_routines.size()) ? g_routines[g_selected].name : "NONE SELECTED";
}
void runSelectedAuton() {
    if (g_selected >= 0 && g_selected < (int)g_routines.size()) g_routines[g_selected].run();
}
void setWatchedMotors(const std::vector<WatchedMotor>& motors) { g_motors = motors; }

//  styling helpers 
static lv_style_t style_btn_default, style_btn_selected, style_card;

static void init_styles() {
    lv_style_init(&style_btn_default);
    lv_style_set_bg_color(&style_btn_default, lv_color_hex(0x2b2b2b));
    lv_style_set_radius(&style_btn_default, 8);
    lv_style_set_border_width(&style_btn_default, 2);
    lv_style_set_border_color(&style_btn_default, lv_color_hex(0x444444));

    lv_style_init(&style_btn_selected);
    lv_style_set_bg_color(&style_btn_selected, lv_color_hex(0x0072e3));
    lv_style_set_radius(&style_btn_selected, 8);
    lv_style_set_border_width(&style_btn_selected, 2);
    lv_style_set_border_color(&style_btn_selected, lv_color_hex(0x00baff));

    lv_style_init(&style_card);
    lv_style_set_bg_color(&style_card, lv_color_hex(0x1c1c1c));
    lv_style_set_radius(&style_card, 6);
    lv_style_set_pad_all(&style_card, 6);
}

//  Home tab 
static lv_obj_t* lbl_status;
static lv_obj_t* lbl_battery;

static void build_home_tab(lv_obj_t* tab) {
    if (g_logo_dsc) {
        lv_obj_t* img = lv_image_create(tab);
        lv_image_set_src(img, g_logo_dsc);
        lv_obj_align(img, LV_ALIGN_TOP_LEFT, 5, 5);
    } else {
        lv_obj_t* placeholder = lv_obj_create(tab);
        lv_obj_add_style(placeholder, &style_card, 0);
        lv_obj_set_size(placeholder, 60, 60);
        lv_obj_align(placeholder, LV_ALIGN_TOP_LEFT, 5, 5);
        lv_obj_t* lbl = lv_label_create(placeholder);
        lv_label_set_text(lbl, "LOGO");
        lv_obj_center(lbl);
    }

    lv_obj_t* team = lv_label_create(tab);
    lv_label_set_text(team, "81Y - Paradox");
    lv_obj_set_style_text_font(team, &lv_font_montserrat_24, 0);
    lv_obj_align(team, LV_ALIGN_TOP_RIGHT, -10, 10);

    lbl_status = lv_label_create(tab);
    lv_label_set_text(lbl_status, "Status: --");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_RIGHT, -10, 45);

    lbl_battery = lv_label_create(tab);
    lv_label_set_text(lbl_battery, "Battery: --%");
    lv_obj_align(lbl_battery, LV_ALIGN_TOP_RIGHT, -10, 70);

    lv_obj_t* build = lv_label_create(tab);
    lv_label_set_text_fmt(build, "Build: %s %s", __DATE__, __TIME__);
    lv_obj_set_style_text_color(build, lv_color_hex(0x888888), 0);
    lv_obj_align(build, LV_ALIGN_BOTTOM_LEFT, 5, -5);
}

//  Autonomous tab 
static void auton_btn_event_cb(lv_event_t* e) {
    if (g_locked) return;

    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    g_selected = idx;

    for (size_t i = 0; i < g_auton_btns.size(); i++) {
        lv_obj_t* btn = g_auton_btns[i];

        // Remove only OUR color styles—keep the button's size and position.
        lv_obj_remove_style(btn, &style_btn_default, LV_PART_MAIN);
        lv_obj_remove_style(btn, &style_btn_selected, LV_PART_MAIN);

        lv_obj_add_style(
            btn,
            (int)i == idx ? &style_btn_selected : &style_btn_default,
            LV_PART_MAIN
        );
    }
}

static lv_obj_t* lbl_selected_display;

static void build_auton_tab(lv_obj_t* tab) {
    lv_obj_t* header = lv_label_create(tab);
    lv_label_set_text(header, "Select Autonomous:");
    lv_obj_align(header, LV_ALIGN_TOP_LEFT, 5, 2);

    int x = 5, y = 30, w = 150, h = 45, gap = 10;
    for (size_t i = 0; i < g_routines.size(); i++) {
        lv_obj_t* btn = lv_button_create(tab);
        lv_obj_set_size(btn, w, h);
        lv_obj_set_pos(btn, x + (i % 3) * (w + gap), y + (i / 3) * (h + gap));
        lv_obj_add_style(btn, &style_btn_default, 0);
        lv_obj_add_event_cb(btn, auton_btn_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, g_routines[i].name.c_str());
        lv_obj_center(lbl);

        g_auton_btns.push_back(btn);
    }

    lbl_selected_display = lv_label_create(tab);
    lv_obj_set_style_text_font(lbl_selected_display, &lv_font_montserrat_20, 0);
    lv_obj_align(lbl_selected_display, LV_ALIGN_BOTTOM_LEFT, 5, -8);
    lv_label_set_text(lbl_selected_display, "Selected: NONE - pick one!");
    lv_obj_set_style_text_color(lbl_selected_display, lv_color_hex(0xff5050), 0);
}

//  Motors tab 
static std::vector<lv_obj_t*> g_motor_rows;   // container per motor
static std::vector<lv_obj_t*> g_motor_labels; // text label per motor

static void build_motors_tab(lv_obj_t* tab) {
    int y = 4;
    for (size_t i = 0; i < g_motors.size(); i++) {
        lv_obj_t* row = lv_obj_create(tab);
        lv_obj_add_style(row, &style_card, 0);
        lv_obj_set_size(row, 460, 26);
        lv_obj_set_pos(row, 4, y);
        lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* lbl = lv_label_create(row);
        lv_label_set_text(lbl, "--");
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 4, 0);

        g_motor_rows.push_back(row);
        g_motor_labels.push_back(lbl);
        y += 30;
    }
    if (g_motors.empty()) {
        lv_obj_t* lbl = lv_label_create(tab);
        lv_label_set_text(lbl, "No motors registered.\nCall gui::setWatchedMotors() before gui::init().");
    }
}

// Color thresholds - VEX motors start throttling around 55C and the
// cartridge/motor is generally considered "hot" near the 60-65C mark.
static lv_color_t temp_color(double c) {
    if (c >= 55) return lv_color_hex(0xff3b30);   // red - hotswap now
    if (c >= 45) return lv_color_hex(0xffcc00);   // yellow - watch it
    return lv_color_hex(0x30d158);                // green - fine
}

//  background telemetry task 
static void telemetry_task() {
    while (true) {
        // battery + connection status
        char buf[32];
        snprintf(buf, sizeof(buf), "Battery: %d%%", (int)pros::battery::get_capacity());
        lv_label_set_text(lbl_battery, buf);

        const char* status = pros::competition::is_disabled()   ? "Status: Disabled"
                              : pros::competition::is_autonomous() ? "Status: Autonomous"
                                                                     : "Status: Driver Control";
        lv_label_set_text(lbl_status, status);

        // once the field/competition switch flips out of disabled the FIRST
        // time, lock the auton selector so nobody can bump it mid-match
        // Selectable whenever the robot is disabled; locked only while enabled.
        g_locked = !pros::competition::is_disabled();

        // update selected-auton readout
        char sel[48];
        snprintf(sel, sizeof(sel), "Selected: %s%s", getSelectedAutonName().c_str(), g_locked ? " (LOCKED)" : "");
        lv_label_set_text(lbl_selected_display, sel);
        lv_obj_set_style_text_color(lbl_selected_display, g_selected >= 0 ? lv_color_hex(0x30d158) : lv_color_hex(0xff5050), 0);

        // update each watched motor row
        for (size_t i = 0; i < g_motors.size(); i++) {
            pros::Motor* m = g_motors[i].motor;
            double temp = m->get_temperature();
            double cur = m->get_current_draw() / 1000.0;  // mA -> A
            bool connected = m->is_installed();

            char line[64];
            if (!connected) {
                snprintf(line, sizeof(line), "%-10s DISCONNECTED", g_motors[i].label.c_str());
                lv_obj_set_style_text_color(g_motor_labels[i], lv_color_hex(0xff3b30), 0);
            } else {
                snprintf(line, sizeof(line), "%-10s %5.1fC   %4.1fA", g_motors[i].label.c_str(), temp, cur);
                lv_obj_set_style_text_color(g_motor_labels[i], lv_color_white(), 0);
                lv_obj_set_style_bg_color(g_motor_rows[i], temp_color(temp), 0);
                lv_obj_set_style_bg_opa(g_motor_rows[i], LV_OPA_30, 0);
            }
            lv_label_set_text(g_motor_labels[i], line);
        }

        pros::delay(200);
    }
}

// initialization
void init() {
    init_styles();

    lv_obj_t* tabview = lv_tabview_create(lv_screen_active());
    lv_tabview_set_tab_bar_position(tabview, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(tabview, 40);

    lv_obj_t* tab_home   = lv_tabview_add_tab(tabview, "Home");
    lv_obj_t* tab_auton  = lv_tabview_add_tab(tabview, "Autonomous");
    lv_obj_t* tab_motors = lv_tabview_add_tab(tabview, "Motors");

    build_home_tab(tab_home);
    build_auton_tab(tab_auton);
    build_motors_tab(tab_motors);

    pros::Task(telemetry_task, "gui_telemetry");
}

}  // namespace gui
