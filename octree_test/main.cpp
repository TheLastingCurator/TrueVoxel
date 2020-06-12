// Copyright (c) <year> Your name

#include "engine/easy.h"

using namespace arctic;  // NOLINT
using namespace arctic::easy;  // NOLINT

double g_prev_time;
double g_cur_time;
Font g_font;

double g_displayed_dt = 1.0/60.0;
double g_time_to_displayed_dt_change = 0.0;

void EasyMain() {
  ResizeScreen(1920, 1080);
  g_font.Load("data/arctic_one_bmf.fnt");
  g_cur_time = Time();
  while (!IsKeyDownward(kKeyEscape)) {
    g_prev_time = g_cur_time;
    g_cur_time = Time();
    double dt = g_cur_time - g_prev_time;
    g_time_to_displayed_dt_change -= dt;
    if (g_time_to_displayed_dt_change <= 0.0) {
      g_time_to_displayed_dt_change = 0.2;
      g_displayed_dt = dt;
    }

    Clear();

    char message[4096];
    snprintf(message, 4096, "FPS: %2.1f (%2.2f msPF)", 1.f / std::max(1e-9, g_displayed_dt), g_displayed_dt*1000.f);
    g_font.Draw(message, 0, ScreenSize().y, kTextOriginTop);
    ShowFrame();
  }
}
