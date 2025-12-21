void Input::onMouseMove(double x, double y) {
  // WSLg/Wayland safety: some stacks report (0,0) constantly in CURSOR_DISABLED.
  // If we see repeated 0,0, treat it as invalid and do NOT accumulate deltas.
  static int zeroStreak = 0;

  if (x == 0.0 && y == 0.0) {
    zeroStreak++;
    if (zeroStreak >= 2) {   // two frames in a row => likely bogus stream
      g_haveLast = false;
      return;
    }
  } else {
    zeroStreak = 0;
  }

  if (!g_haveLast) {
    g_lastX = x;
    g_lastY = y;
    g_haveLast = true;
    return;
  }

  double dx = x - g_lastX;
  double dy = y - g_lastY;

  g_lastX = x;
  g_lastY = y;

  // Clamp insane deltas (prevents runaway due to compositor glitches)
  const double maxDelta = 200.0;
  if (dx >  maxDelta) dx =  maxDelta;
  if (dx < -maxDelta) dx = -maxDelta;
  if (dy >  maxDelta) dy =  maxDelta;
  if (dy < -maxDelta) dy = -maxDelta;

  // If delta is still suspiciously large frequently, you can lower maxDelta.
  g_dx += static_cast<float>(dx);
  g_dy += static_cast<float>(dy);
}
