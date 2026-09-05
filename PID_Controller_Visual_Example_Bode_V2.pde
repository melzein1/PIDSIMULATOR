/**
 * PID Robot Arm Simulator — Bode + Nyquist Analysis v1.5.1
 *
 * By: Mohammad Elzein 08/15/2025
 * Changes
 *  - Spring is tied to TARGET angle: τspring = −k(θ − θ_target)
 *  - New input: Arm Length (m); payload distance clamped ≤ L
 *  - Left panel uses a true scrollable viewport with wheel + draggable scrollbar
 *  - Time plot: adaptive Y range over last 10 s with nice ticks
 *  - Bode plot: cleaned background, Y‑axis labels for dB/deg
 *  - Nyquist plot can toggle between open-loop L(jw) and closed-loop T(jw)
 *  - Bode linearization angle is now an active, single-angle input
 *  - Resize-safe window handling: native maximize + taskbar-safe F toggle
 */

import processing.event.MouseEvent;
import processing.awt.PSurfaceAWT;
import java.awt.Frame;
import java.awt.Rectangle;
import java.awt.Insets;
import java.awt.GraphicsEnvironment;

// ---------- Window / resize handling ----------
boolean fillScreenToggle = false; // runtime "fit to usable desktop" mode
boolean fLatch = false;           // debounce for F key
final int WINDOW_W = 1200, WINDOW_H = 800;

// F-mode restores only through Processing's surface API.  We intentionally do
// NOT change the native Frame maximize state or call Frame.setBounds().
int savedContentW = WINDOW_W, savedContentH = WINDOW_H;
int savedWindowX = -1, savedWindowY = -1;
long lastWindowToggleMs = -1000;

// Resize debounce: native Windows maximize can report several intermediate
// drawable sizes.  Wait until the size is stable before rebuilding the layout.
int observedW = -1, observedH = -1;
int resizeStableFrames = 0;
final int RESIZE_SETTLE_FRAMES = 3;

// ---------- Globals ----------
ArmSim sim; PID pid; Plot plot; HUD hud; BodePlot bode; NyquistPlot nyquist;
ArrayList<InputField> fields = new ArrayList<InputField>();
Button btnReset, btnZeroI, btnDisturb;
Button btnStepUp, btnStepDown, btnSineToggle, btnRunBode, btnNyquistMode;
InputField fKp, fKi, fKd, fTarget, fArmLen, fMass, fPayload, fPayloadDist, fDrag, fSpring, fTlim;
InputField fAmp, fFreq, fCenter;
InputField fBodeFreq, fBodeAngle;

float pxScale;                 // meters -> pixels for arm length
int lastW = -1, lastH = -1;    // detect window size change
float panelX, panelY, panelW;  // layout
float armCX, armCY;            // arm center
float uiScale = 1;             // global UI scale factor

// panel scrolling support
float panelScroll = 0;       // 0 = top; negative scrolls upward
float panelScrollMin = 0;    // most negative allowed (computed in layout)
float panelViewTop, panelViewBottom, panelViewH, panelContentH;
float panelScrollBarX, panelScrollBarW;
boolean panelScrollDragging = false;
float panelScrollDragOffset = 0;

// PID term visualization
float pTerm=0, iTerm=0, dTermOut=0, uCmd=0, uSat=0;

// Setpoint generator state
boolean sineOn = false;
float ampDeg = 10.0;   // sine amplitude
float freqHz = 0.5;    // sine frequency
float centerDeg;       // sine center
float simTime = 0;     // seconds

void settings() {
  // Create the renderer at the intended size from the start.  This is much
  // more reliable during native Windows maximize/restore than resizing the
  // default 100x100 Processing surface from setup().
  size(WINDOW_W, WINDOW_H, JAVA2D);
  smooth(4);
}

void setup() {
  surface.setResizable(true);
  centerWindow(WINDOW_W, WINDOW_H);

  // Model defaults
  float L = 0.5; // m
  sim = new ArmSim(L);
  sim.massArm = 1.5;
  sim.payloadMass = 0.2;
  sim.payloadDist = 0.45;
  sim.drag = 0.12;
  sim.springK = 0.0;   // Nm/rad (modifiable via UI)
  sim.torqueLimit = 20.0;
  sim.theta = radians(-20);
  sim.thetaDot = 0;
  sim.targetDeg = 30;

  // PID defaults
  pid = new PID(); pid.Kp = 5.0; pid.Ki = 0.0; pid.Kd = 0.0; pid.derivLPF = 0.15; // LPF lerp

  // UI elements (positions will be updated by layoutUI)
  fKp          = mkField("Kp",  0,0, 140,38, nf(pid.Kp, 0, 3));
  fKi          = mkField("Ki",  0,0, 140,38, nf(pid.Ki, 0, 3));
  fKd          = mkField("Kd",  0,0, 140,38, nf(pid.Kd, 0, 3));
  fTarget      = mkField("Target (deg)",   0,0, 140,38, str(sim.targetDeg));
  fArmLen      = mkField("Arm Length (m)", 0,0, 140,38, nf(sim.L, 0, 3));
  fMass        = mkField("ArmMass (kg)",   0,0, 140,38, nf(sim.massArm, 0, 3));
  fPayload     = mkField("Payload (kg)",   0,0, 140,38, nf(sim.payloadMass, 0, 3));
  fPayloadDist = mkField("PayloadDist (m)",0,0, 140,38, nf(sim.payloadDist, 0, 3));
  fDrag        = mkField("Drag (Nm*s/rad)",0,0, 140,38, nf(sim.drag, 0, 3));
  fSpring      = mkField("Spring (Nm/rad)",0,0, 140,38, nf(sim.springK, 0, 3));
  fTlim        = mkField("TorqueLimit (Nm)",0,0,140,38, nf(sim.torqueLimit, 0, 2));

  // Setpoint generator fields
  fAmp   = mkField("Sine Amp (deg)",   0,0, 140,38, nf(ampDeg, 0, 2));
  fFreq  = mkField("Sine Freq (Hz)",   0,0, 140,38, nf(freqHz, 0, 2));
  centerDeg = sim.targetDeg; fCenter= mkField("Sine Center (deg)",0,0, 140,38, nf(centerDeg, 0, 2));

  // Bode controls (ranges as text "a-b")
  fBodeFreq  = mkField("Bode Freq (Hz a-b)", 0,0, 140,38, "0.1-10");
  fBodeAngle = mkField("Bode Lin. Angle (deg)",0,0,140,38, nf(sim.targetDeg, 0, 1));

  // Buttons
  btnStepUp     = new Button("Step +10°",     0,0, 160,40);
  btnStepDown   = new Button("Step −10°",     0,0, 160,40);
  btnSineToggle = new Button("Sine: OFF",     0,0, 160,40);
  btnRunBode    = new Button("Run Bode & Nyquist ",      0,0, 160,40);
  btnNyquistMode = new Button("Nyquist: OPEN LOOP", 0,0, 160,40);
  btnReset      = new Button("Reset",         0,0, 160,40);
  btnZeroI      = new Button("Zero Integral", 0,0, 160,40);
  btnDisturb    = new Button("Disturb (+Nm)", 0,0, 160,40);

  // Plots
  plot = new Plot(0,0, 700,260, 2400); plot.windowSeconds = 10; // rolling 10s window
  bode = new BodePlot();
  nyquist = new NyquistPlot();
  hud = new HUD();

  layoutUI();
}

void draw() {
  // Native Windows maximize/restore may produce several intermediate sizes.
  // Do not draw the full GUI until the Processing drawable size has remained
  // unchanged for a few frames.
  if (width != observedW || height != observedH) {
    observedW = width;
    observedH = height;
    resizeStableFrames = 0;
    background(12,14,20);
    return;
  }

  if (resizeStableFrames < RESIZE_SETTLE_FRAMES) {
    resizeStableFrames++;
    background(12,14,20);
    return;
  }

  // During minimize the drawable area can be momentarily tiny.
  if (width < 320 || height < 240) {
    background(12,14,20);
    return;
  }

  if (width != lastW || height != lastH) {
    layoutUI();
    lastW = width;
    lastH = height;
  }
  background(12,14,20);

  // --- Update setpoint from generator (if active) ---
  if (sineOn) { sim.targetDeg = centerDeg + ampDeg * sin(TWO_PI * freqHz * simTime); fTarget.text = nf(sim.targetDeg, 0, 2); }

  applyFieldValues();

  // --- Control law ---
  float dt = 1.0/120.0;
  float targetRad = radians(sim.targetDeg);
  float error = angleWrap(targetRad - sim.theta);
  float errorDot = -sim.thetaDot;

  float dFilt = pid.derivative(errorDot, dt);
  pTerm = pid.Kp * error; iTerm = pid.Ki * pid.integral; dTermOut = pid.Kd * dFilt; uCmd = pTerm + iTerm + dTermOut;

  uSat = clamp(uCmd, -sim.torqueLimit, sim.torqueLimit);
  boolean saturated = (abs(uCmd - uSat) > 1e-6); boolean windupDirection = (sign(uCmd) == sign(error));
  if (!(saturated && windupDirection)) {
    pid.integral += error * dt;
    // Clamp integral so Ki*integral cannot demand more than ~90% of torque limit
    if (pid.Ki > 0) {
      float iClamp = (0.9 * sim.torqueLimit) / max(1e-6, pid.Ki);
      pid.integral = clamp(pid.integral, -iClamp, iClamp);
    } else {
      pid.integral = clamp(pid.integral, -5.0, 5.0);
    }
  }

  sim.step(uSat, dt); simTime += dt;

  // Plots
  plot.addSample(simTime, degrees(sim.theta), sim.targetDeg);

  // Draw
  drawArm(sim);
  drawPanel();
  plot.draw();
  bode.draw();
  nyquist.draw();
  hud.draw(plot);
  drawFooter();
}

// ---------- Fullscreen / Window helpers ----------
// getMaximumWindowBounds() is the usable Windows desktop area with taskbar
// space removed.  Unlike the prior version, we do NOT call setMaximizedBounds(),
// setExtendedState(), or Frame.setBounds(); those native AWT operations can race
// Processing's drawing surface during maximize/restore.
Rectangle getUsableDesktopBounds() {
  try {
    return GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
  } catch (Exception e) {
    return new Rectangle(0, 0, displayWidth, displayHeight);
  }
}

Frame getNativeFrame() {
  try {
    Object nativeSurface = surface.getNative();
    if (nativeSurface instanceof PSurfaceAWT.SmoothCanvas) {
      return ((PSurfaceAWT.SmoothCanvas)nativeSurface).getFrame();
    }
  } catch (Exception e) {
    println("Window-frame lookup warning: " + e.getMessage());
  }
  return null;
}

void centerWindow(int contentW, int contentH) {
  Rectangle work = getUsableDesktopBounds();
  Frame frame = getNativeFrame();

  int outerW = contentW;
  int outerH = contentH;
  if (frame != null) {
    Insets in = frame.getInsets();
    outerW += in.left + in.right;
    outerH += in.top + in.bottom;
  }

  int x = work.x + max(0, (work.width  - outerW) / 2);
  int y = work.y + max(0, (work.height - outerH) / 2);
  surface.setLocation(x, y);
}

void applyWindowMode() {
  lastWindowToggleMs = millis();

  Rectangle work = getUsableDesktopBounds();
  Frame frame = getNativeFrame();

  if (fillScreenToggle) {
    // Save the current *content* size and outer-window location.
    savedContentW = max(640, width);
    savedContentH = max(480, height);
    if (frame != null) {
      savedWindowX = frame.getX();
      savedWindowY = frame.getY();
    }

    // Compute a taskbar-safe Processing content size by subtracting title-bar
    // and border insets from the usable outer desktop bounds.
    int insetLR = 0;
    int insetTB = 0;
    if (frame != null) {
      Insets in = frame.getInsets();
      insetLR = in.left + in.right;
      insetTB = in.top + in.bottom;
    }

    int safeW = max(640, work.width  - insetLR);
    int safeH = max(480, work.height - insetTB);

    // Use Processing's supported surface API only.
    surface.setSize(safeW, safeH);
    surface.setLocation(work.x, work.y);
  } else {
    // Restore through the same Processing surface API.  Do not touch the
    // native maximize state or native frame bounds.
    int restoreW = max(640, savedContentW);
    int restoreH = max(480, savedContentH);
    surface.setSize(restoreW, restoreH);

    if (savedWindowX >= 0 && savedWindowY >= 0) {
      surface.setLocation(savedWindowX, savedWindowY);
    } else {
      centerWindow(restoreW, restoreH);
    }
  }

  // Force a clean layout rebuild only after the new drawable size settles.
  observedW = -1;
  observedH = -1;
  resizeStableFrames = 0;
  lastW = -1;
  lastH = -1;
  cursor();
}

// ---------- Layout + Scroll ----------
void layoutUI() {
  // Scale text and controls, but do not compress rows until they become unreadable.
  uiScale = constrain(min(width, height) / 900.0, 0.75, 1.35);

  panelX = 24 * uiScale;
  panelY = 30 * uiScale;
  panelW = constrain(width * 0.24, 300 * uiScale, 430 * uiScale);

  float footerH = 30 * uiScale;
  panelViewTop = panelY;
  panelViewBottom = height - footerH - 8 * uiScale;
  panelViewH = max(120 * uiScale, panelViewBottom - panelViewTop);

  float rowH = 38 * uiScale;
  float rowGap = 7 * uiScale;
  float sectionGap = 10 * uiScale;
  float w = panelW - 18 * uiScale;  // leave room for the scrollbar

  // 16 fields + 8 buttons = 24 rows.  There are 6 larger section breaks.
  int itemCount = 24;
  int normalGaps = itemCount - 1;
  int sectionBreaks = 6;
  panelContentH = itemCount * rowH + normalGaps * rowGap + sectionBreaks * sectionGap;

  panelScrollMin = min(0, panelViewH - panelContentH);
  panelScroll = constrain(panelScroll, panelScrollMin, 0);

  panelScrollBarW = max(7 * uiScale, 5);
  panelScrollBarX = panelX + panelW - panelScrollBarW;

  float x = panelX;
  float y = panelViewTop + panelScroll;

  // Fields
  fKp.setPos(x, y, w, rowH); y += rowH + rowGap;
  fKi.setPos(x, y, w, rowH); y += rowH + rowGap;
  fKd.setPos(x, y, w, rowH); y += rowH + rowGap + sectionGap;

  fTarget.setPos(x, y, w, rowH); y += rowH + rowGap + sectionGap;

  fArmLen.setPos(x, y, w, rowH); y += rowH + rowGap;
  fMass.setPos(x, y, w, rowH); y += rowH + rowGap;
  fPayload.setPos(x, y, w, rowH); y += rowH + rowGap;
  fPayloadDist.setPos(x, y, w, rowH); y += rowH + rowGap;
  fDrag.setPos(x, y, w, rowH); y += rowH + rowGap;
  fSpring.setPos(x, y, w, rowH); y += rowH + rowGap;
  fTlim.setPos(x, y, w, rowH); y += rowH + rowGap + sectionGap;

  // Setpoint generator
  fAmp.setPos(x, y, w, rowH); y += rowH + rowGap;
  fFreq.setPos(x, y, w, rowH); y += rowH + rowGap;
  fCenter.setPos(x, y, w, rowH); y += rowH + rowGap + sectionGap;

  // Frequency-response controls
  fBodeFreq.setPos(x, y, w, rowH); y += rowH + rowGap;
  fBodeAngle.setPos(x, y, w, rowH); y += rowH + rowGap + sectionGap;

  // Buttons
  btnStepUp.setPos(x, y, w, rowH); y += rowH + rowGap;
  btnStepDown.setPos(x, y, w, rowH); y += rowH + rowGap;
  btnSineToggle.setPos(x, y, w, rowH); y += rowH + rowGap + sectionGap;

  btnRunBode.setPos(x, y, w, rowH); y += rowH + rowGap;
  btnNyquistMode.setPos(x, y, w, rowH); y += rowH + rowGap;
  btnReset.setPos(x, y, w, rowH); y += rowH + rowGap;
  btnZeroI.setPos(x, y, w, rowH); y += rowH + rowGap;
  btnDisturb.setPos(x, y, w, rowH);

  // Right-side plots
  float rightX = panelX + panelW + 26 * uiScale;
  float rightW = max(440 * uiScale, width - rightX - 24 * uiScale);

  float topH = max(235 * uiScale, height * 0.30);
  plot.setRect(rightX, 18 * uiScale, rightW, topH);
  plot.lineWeight = 2.4 * uiScale;
  plot.gridWeight = 1.2 * uiScale;
  plot.textSize = 12 * uiScale;
  plot.labelPadding = 8 * uiScale;

  // Bode and Nyquist share the frequency-response row.
  float freqY = plot.y + topH + 12 * uiScale;
  float freqH = max(220 * uiScale, min(height * 0.29, 290 * uiScale));
  float freqGap = 12 * uiScale;
  float nyqW = constrain(rightW * 0.36, 250 * uiScale, 360 * uiScale);
  float bodeW = max(300 * uiScale, rightW - nyqW - freqGap);

  bode.setRect(rightX, freqY, bodeW, freqH);
  nyquist.setRect(rightX + bodeW + freqGap, freqY, nyqW, freqH);

  // HUD near top plot
  hud.x = rightX + rightW - 310 * uiScale;
  hud.y = plot.y + 10 * uiScale;
  hud.w = 300 * uiScale;
  hud.h = 170 * uiScale;
  hud.textSize = 12 * uiScale;

  // Arm area below the frequency-response plots; reserve the footer strip.
  float armTop = freqY + freqH + 10 * uiScale;
  float armBottom = height - footerH - 6 * uiScale;
  float armAvail = max(90 * uiScale, armBottom - armTop);
  armCX = rightX + rightW * 0.55;
  armCY = armTop + armAvail * 0.50;
  pxScale = min(rightW * 0.56, max(300 * uiScale, armAvail * 1.45));
}

float panelThumbH() {
  if (panelContentH <= panelViewH) return panelViewH;
  return max(36 * uiScale, panelViewH * (panelViewH / panelContentH));
}

float panelThumbY() {
  if (panelScrollMin >= 0) return panelViewTop;
  float travel = max(1, panelViewH - panelThumbH());
  float frac = constrain(panelScroll / panelScrollMin, 0, 1); // negative/negative -> 0..1
  return panelViewTop + frac * travel;
}

void setPanelScrollFromThumb(float thumbTop) {
  if (panelScrollMin >= 0) return;
  float travel = max(1, panelViewH - panelThumbH());
  float frac = constrain((thumbTop - panelViewTop) / travel, 0, 1);
  panelScroll = panelScrollMin * frac;
  layoutUI(); // scrolling changes every control's screen position
}

void mouseWheel(MouseEvent evt){
  if (mouseX >= panelX && mouseX <= panelX + panelW &&
      mouseY >= panelViewTop && mouseY <= panelViewBottom) {
    panelScroll = constrain(panelScroll - evt.getCount() * 34 * uiScale, panelScrollMin, 0);
    layoutUI(); // important: immediately move the controls after changing scroll
  }
}

// ---------- Input ----------
void mousePressed() {
  boolean inPanelView = (mouseX >= panelX && mouseX <= panelX + panelW &&
                         mouseY >= panelViewTop && mouseY <= panelViewBottom);

  // Scrollbar: drag the thumb or click the track to jump.
  if (inPanelView && panelScrollMin < 0 &&
      mouseX >= panelScrollBarX - 3*uiScale && mouseX <= panelX + panelW + 2*uiScale) {
    float th = panelThumbH();
    float ty = panelThumbY();
    if (mouseY >= ty && mouseY <= ty + th) {
      panelScrollDragging = true;
      panelScrollDragOffset = mouseY - ty;
    } else {
      setPanelScrollFromThumb(mouseY - th/2);
    }
    for (InputField f : fields) f.active = false;
    return;
  }

  // Hidden/scrolled-off controls are never allowed to receive clicks.
  for (InputField f : fields) {
    if (inPanelView) f.onMouse(mouseX, mouseY);
    else f.active = false;
  }

  if (!inPanelView) return;

  if (btnReset.hit(mouseX, mouseY)) resetSim();
  if (btnZeroI.hit(mouseX, mouseY)) pid.integral = 0;
  if (btnDisturb.hit(mouseX, mouseY)) sim.impulse(1.5);

  if (btnStepUp.hit(mouseX, mouseY)) {
    if (sineOn) { centerDeg += 10; fCenter.text = nf(centerDeg,0,2); }
    else { sim.targetDeg += 10; fTarget.text = nf(sim.targetDeg,0,2); }
  }
  if (btnStepDown.hit(mouseX, mouseY)) {
    if (sineOn) { centerDeg -= 10; fCenter.text = nf(centerDeg,0,2); }
    else { sim.targetDeg -= 10; fTarget.text = nf(sim.targetDeg,0,2); }
  }
  if (btnSineToggle.hit(mouseX, mouseY)) {
    sineOn = !sineOn;
    btnSineToggle.label = sineOn ? "Sine: ON" : "Sine: OFF";
    if (sineOn) centerDeg = parseSafe(fCenter.text, centerDeg);
  }
  if (btnRunBode.hit(mouseX, mouseY)) runBode();
  if (btnNyquistMode.hit(mouseX, mouseY)) {
    nyquist.showClosedLoop = !nyquist.showClosedLoop;
    btnNyquistMode.label = nyquist.showClosedLoop ? "Nyquist: CLOSED LOOP" : "Nyquist: OPEN LOOP";
  }
}

void mouseDragged() {
  if (panelScrollDragging) {
    setPanelScrollFromThumb(mouseY - panelScrollDragOffset);
  }
}

void mouseReleased() {
  panelScrollDragging = false;
}

void keyTyped() { for (InputField f : fields) if (f.active) { f.onKeyTyped(key, keyCode); if (key == ENTER || key == RETURN) f.active = false; } }
void keyPressed() {
  if (key == ESC) { key = 0; return; } if (key == 'q' || key == 'Q') exit();
  if ((key == 'f' || key == 'F') && !fLatch) {
    fLatch = true;
    if (millis() - lastWindowToggleMs >= 300) {
      fillScreenToggle = !fillScreenToggle;
      applyWindowMode();
    }
    return;
  }
  for (InputField f : fields) { if (!f.active) continue; if (keyCode == BACKSPACE) { if (f.text.length() > 0) f.text = f.text.substring(0, f.text.length()-1); return; } if (keyCode == DELETE) { f.text = ""; return; } if (keyCode == ENTER || key == RETURN) { f.active = false; return; } }
}
void keyReleased() { if (key == 'f' || key == 'F') fLatch = false; }

// ---------- Utils ----------
void resetSim() { sim.theta = radians(-10); sim.thetaDot = 0; pid.integral = 0; pid.dFilt = 0; simTime = 0; plot.clear(); }

void applyFieldValues() {
  if (!fKp.active) pid.Kp = parseSafe(fKp.text, pid.Kp);
  if (!fKi.active) pid.Ki = parseSafe(fKi.text, pid.Ki);
  if (!fKd.active) pid.Kd = parseSafe(fKd.text, pid.Kd);
  if (!fTarget.active && !sineOn) sim.targetDeg = parseSafe(fTarget.text, sim.targetDeg);
  if (!fArmLen.active)  sim.L = max(0.05, parseSafe(fArmLen.text, sim.L));
  if (!fMass.active) sim.massArm = max(0.01, parseSafe(fMass.text, sim.massArm));
  if (!fPayload.active) sim.payloadMass = max(0, parseSafe(fPayload.text, sim.payloadMass));
  if (!fPayloadDist.active) sim.payloadDist = constrain(parseSafe(fPayloadDist.text, sim.payloadDist), 0, sim.L);
  if (!fDrag.active) sim.drag = max(0, parseSafe(fDrag.text, sim.drag));
  if (!fSpring.active) sim.springK = max(0, parseSafe(fSpring.text, sim.springK));
  if (!fTlim.active) sim.torqueLimit = max(0.1, parseSafe(fTlim.text, sim.torqueLimit));
  if (!fAmp.active) ampDeg = max(0, parseSafe(fAmp.text, ampDeg));
  if (!fFreq.active) freqHz = max(0, parseSafe(fFreq.text, freqHz));
  if (!fCenter.active) centerDeg = parseSafe(fCenter.text, centerDeg);
}

float parseSafe(String s, float fallback) { try { return Float.parseFloat(s.trim()); } catch(Exception e) { return fallback; } }
float clamp(float v, float lo, float hi) { return max(lo, min(hi, v)); }
int sign(float v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }

// Wrap to [-PI, PI]
float angleWrap(float a) { a = (a + PI) % (2*PI); if (a < 0) a += 2*PI; return a - PI; }
float angleDiffDeg(float targetDeg, float currentDeg) { float d = ((targetDeg - currentDeg + 180) % 360); if (d < 0) d += 360; return d - 180; }

// ---------- Drawing ----------
void drawArm(ArmSim s) {
  noStroke(); fill(40,45,60,220); rect(armCX-30*uiScale, armCY-20*uiScale, 60*uiScale, 40*uiScale, 8*uiScale);
  float ang = s.theta; PVector tip = new PVector(armCX + pxScale*s.L*cos(ang), armCY - pxScale*s.L*sin(ang));
  stroke(210,220,240); strokeWeight(12*uiScale); line(armCX, armCY, tip.x, tip.y);
  float pd = s.payloadDist; PVector pb = new PVector(armCX + pxScale*pd*cos(ang), armCY - pxScale*pd*sin(ang));
  noStroke(); fill(120,200,255,220); float payloadSize = map(s.payloadMass, 0, 1.5, 8*uiScale, 22*uiScale); ellipse(pb.x, pb.y, payloadSize, payloadSize);
  fill(255, 160); ellipse(tip.x, tip.y, 10*uiScale, 10*uiScale);
  float thT = radians(s.targetDeg); stroke(140,180,255, 200); strokeWeight(2.2*uiScale); line(armCX, armCY, armCX + 90*uiScale*cos(thT), armCY - 90*uiScale*sin(thT));
}

void drawPanel() {
  pushStyle();

  // The left controls live in a true viewport.  Do not use nested clip()/noClip()
  // calls inside InputField.draw(), because Processing's clip state is not stacked.
  noStroke();
  fill(16,18,24);
  rect(panelX-4*uiScale, panelViewTop-4*uiScale, panelW+8*uiScale, panelViewH+8*uiScale, 6*uiScale);

  clip((int)panelX-2, (int)panelViewTop, (int)panelW+4, (int)panelViewH);
  for (InputField f : fields) f.draw();
  btnStepUp.draw();
  btnStepDown.draw();
  btnSineToggle.draw();
  btnRunBode.draw();
  btnNyquistMode.draw();
  btnReset.draw();
  btnZeroI.draw();
  btnDisturb.draw();
  noClip();

  // Visible scrollbar
  if (panelScrollMin < 0) {
    float th = panelThumbH();
    float ty = panelThumbY();
    noStroke();
    fill(55,60,72,190);
    rect(panelScrollBarX, panelViewTop, panelScrollBarW, panelViewH, panelScrollBarW/2);
    fill(panelScrollDragging ? color(150,210,255) : color(115,130,155));
    rect(panelScrollBarX, ty, panelScrollBarW, th, panelScrollBarW/2);

    fill(180);
    textAlign(LEFT, BOTTOM);
    textSize(11*uiScale);
    text("Mouse wheel or drag scrollbar", panelX, panelY - 5*uiScale);
  }

  // Time-plot title and legend
  fill(255);
  textAlign(LEFT, TOP);
  textSize(12*uiScale);
  text("Plot: angle vs time (last 10 s)", plot.x, plot.y - 16*uiScale);

float legW = 170*uiScale;

// Center the entire legend horizontally in the plot
float legX = plot.x + (plot.w - legW)/2.0;

float legY = plot.y + 12*uiScale;

  strokeWeight(4*uiScale);
  stroke(120,200,255);
  line(legX, legY, legX+36*uiScale, legY);
  noStroke();
  fill(200);
  textAlign(LEFT, CENTER);
  text("current", legX+44*uiScale, legY);

  strokeWeight(4*uiScale);
  stroke(255,120,160);
  line(legX, legY + 12*uiScale, legX+36*uiScale, legY + 12*uiScale);
  noStroke();
  fill(200);
  text("target", legX+44*uiScale, legY + 12*uiScale);

  popStyle();
}

void drawFooter() {
  pushStyle();
  float footerH = 30*uiScale;
  noStroke();
  fill(12,14,20,245);
  rect(plot.x, height-footerH, width-plot.x, footerH);

  fill(200);
  textAlign(LEFT, CENTER);
  textSize(12*uiScale);
  text("F: fit/restore usable screen  •  Q: quit  •  Click a box → type → Enter",
       plot.x + 6*uiScale, height-footerH/2);
  popStyle();
}

// ---------- Model ----------
class ArmSim { 
  float L, massArm, payloadMass, payloadDist, drag, springK, theta, thetaDot, targetDeg, torqueLimit; 
  final float g = 9.81; 
  ArmSim(float L){ this.L=L; }
  float inertia(){ return (1.0/3.0)*massArm*L*L + payloadMass*payloadDist*payloadDist; }
  float gravityK(){ return g * (massArm * L*0.5 + payloadMass * payloadDist); }
  float gravityTorque(){ return - gravityK() * cos(theta); }
  void step(float tauMotor, float dt){ 
    float I=max(1e-4, inertia()); 
    float thRef = radians(targetDeg); // spring rest tied to target
    float tauSpring = - springK * (theta - thRef);
    float tau = tauMotor + gravityTorque() - drag*thetaDot + tauSpring; 
    float thetaDD=tau/I; 
    thetaDot += thetaDD*dt; 
    theta += thetaDot*dt; 
  } 
  void impulse(float J){ float I=max(1e-4, inertia()); thetaDot += J / I; } 
}

class PID { float Kp=0, Ki=0, Kd=0; float integral=0, dFilt=0, derivLPF=0.15; float derivative(float raw, float _dt){ dFilt = lerp(dFilt, raw, clamp(derivLPF, 0, 1)); return dFilt; } }// ---------- HUD ----------
class HUD {
  float x, y, w, h, textSize=12;
  void draw(Plot p){
    pushStyle();
    // Build lines with explicit newlines and auto-size box
    // Compute a couple of derived spring stats for clarity
    float thRefHUD = radians(sim.targetDeg);
    float tauSpringHUD = - sim.springK * (sim.theta - thRefHUD);

    String[] lines = new String[]{
      "Current: " + nf(degrees(sim.theta),0,2) + "°",
      "Target : " + nf(sim.targetDeg,0,2) + "°",
      "Error  : " + nf(angleDiffDeg(sim.targetDeg, degrees(sim.theta)),0,2) + "°",
      "Torque (Nm)  Cmd: " + nf(uCmd,0,2) + "  Sat: " + nf(uSat,0,2),
      "P/I/D (Nm): " + nf(pTerm,0,2) + ", " + nf(iTerm,0,2) + ", " + nf(dTermOut,0,2),
      "k_spring (Nm/rad): " + nf(sim.springK,0,3),
      "τ_spring (Nm): " + nf(tauSpringHUD,0,2) + "  (θ_ref = " + nf(sim.targetDeg,0,2) + "°)"
    };

    textSize(textSize);
    float pad = 10*uiScale;
    float lead = textAscent() + textDescent() + 2*uiScale;
    float maxW = 0; for (String s : lines) maxW = max(maxW, textWidth(s));
    float boxW = max(220*uiScale, maxW + 2*pad);
    float boxH = lines.length*lead + 2*pad;

    // keep inside top plot area (p)
    float bx = constrain(x, p.x + 8*uiScale, p.x + p.w - boxW - 8*uiScale);
    float by = constrain(y, p.y + 8*uiScale, p.y + p.h - boxH - 8*uiScale);

    // panel
    noStroke(); fill(20, 24, 34, 220); rect(bx, by, boxW, boxH, 8*uiScale);
    stroke(120, 200, 255, 150); noFill(); strokeWeight(1.5*uiScale); rect(bx, by, boxW, boxH, 8*uiScale);

    // text
    fill(230); textAlign(LEFT, TOP);
    float ty = by + pad; float tx = bx + pad;
    for (String s : lines) { text(s, tx, ty); ty += lead; }
    popStyle();
  }
}

// ---------- Plot with adaptive Y for last 10s ----------
class Plot {
  float x, y, w, h; int capacity; float lineWeight=2, gridWeight=1.2, textSize=12, labelPadding=8; float windowSeconds = 10;
  ArrayList<Float> a1 = new ArrayList<Float>();
  ArrayList<Float> a2 = new ArrayList<Float>();
  ArrayList<Float> tt = new ArrayList<Float>();
  Plot(float x,float y,float w,float h,int cap){ setRect(x,y,w,h); capacity=cap; }
  void setRect(float X,float Y,float W,float H){ x=X; y=Y; w=W; h=H; }
  void clear(){ a1.clear(); a2.clear(); tt.clear(); }
  void addSample(float t, float curDeg, float tgtDeg){ a1.add(curDeg); a2.add(tgtDeg); tt.add(t); while(a1.size()>capacity){ a1.remove(0); a2.remove(0); tt.remove(0);} }
  void draw(){
    noFill(); stroke(220,230,240,160); strokeWeight(gridWeight); rect(x, y, w, h, 6*uiScale);
    if (a1.size()<2) return;
    float tLast = tt.get(tt.size()-1);
    float tStart = max(tt.get(0), tLast - windowSeconds);
    float span = max(0.001, tLast - tStart);
    // compute min/max over window
    float vMin=  1e9, vMax=-1e9; 
    for (int i=0;i<a1.size();i++){ float t=tt.get(i); if (t < tStart) continue; vMin=min(vMin, min(a1.get(i), a2.get(i))); vMax=max(vMax, max(a1.get(i), a2.get(i))); }
    if (!Float.isFinite(vMin) || !Float.isFinite(vMax)) { vMin=-10; vMax=10; }
    float range=max(1e-6, vMax - vMin); float pad=max(2, range*0.12);
    float minY=floor((vMin - pad)); float maxY=ceil((vMax + pad));
    if (maxY - minY < 4) { minY = floor((vMin+vMax)/2 - 2); maxY = minY + 4; }
    float stepY = niceStep(maxY - minY);
    // grid + labels
    stroke(120,130,150,60); strokeWeight(gridWeight); fill(190); textSize(textSize); textAlign(LEFT, CENTER);
    for (float val = ceil(minY/stepY)*stepY; val <= maxY; val += stepY){ float yy=map(val, minY, maxY, y+h-6*uiScale, y+6*uiScale); line(x+6*uiScale, yy, x+w-6*uiScale, yy); noStroke(); text(nf(val,0,(stepY<1)?1:0) + "°", x+10*uiScale, yy-1); stroke(120,130,150,60); }
    // curves
    stroke(120,200,255); strokeWeight(lineWeight); noFill(); beginShape(); for (int i=0;i<a1.size();i++){ float t = tt.get(i); if (t < tStart) continue; float xx=map(t - tStart, 0, span, x+6*uiScale, x+w-6*uiScale); float yy=map(a1.get(i), minY, maxY, y+h-6*uiScale, y+6*uiScale); vertex(xx, yy);} endShape();
    stroke(255,120,160); strokeWeight(lineWeight); beginShape(); for (int i=0;i<a2.size();i++){ float t = tt.get(i); if (t < tStart) continue; float xx=map(t - tStart, 0, span, x+6*uiScale, x+w-6*uiScale); float yy=map(a2.get(i), minY, maxY, y+h-6*uiScale, y+6*uiScale); vertex(xx, yy);} endShape();
    // time ticks
    // time ticks (draw inside plot so Bode panel below can't cover them)
textAlign(CENTER, BOTTOM); fill(200); textSize(textSize); stroke(160,170,190,100); strokeWeight(1);
float stepT = 1; float firstTick = ceil(tStart/stepT)*stepT; 
for (float ts = firstTick; ts <= tLast; ts += stepT) { 
  float xx = map(ts - tStart, 0, span, x+6*uiScale, x+w-6*uiScale); 
  line(xx, y+h-4*uiScale, xx, y+h); 
  text(nf(ts, 0, 0) + " s", xx, y+h - 2*uiScale); 
}
  }
}


float niceStep(float range){ float rough = range/6.0; float pow10 = pow(10, floor(log10f(max(1e-6, rough)))); float base = rough / pow10; float mult = (base<=1.2)?1:(base<=2.5)?2:(base<=5.5)?5:10; return mult * pow10; }

class BodePlot {
  float x, y, w, h; float textSize=12; float lineW=2.2; float gridW=1.1; boolean hasData=false; float linAngleDeg=0; float[] fHz, magPlant, phaPlant, magCL, phaCL; // arrays
  void setRect(float X,float Y,float W,float H){ x=X; y=Y; w=W; h=H; }
  void draw(){
    pushStyle();
    // full background to avoid residual banding
    noStroke(); fill(18,20,26); rect(x, y, w, h);
    // Frame & title
    noFill(); stroke(220,230,240,160); strokeWeight(1.2*uiScale); rect(x, y, w, h, 6*uiScale);
    fill(200); textAlign(LEFT, TOP); textSize(12*uiScale); text("Bode: G(jω) and T(jω)  •  linearized at θ = " + nf(linAngleDeg,0,1) + "°", x+8*uiScale, y+6*uiScale);
    if (!hasData) { fill(170); text("Click 'Run Bode' to compute using current parameters", x+8*uiScale, y+28*uiScale); popStyle(); return; }

    float top = y + 30*uiScale; float mid = y + h*0.55; float bot = y + h - 24*uiScale; // two panels
    float left = x + 56*uiScale; float right = x + w - 16*uiScale;

    float fmin = max(1e-6, fHz[0]), fmax = fHz[fHz.length-1];
    float mMin =  1e9, mMax = -1e9; for (int i=0;i<magCL.length;i++){ mMin=min(mMin,min(magPlant[i],magCL[i])); mMax=max(mMax,max(magPlant[i],magCL[i])); }
    mMin = floor(mMin/5)*5; mMax = ceil(mMax/5)*5; if (mMax-mMin < 10) mMax=mMin+10;
    float pMin = -180, pMax = 180;

    // sub-panel backgrounds
    noStroke(); fill(24,28,36); rect(left, top, right-left, mid-top-8*uiScale); rect(left, mid+8*uiScale, right-left, bot-(mid+8*uiScale));

    // grids
    stroke(120,130,150,60); strokeWeight(gridW); for (float db=mMin; db<=mMax; db+=5) { float yy = map(db, mMin, mMax, mid-6*uiScale, top+6*uiScale); line(left, yy, right, yy);} 
    for (float deg=-180; deg<=180; deg+=30) { float yy = map(deg, pMin, pMax, bot-6*uiScale, mid+16*uiScale); line(left, yy, right, yy);} 

    // Y labels
    fill(185); textAlign(RIGHT, CENTER); textSize(11*uiScale);
    for (float db=mMin; db<=mMax; db+=10) { float yy = map(db, mMin, mMax, mid-6*uiScale, top+6*uiScale); text(nf(db,0,0)+" dB", left-8*uiScale, yy); }
    for (float deg=-180; deg<=180; deg+=45) { float yy = map(deg, pMin, pMax, bot-6*uiScale, mid+16*uiScale); text(nf(deg,0,0)+"°", left-8*uiScale, yy); }

    // log X ticks
    textAlign(CENTER, TOP); fill(190);
    for (float d=floor(log10f(fmin)); d<=ceil(log10f(fmax)); d+=1){ float ff = pow(10, d); if (ff < fmin) continue; float xx = mapLog(ff, fmin, fmax, left, right); stroke(120,130,150,70); line(xx, top, xx, bot); text(nfc(ff, (ff<1)?2: (ff<10?1:0)) + " Hz", xx, bot+2*uiScale);
      for (int k=2;k<10;k++){ float f2=ff*k; if (f2>=ff*10 || f2>fmax) break; float xx2=mapLog(f2, fmin, fmax, left, right); stroke(120,130,150,30); line(xx2, top, xx2, bot); }
    }

    // curves
    noFill(); strokeWeight(lineW);
    noFill();
stroke(160,170,190); beginShape(); for (int i=0;i<fHz.length;i++){ float xx=mapLog(fHz[i],fmin,fmax,left,right); float yy=map(magPlant[i], mMin, mMax, mid-6*uiScale, top+6*uiScale); vertex(xx,yy);} endShape();
    noFill();
stroke(120,200,255); beginShape(); for (int i=0;i<fHz.length;i++){ float xx=mapLog(fHz[i],fmin,fmax,left,right); float yy=map(magCL[i], mMin, mMax, mid-6*uiScale, top+6*uiScale); vertex(xx,yy);} endShape();
    fill(200); textAlign(LEFT, TOP); text("Mag (dB)  Plant: gray  •  Closed-loop: blue", left, top + 6*uiScale);

    noFill();
stroke(160,170,190); beginShape(); for (int i=0;i<fHz.length;i++){ float xx=mapLog(fHz[i],fmin,fmax,left,right); float yy=map(phaPlant[i], pMin, pMax, bot-6*uiScale, mid+16*uiScale); vertex(xx,yy);} endShape();
    noFill();
stroke(120,200,255); beginShape(); for (int i=0;i<fHz.length;i++){ float xx=mapLog(fHz[i],fmin,fmax,left,right); float yy=map(phaCL[i], pMin, pMax, bot-6*uiScale, mid+16*uiScale); vertex(xx,yy);} endShape();
    fill(200); textAlign(LEFT, TOP); text("Phase (deg)", left, mid + 10*uiScale);
    popStyle();
  }
}


// ---------- Nyquist plot: selectable open-loop L(jw) or closed-loop T(jw) ----------
class NyquistPlot {
  float x, y, w, h;
  float lineW=2.0, gridW=1.0;
  boolean hasData=false;
  boolean showClosedLoop=false;
  float linAngleDeg=0;
  float[] fHz;
  float[] reOpen, imOpen;
  float[] reClosed, imClosed;

  void setRect(float X, float Y, float W, float H){
    x=X; y=Y; w=W; h=H;
  }

  void draw(){
    pushStyle();

    noStroke();
    fill(18,20,26);
    rect(x, y, w, h);

    noFill();
    stroke(220,230,240,160);
    strokeWeight(1.2*uiScale);
    rect(x, y, w, h, 6*uiScale);

    float[] reData = showClosedLoop ? reClosed : reOpen;
    float[] imData = showClosedLoop ? imClosed : imOpen;
    String symbol = showClosedLoop ? "T" : "L";
    String modeText = showClosedLoop ? "closed-loop T(jω)" : "open-loop L(jω)";

    fill(200);
    textAlign(LEFT, TOP);
    textSize(12*uiScale);
    text("Nyquist: " + modeText, x+8*uiScale, y+6*uiScale);

    if (!hasData || reData == null || imData == null || reData.length < 2) {
      fill(170);
      text("Click 'Run Bode' to compute", x+8*uiScale, y+28*uiScale);
      popStyle();
      return;
    }

    float left = x + 46*uiScale;
    float right = x + w - 14*uiScale;
    float top = y + 34*uiScale;
    float bottom = y + h - 34*uiScale;

    // For open loop, always include the classical -1+j0 critical point.
    // For closed loop, scale only to T(jw) and the origin.
    float reMin = showClosedLoop ? 0.0 : -1.0;
    float reMax = 0.0;
    float imAbs = 0.0;

    for (int i=0; i<reData.length; i++){
      if (!Float.isFinite(reData[i]) || !Float.isFinite(imData[i])) continue;
      reMin = min(reMin, reData[i]);
      reMax = max(reMax, reData[i]);
      imAbs = max(imAbs, abs(imData[i]));
    }

    float reRange = max(1e-4, reMax - reMin);
    float rePad = max(0.18, 0.10*reRange);
    reMin -= rePad;
    reMax += rePad;
    imAbs = max(0.25, imAbs*1.12);

    // Keep approximately equal real/imaginary scale so loops do not look distorted.
    float pxW = max(1, right-left);
    float pxH = max(1, bottom-top);
    float desiredImRange = (reMax-reMin) * (pxH/pxW);
    float imRange = 2*imAbs;
    if (imRange < desiredImRange) imAbs = desiredImRange/2;
    else {
      float desiredReRange = imRange * (pxW/pxH);
      float extra = desiredReRange - (reMax-reMin);
      if (extra > 0) { reMin -= extra/2; reMax += extra/2; }
    }

    float imMin = -imAbs;
    float imMax = imAbs;

    // Background
    noStroke();
    fill(24,28,36);
    rect(left, top, right-left, bottom-top);

    // Grid with readable "nice" spacing
    float stepX = niceStep(reMax-reMin);
    float stepY = niceStep(imMax-imMin);

    stroke(120,130,150,55);
    strokeWeight(gridW*uiScale);

    float firstX = ceil(reMin/stepX)*stepX;
    for (float rv=firstX; rv<=reMax+0.5*stepX; rv+=stepX) {
      float xx = map(rv, reMin, reMax, left, right);
      line(xx, top, xx, bottom);
    }

    float firstY = ceil(imMin/stepY)*stepY;
    for (float iv=firstY; iv<=imMax+0.5*stepY; iv+=stepY) {
      float yy = map(iv, imMin, imMax, bottom, top);
      line(left, yy, right, yy);
    }

    // Real and imaginary axes
    stroke(175,185,205,135);
    strokeWeight(1.4*uiScale);
    if (0 >= reMin && 0 <= reMax) {
      float x0 = map(0, reMin, reMax, left, right);
      line(x0, top, x0, bottom);
    }
    if (0 >= imMin && 0 <= imMax) {
      float y0 = map(0, imMin, imMax, bottom, top);
      line(left, y0, right, y0);
    }

    // Tick labels
    fill(180);
    textSize(10.5*uiScale);
    textAlign(CENTER, TOP);
    for (float rv=firstX; rv<=reMax+0.5*stepX; rv+=stepX) {
      float xx = map(rv, reMin, reMax, left, right);
      text(nf(rv,0,(abs(stepX)<1)?1:0), xx, bottom+3*uiScale);
    }
    textAlign(RIGHT, CENTER);
    for (float iv=firstY; iv<=imMax+0.5*stepY; iv+=stepY) {
      if (abs(iv) < 0.25*stepY) continue;
      float yy = map(iv, imMin, imMax, bottom, top);
      text(nf(iv,0,(abs(stepY)<1)?1:0), left-5*uiScale, yy);
    }

    // Negative-frequency branch is the conjugate of the positive-frequency branch
    // for this real-coefficient system.
    noFill();
    stroke(160,170,190);
    strokeWeight(lineW*uiScale);
    beginShape();
    for (int i=reData.length-1; i>=0; i--) {
      if (!Float.isFinite(reData[i]) || !Float.isFinite(imData[i])) continue;
      vertex(map(reData[i], reMin, reMax, left, right),
             map(-imData[i], imMin, imMax, bottom, top));
    }
    endShape();

    // Positive-frequency branch
    stroke(120,200,255);
    strokeWeight(lineW*uiScale);
    beginShape();
    for (int i=0; i<reData.length; i++) {
      if (!Float.isFinite(reData[i]) || !Float.isFinite(imData[i])) continue;
      vertex(map(reData[i], reMin, reMax, left, right),
             map(imData[i], imMin, imMax, bottom, top));
    }
    endShape();

    // The -1+j0 critical point belongs to the classical open-loop Nyquist criterion.
    // Do not show it on the closed-loop T(jw) view, where it would be misleading.
    if (!showClosedLoop) {
      float critX = map(-1, reMin, reMax, left, right);
      float critY = map(0, imMin, imMax, bottom, top);
      stroke(255,150,110);
      strokeWeight(2*uiScale);
      line(critX-5*uiScale, critY-5*uiScale, critX+5*uiScale, critY+5*uiScale);
      line(critX-5*uiScale, critY+5*uiScale, critX+5*uiScale, critY-5*uiScale);
      noStroke();
      fill(255,180,140);
      textAlign(LEFT, BOTTOM);
      textSize(10.5*uiScale);
      text("-1 + j0", critX+6*uiScale, critY-3*uiScale);
    }

    // Start/end markers on the +w branch
    noStroke();
    fill(120,200,255);
    float sx = map(reData[0], reMin, reMax, left, right);
    float sy = map(imData[0], imMin, imMax, bottom, top);
    ellipse(sx, sy, 6*uiScale, 6*uiScale);
    float ex = map(reData[reData.length-1], reMin, reMax, left, right);
    float ey = map(imData[imData.length-1], imMin, imMax, bottom, top);
    ellipse(ex, ey, 6*uiScale, 6*uiScale);

    fill(190);
    textSize(10.5*uiScale);
    textAlign(CENTER, TOP);
    text("Re{" + symbol + "}", (left+right)/2, y+h-18*uiScale);
    textAlign(LEFT, TOP);
    text("Im{" + symbol + "}", x+5*uiScale, top);

    fill(175);
    textAlign(LEFT, BOTTOM);
    textSize(10*uiScale);
    text("+ω: blue  •  −ω: gray  •  θlin=" + nf(linAngleDeg,0,1) + "°",
         left, y+h-3*uiScale);

    popStyle();
  }
}

void runBode(){
  // Frequency range is logarithmic.  The linearization angle is a single
  // operating angle used to linearize the gravity term.
  Range fr = parseRange(fBodeFreq.text, 0.01, 100.0);
  float linAngleDeg = parseSafe(fBodeAngle.text, sim.targetDeg);

  int N = 240;
  float[] f = new float[N];
  float fmin = max(1e-3, fr.lo);
  float fmax = max(fmin*1.01, fr.hi);
  for (int i=0; i<N; i++){
    float t = i/(float)(N-1);
    f[i] = exp(log(fmin)*(1-t) + log(fmax)*t);
  }

  // Small-signal plant around theta0:
  //   I*d2(delta)/dt2 + b*d(delta)/dt + k_eff*delta = delta(tau)
  // where k_eff = springK - Kg*sin(theta0)
  float I = sim.inertia();
  float b = sim.drag;
  float th0 = radians(linAngleDeg);
  float k = sim.springK - sim.gravityK() * sin(th0);

  // PID(s) with the same first-order derivative filtering approximation
  // used by the original Bode implementation.
  float dt = 1.0/120.0;
  float alpha = clamp(pid.derivLPF, 1e-4, 0.999f);
  float Tf = dt/alpha;

  float[] magP = new float[N];
  float[] phaP = new float[N];
  float[] magC = new float[N];
  float[] phaC = new float[N];
  float[] reL = new float[N];
  float[] imL = new float[N];
  float[] reT = new float[N];
  float[] imT = new float[N];

  for (int i=0; i<N; i++){
    float w = TWO_PI * f[i];
    Complex s = new Complex(0, w);

    // Plant G(s) = 1 / (I*s^2 + b*s + k_eff)
    Complex denom = s.mul(s).mul(I).add(s.mul(b)).add(new Complex(k,0));
    Complex G = new Complex(1,0).div(denom);

    // Controller C(s) = Kp + Ki/s + Kd*s/(1 + Tf*s)
    Complex C = new Complex(pid.Kp,0)
      .add(new Complex(pid.Ki,0).div(s))
      .add(new Complex(pid.Kd,0).mul(s).div(new Complex(1,0).add(s.mul(Tf))));

    // Open-loop L(s) and closed-loop T(s)
    Complex L = C.mul(G);
    Complex T = L.div(new Complex(1,0).add(L));

    magP[i] = 20*log10f(max(1e-20, G.abs()));
    phaP[i] = degrees(G.arg());
    magC[i] = 20*log10f(max(1e-20, T.abs()));
    phaC[i] = degrees(T.arg());

    reL[i] = L.re;
    imL[i] = L.im;
    reT[i] = T.re;
    imT[i] = T.im;
  }

  bode.fHz = f;
  bode.magPlant = magP;
  bode.phaPlant = phaP;
  bode.magCL = magC;
  bode.phaCL = phaC;
  bode.linAngleDeg = linAngleDeg;
  bode.hasData = true;

  nyquist.fHz = f;
  nyquist.reOpen = reL;
  nyquist.imOpen = imL;
  nyquist.reClosed = reT;
  nyquist.imClosed = imT;
  nyquist.linAngleDeg = linAngleDeg;
  nyquist.hasData = true;
}

class Range { float lo, hi; }
Range parseRange(String s, float defLo, float defHi){ Range r=new Range(); try{ String t=s.replace(" ",""); String[] parts = splitTokens(t, "-–,"); if (parts.length>=2){ r.lo = max(0.0, Float.parseFloat(parts[0])); r.hi = max(r.lo+1e-6, Float.parseFloat(parts[1])); } else if (parts.length==1){ r.lo = defLo; r.hi = max(defLo+1e-6, Float.parseFloat(parts[0])); } else { r.lo=defLo; r.hi=defHi; } } catch(Exception e){ r.lo=defLo; r.hi=defHi; } return r; }

float mapLog(float v, float lo, float hi, float a, float b){ return map(log(v), log(lo), log(hi), a, b); }

// Processing has only natural log; helper for base-10
float log10f(float v){ return log(v) / log(10); }

// ---------- Minimal UI (IN‑BOX LABELS) with safer text drawing ----------
class InputField { 
  String label, text; float x,y,w,h; boolean active=false; 
  InputField(String label, float x, float y, float w, float h, String initial){ this.label=label; setPos(x,y,w,h); this.text=initial; } 
  void setPos(float X,float Y,float W,float H){ x=X; y=Y; w=W; h=H; } 
  void draw(){ 
    stroke(active ? color(120,200,255) : color(150));
    fill(30,35,45);
    rect(x, y, w, h, 6*uiScale);

    // Label chip.  Cap its width so every field always keeps a readable value area.
    float labelSize = 11.5*uiScale;
    textSize(labelSize);
    textAlign(LEFT, CENTER);
    float pad = 6*uiScale;
    float minValueW = 76*uiScale;
    float maxChipW = max(70*uiScale, w - minValueW);
    float chipW = min(maxChipW, max(78*uiScale, textWidth(label) + 2*pad));
    String labelDraw = ellipsize(label, chipW - 2*pad, labelSize);

    noStroke();
    fill(42,48,62);
    rect(x, y, chipW, h, 6*uiScale, 0, 0, 6*uiScale);
    stroke(70,80,98);
    line(x+chipW, y, x+chipW, y+h);

    fill(205);
    text(labelDraw, x + pad, y + h/2);

    // Value text uses ellipsis only; no nested clip()/noClip().
    // That preserves the outer panel clip and prevents bottom-of-screen overlap.
    float tStart = x + chipW + 8*uiScale;
    float availW = max(18*uiScale, w - (tStart - x) - 8*uiScale);
    String toDraw = ellipsize(text, availW, 12*uiScale);
    fill(240);
    textAlign(LEFT, CENTER);
    textSize(12*uiScale);
    text(toDraw, tStart, y + h/2);

    // Caret
    if (active && (frameCount/30)%2==0){
      float twv = textWidth(toDraw);
      stroke(240);
      float cx = tStart + min(twv, availW-3*uiScale);
      line(cx+2, y+7*uiScale, cx+2, y+h-7*uiScale);
    }
  } 
  void onMouse(float mx,float my){ active=(mx>=x && mx<=x+w && my>=y && my<=y+h); } 
  void onKeyTyped(char k,int _kc){ if(!active) return; if ((k>='0'&&k<='9')||k=='-'||k=='+'||k=='.'||k=='e'||k=='E') text+=k; } 
}

// Trim a string to fit width and add ellipsis if needed
String ellipsize(String s, float maxW, float sz){ textSize(sz); if (textWidth(s) <= maxW) return s; String ell = "…"; float ellW = textWidth(ell); String r = s; while (r.length()>0 && textWidth(r) + ellW > maxW) { r = r.substring(0, r.length()-1); } return r + ell; }

class Button { String label; float x,y,w,h; Button(String label,float x,float y,float w,float h){ this.label=label; setPos(x,y,w,h);} void setPos(float X,float Y,float W,float H){ x=X; y=Y; w=W; h=H; } void draw(){ boolean hover=(mouseX>=x&&mouseX<=x+w&&mouseY>=y&&mouseY<=y+h); stroke(hover?color(120,200,255):color(160)); fill(hover?color(40,50,70):color(30,35,45)); rect(x,y,w,h,6*uiScale); fill(230); textAlign(CENTER,CENTER); textSize(12*uiScale); text(label,x+w/2,y+h/2);} boolean hit(float mx,float my){ return (mx>=x&&mx<=x+w&&my>=y&&my<=y+h); } }

InputField mkField(String label, float x, float y, float w, float h, String v){ InputField f=new InputField(label,x,y,w,h,v); fields.add(f); return f; }

// ---------- Tiny complex helper ----------
class Complex { float re, im; Complex(float re,float im){ this.re=re; this.im=im; } Complex add(Complex o){ return new Complex(re+o.re, im+o.im);} Complex sub(Complex o){ return new Complex(re-o.re, im-o.im);} Complex mul(float s){ return new Complex(re*s, im*s);} Complex mul(Complex o){ return new Complex(re*o.re - im*o.im, re*o.im + im*o.re);} Complex div(Complex o){ float d=o.re*o.re + o.im*o.im; return new Complex((re*o.re + im*o.im)/d, (im*o.re - re*o.im)/d);} float abs(){ return sqrt(re*re + im*im);} float arg(){ return atan2(im, re);} }
