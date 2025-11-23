/**
 * PID Robot Arm Simulator — Bode Analysis v1.2 (clean)
 *
 * By: Mohammad Elzein 08/15/2025
 * Changes
 *  - Spring is tied to TARGET angle: τspring = −k(θ − θ_target)
 *  - New input: Arm Length (m); payload distance clamped ≤ L
 *  - Left panel auto‑compacts and supports mouse‑wheel scrolling
 *  - Time plot: adaptive Y range over last 10 s with nice ticks
 *  - Bode plot: cleaned background, Y‑axis labels for dB/deg
 */

import processing.event.MouseEvent;

// ---------- Window / fullscreen (no settings()) ----------
boolean fillScreenToggle = false; // runtime fill‑screen (resizable window)
boolean fLatch = false;           // debounce for F key
final int WINDOW_W = 1200, WINDOW_H = 800;

// ---------- Globals ----------
ArmSim sim; PID pid; Plot plot; HUD hud; BodePlot bode;
ArrayList<InputField> fields = new ArrayList<InputField>();
Button btnReset, btnZeroI, btnDisturb;
Button btnStepUp, btnStepDown, btnSineToggle, btnRunBode;
InputField fKp, fKi, fKd, fTarget, fArmLen, fMass, fPayload, fPayloadDist, fDrag, fSpring, fTlim;
InputField fAmp, fFreq, fCenter;
InputField fBodeFreq, fBodeAngle;

float pxScale;                 // meters -> pixels for arm length
int lastW = -1, lastH = -1;    // detect window size change
float panelX, panelY, panelW;  // layout
float armCX, armCY;            // arm center
float uiScale = 1;             // global UI scale factor

// panel scrolling support
float panelScroll = 0;   // 0 = top; negative scrolls upward
float panelScrollMin = 0; // most negative allowed (computed in layout)

// PID term visualization
float pTerm=0, iTerm=0, dTermOut=0, uCmd=0, uSat=0;

// Setpoint generator state
boolean sineOn = false;
float ampDeg = 10.0;   // sine amplitude
float freqHz = 0.5;    // sine frequency
float centerDeg;       // sine center
float simTime = 0;     // seconds

void setup() {
  surface.setResizable(true);
  surface.setSize(WINDOW_W, WINDOW_H);
  centerWindow(WINDOW_W, WINDOW_H);
  smooth(8);

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
  fBodeAngle = mkField("Bode Angle (deg a-b)",0,0,140,38, "1-20");

  // Buttons
  btnStepUp     = new Button("Step +10°",     0,0, 160,40);
  btnStepDown   = new Button("Step −10°",     0,0, 160,40);
  btnSineToggle = new Button("Sine: OFF",     0,0, 160,40);
  btnRunBode    = new Button("Run Bode",      0,0, 160,40);
  btnReset      = new Button("Reset",         0,0, 160,40);
  btnZeroI      = new Button("Zero Integral", 0,0, 160,40);
  btnDisturb    = new Button("Disturb (+Nm)", 0,0, 160,40);

  // Plots
  plot = new Plot(0,0, 700,260, 2400); plot.windowSeconds = 10; // rolling 10s window
  bode = new BodePlot();
  hud = new HUD();

  layoutUI();
}

void draw() {
  if (width != lastW || height != lastH) { layoutUI(); lastW = width; lastH = height; }
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
  hud.draw(plot);
  drawFooter();
}

// ---------- Fullscreen / Window helpers ----------
void centerWindow(int w, int h) { surface.setLocation((displayWidth - w)/2, (displayHeight - h)/2); }
void applyWindowMode() { if (fillScreenToggle) { surface.setSize(displayWidth, displayHeight); surface.setLocation(0, 0); cursor(); } else { surface.setSize(WINDOW_W, WINDOW_H); centerWindow(WINDOW_W, WINDOW_H); } layoutUI(); }

// ---------- Layout + Scroll ----------
void layoutUI() {
  // allow smaller UI when the window is short
  uiScale = constrain(min(width, height) / 900.0, 0.70, 1.4);
  panelX = 26 * uiScale; panelY = 30 * uiScale; panelW = constrain(width * 0.24, 280 * uiScale, 430 * uiScale);

  float w = panelW - 52*uiScale, h = 40*uiScale, s = 56*uiScale; // one row per control
  int items = 23; // fields + buttons including bode (+1 for Arm Length)

  float bottomMargin = 40*uiScale; 
  float needed = (items-1)*s + h; 
  float avail = height - bottomMargin - panelY; 
  // compact spacing if needed
  if (needed > avail) { float factor = max(0.45, avail/needed); s *= factor; h *= max(0.80, factor); }

  // compute scroll bounds
  needed = (items-1)*s + h; // recompute after compaction
  panelScrollMin = min(0, avail - needed);
  panelScroll = constrain(panelScroll, panelScrollMin, 0);

  float x = panelX, y0 = panelY + panelScroll; float y = y0, colX = x;

  // Fields
  fKp.setPos(colX, y, w, h); y += s; fKi.setPos(colX, y, w, h); y += s; fKd.setPos(colX, y, w, h); y += s + 8*uiScale;
  fTarget.setPos(colX, y, w, h); y += s + 8*uiScale;
  fArmLen.setPos(colX, y, w, h); y += s;
  fMass.setPos(colX, y, w, h); y += s; fPayload.setPos(colX, y, w, h); y += s; fPayloadDist.setPos(colX, y, w, h); y += s; fDrag.setPos(colX, y, w, h); y += s; fSpring.setPos(colX, y, w, h); y += s; fTlim.setPos(colX, y, w, h); y += s + 8*uiScale;

  // Setpoint
  fAmp.setPos(colX, y, w, h); y += s; fFreq.setPos(colX, y, w, h); y += s; fCenter.setPos(colX, y, w, h); y += s + 8*uiScale;

  // Bode ranges
  fBodeFreq.setPos(colX, y, w, h); y += s; fBodeAngle.setPos(colX, y, w, h); y += s + 8*uiScale;

  // Buttons
  btnStepUp.setPos(colX, y, w, h); y += s; btnStepDown.setPos(colX, y, w, h); y += s; btnSineToggle.setPos(colX, y, w, h); y += s + 8*uiScale;
  btnRunBode.setPos(colX, y, w, h); y += s; btnReset.setPos(colX, y, w, h); y += s; btnZeroI.setPos(colX, y, w, h); y += s; btnDisturb.setPos(colX, y, w, h); y += s;

  // Plots on right: time plot on top, bode at bottom
  float rightX = panelX + panelW + 26*uiScale; float rightW = max(560*uiScale, width - rightX - 26*uiScale);
  float topH = max(240*uiScale, height * 0.32); plot.setRect(rightX, 18*uiScale, rightW, topH); plot.lineWeight = 2.4 * uiScale; plot.gridWeight = 1.2 * uiScale; plot.textSize = 12 * uiScale; plot.labelPadding = 8 * uiScale;

  float bodeH = max(220*uiScale, height * 0.28); bode.setRect(rightX, 22*uiScale + topH, rightW, bodeH);

  // HUD near top plot
  hud.x = rightX + rightW - 310*uiScale; hud.y = plot.y + 10*uiScale; hud.w = 300*uiScale; hud.h = 170*uiScale; hud.textSize = 12*uiScale;

  // Arm area center lower-right
  armCX = rightX + rightW * 0.52; armCY = max(plot.y + topH + bodeH*0.35, height * 0.70); pxScale = min(width, height) * 0.46;
}

void mouseWheel(MouseEvent evt){
  // scroll only when pointer is on the left panel region
  if (mouseX >= panelX && mouseX <= panelX + panelW) {
    float e = evt.getCount();
    panelScroll = constrain(panelScroll - e * 22, panelScrollMin, 0);
  }
}

// ---------- Input ----------
void mousePressed() {
  for (InputField f : fields) f.onMouse(mouseX, mouseY);
  if (btnReset.hit(mouseX, mouseY)) resetSim();
  if (btnZeroI.hit(mouseX, mouseY)) pid.integral = 0;
  if (btnDisturb.hit(mouseX, mouseY)) sim.impulse(1.5);

  if (btnStepUp.hit(mouseX, mouseY)) { if (sineOn) { centerDeg += 10; fCenter.text = nf(centerDeg,0,2); } else { sim.targetDeg += 10; fTarget.text = nf(sim.targetDeg,0,2); } }
  if (btnStepDown.hit(mouseX, mouseY)) { if (sineOn) { centerDeg -= 10; fCenter.text = nf(centerDeg,0,2); } else { sim.targetDeg -= 10; fTarget.text = nf(sim.targetDeg,0,2); } }
  if (btnSineToggle.hit(mouseX, mouseY)) { sineOn = !sineOn; btnSineToggle.label = sineOn ? "Sine: ON" : "Sine: OFF"; if (sineOn) { centerDeg = parseSafe(fCenter.text, centerDeg); } }
  if (btnRunBode.hit(mouseX, mouseY)) runBode();
}

void keyTyped() { for (InputField f : fields) if (f.active) { f.onKeyTyped(key, keyCode); if (key == ENTER || key == RETURN) f.active = false; } }
void keyPressed() {
  if (key == ESC) { key = 0; return; } if (key == 'q' || key == 'Q') exit();
  if ((key == 'f' || key == 'F') && !fLatch) { fLatch = true; fillScreenToggle = !fillScreenToggle; applyWindowMode(); return; }
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
  // panel clip to keep text inside when scrolled
  clip((int)panelX-2, (int)panelY-2, (int)panelW+4, (int)(height - panelY - 20));
  for (InputField f : fields) f.draw();
  btnStepUp.draw(); btnStepDown.draw(); btnSineToggle.draw(); btnRunBode.draw();
  btnReset.draw(); btnZeroI.draw(); btnDisturb.draw();
  noClip();

  fill(255); textAlign(LEFT, TOP); textSize(12*uiScale); text("Plot: angle vs time (last 10 s)", plot.x, plot.y - 18*uiScale);
  // Legend tucked **inside** the plot, top-right (never off-screen)
float legW = 170*uiScale;
float legX = plot.x + plot.w - legW - 12*uiScale;
float legY = plot.y + 12*uiScale;
strokeWeight(4*uiScale); stroke(120,200,255); line(legX, legY, legX+36*uiScale, legY); noStroke(); fill(200); textAlign(LEFT, CENTER); text("current", legX+44*uiScale, legY);
strokeWeight(4*uiScale); stroke(255,120,160); line(legX, legY + 12*uiScale, legX+36*uiScale, legY + 12*uiScale); noStroke(); fill(200); text("target", legX+44*uiScale, legY + 12*uiScale);

  // hint for scrolling (only when everything fits)
  if (panelScrollMin == 0) {
    fill(170); textSize(11*uiScale); text("Mouse‑wheel over left panel to scroll", panelX, panelY - 14*uiScale);
  }
}

void drawFooter() { fill(200); textAlign(LEFT, BOTTOM); textSize(12*uiScale); text("F: toggle fill‑screen  •  Q: quit  •  Click a box → type → Enter", plot.x, height-12*uiScale); }

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
  float x, y, w, h; float textSize=12; float lineW=2.2; float gridW=1.1; boolean hasData=false; float[] fHz, magPlant, phaPlant, magCL, phaCL; // arrays
  void setRect(float X,float Y,float W,float H){ x=X; y=Y; w=W; h=H; }
  void draw(){
    pushStyle();
    // full background to avoid residual banding
    noStroke(); fill(18,20,26); rect(x, y, w, h);
    // Frame & title
    noFill(); stroke(220,230,240,160); strokeWeight(1.2*uiScale); rect(x, y, w, h, 6*uiScale);
    fill(200); textAlign(LEFT, TOP); textSize(12*uiScale); text("Bode: mag (dB) & phase (deg)", x+8*uiScale, y+6*uiScale);
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

void runBode(){
  // Parse ranges
  Range fr = parseRange(fBodeFreq.text, 0.01, 100.0); Range ar = parseRange(fBodeAngle.text, 0.5, 30.0);
  int N = 200; float[] f = new float[N]; // log-spaced
  float fmin=max(1e-3, fr.lo), fmax=max(fmin*1.01, fr.hi); for (int i=0;i<N;i++){ float t=i/(float)(N-1); f[i]=exp(log(fmin)*(1-t) + log(fmax)*t); }

  // Plant params
  float I=sim.inertia(); float b=sim.drag; float th0 = radians(sim.targetDeg);
  // Small-signal stiffness about current target: spring − d(τ_g)/dθ = spring − K_g sin(θ0)
  float k = sim.springK - sim.gravityK() * sin(th0);
  // PID params in s-domain; derivative LPF time constant from discrete alpha
  float dt=1.0/120.0; float alpha = clamp(pid.derivLPF, 1e-4, 0.999f); float Tf = dt/alpha; // approx RC

  // Arrays
  float[] magP=new float[N], phaP=new float[N], magC=new float[N], phaC=new float[N];

  for (int i=0;i<N;i++){
    float w = TWO_PI * f[i]; Complex s = new Complex(0, w);
    // G(s) = 1 / (I s^2 + b s + k)
    Complex denom = s.mul(s).mul(I).add(s.mul(b)).add(new Complex(k,0)); Complex G = new Complex(1,0).div(denom);
    // PID(s)
    Complex PID = new Complex(pid.Kp,0).add(new Complex(pid.Ki,0).div(s)).add(new Complex(pid.Kd,0).mul(s).div(new Complex(1,0).add(s.mul(Tf))));
    // Closed-loop T(s) = (PID*G) / (1 + PID*G)
    Complex L = PID.mul(G); Complex T = L.div(new Complex(1,0).add(L));
    // Record mag/phase
    magP[i] = 20*log10f(G.abs()); phaP[i] = degrees(G.arg());
    magC[i] = 20*log10f(T.abs()); phaC[i] = degrees(T.arg());
  }

  bode.fHz=f; bode.magPlant=magP; bode.phaPlant=phaP; bode.magCL=magC; bode.phaCL=phaC; bode.hasData=true;
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
    stroke(active ? color(120,200,255) : color(150)); fill(30,35,45); rect(x, y, w, h, 6*uiScale);
    // Label chip
    float labelSize = 12*uiScale*0.9; textSize(labelSize); textAlign(LEFT, CENTER); float pad = 6*uiScale; 
    float chipW = max(62*uiScale, textWidth(label) + 2*pad);
    noStroke(); fill(42, 48, 62); rect(x, y, chipW, h, 6*uiScale, 0, 0, 6*uiScale); 
    stroke(70,80,98); line(x+chipW, y, x+chipW, y+h);
    fill(205); text(label, x + pad, y + h/2);

    // Value text area (clipped + ellipsis if too long)
    float tStart = x + chipW + 8*uiScale; float availW = w - (tStart - x) - 8*uiScale; 
    String toDraw = ellipsize(text, availW, 12*uiScale);
    fill(240); textAlign(LEFT, CENTER); textSize(12*uiScale);
    clip((int)(tStart), (int)(y+1), (int)(availW), (int)(h-2));
    text(toDraw, tStart, y + h/2);
    noClip();

    // Caret
    if (active && (frameCount/30)%2==0){ float twv=textWidth(toDraw); stroke(240); float cx = tStart + min(twv, availW-2); line(cx+2, y+7*uiScale, cx+2, y+h-7*uiScale); }
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
