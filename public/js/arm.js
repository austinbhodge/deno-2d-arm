window.Arm = (() => {
  const state = {
    baseX: 0,
    baseY: 0,
    L1: 120,
    L2: 100,
    theta1: Math.PI / 4,
    theta2: -Math.PI / 6,
    mode: "ik",
    elbowUp: true,
    showTrail: true,
    trail: [],
    maxTrail: 80,
    targetX: 0,
    targetY: 0,
    canvasW: 0,
    canvasH: 0,
  };

  function init(canvasW, canvasH) {
    state.baseX = canvasW / 2;
    state.baseY = canvasH / 2;
    state.canvasW = canvasW;
    state.canvasH = canvasH;
    state.trail = [];
  }

  /** Convert screen coords (y-down) to math coords (y-up) relative to base */
  function screenToMath(sx, sy) {
    return { x: sx - state.baseX, y: -(sy - state.baseY) };
  }

  /** Convert math coords (y-up) to screen coords (y-down) relative to base */
  function mathToScreen(mx, my) {
    return { x: state.baseX + mx, y: state.baseY - my };
  }

  function update(mx, my, mouseOverUI) {
    if (state.mode === "ik" && !mouseOverUI) {
      const m = screenToMath(mx, my);
      state.targetX = m.x;
      state.targetY = m.y;
      const result = Kinematics.inverse(m.x, m.y, state.L1, state.L2, state.elbowUp);
      if (result) {
        state.theta1 = result.theta1;
        state.theta2 = result.theta2;
      }
    }

    // Compute FK for trail
    const fk = Kinematics.forward(state.theta1, state.theta2, state.L1, state.L2);
    const eeScreen = mathToScreen(fk.ee.x, fk.ee.y);

    if (state.showTrail) {
      state.trail.push({ x: eeScreen.x, y: eeScreen.y });
      if (state.trail.length > state.maxTrail) {
        state.trail.shift();
      }
    }
  }

  function render() {
    const fk = Kinematics.forward(state.theta1, state.theta2, state.L1, state.L2);
    const elb = mathToScreen(fk.elbow.x, fk.elbow.y);
    const ee = mathToScreen(fk.ee.x, fk.ee.y);
    const base = { x: state.baseX, y: state.baseY };

    drawGrid();
    if (state.showTrail && state.trail.length > 1) drawTrail();
    if (state.mode === "ik") drawTarget();

    // Link 1
    stroke(100, 200, 255);
    strokeWeight(6);
    line(base.x, base.y, elb.x, elb.y);

    // Link 2
    stroke(150, 230, 180);
    strokeWeight(5);
    line(elb.x, elb.y, ee.x, ee.y);

    drawAngleArcs(base, elb);

    // Joints
    noStroke();
    fill(200, 200, 220);
    circle(base.x, base.y, 16);
    fill(180, 200, 220);
    circle(elb.x, elb.y, 12);
    fill(255, 200, 100);
    circle(ee.x, ee.y, 10);

    drawAnnotations(base, elb, ee);
  }

  function drawGrid() {
    const sp = 50;
    stroke(255, 255, 255, 15);
    strokeWeight(1);
    for (let x = state.baseX % sp; x < state.canvasW; x += sp) {
      line(x, 0, x, state.canvasH);
    }
    for (let y = state.baseY % sp; y < state.canvasH; y += sp) {
      line(0, y, state.canvasW, y);
    }
    stroke(255, 255, 255, 35);
    line(0, state.baseY, state.canvasW, state.baseY);
    line(state.baseX, 0, state.baseX, state.canvasH);
  }

  function drawTrail() {
    noFill();
    for (let i = 1; i < state.trail.length; i++) {
      const a = map(i, 0, state.trail.length, 0, 120);
      stroke(255, 200, 100, a);
      strokeWeight(2);
      line(state.trail[i - 1].x, state.trail[i - 1].y, state.trail[i].x, state.trail[i].y);
    }
  }

  function drawTarget() {
    const t = mathToScreen(state.targetX, state.targetY);
    stroke(255, 100, 100, 120);
    strokeWeight(1);
    const s = 12;
    line(t.x - s, t.y, t.x + s, t.y);
    line(t.x, t.y - s, t.x, t.y + s);
    noFill();
    circle(t.x, t.y, 20);
  }

  function drawAngleArcs(base, elb) {
    noFill();
    strokeWeight(2);

    // Theta1 arc at base
    if (Math.abs(state.theta1) > 0.05) {
      stroke(100, 200, 255, 80);
      const a = -state.theta1;
      arc(base.x, base.y, 60, 60, Math.min(0, a), Math.max(0, a));
    }

    // Theta2 arc at elbow
    if (Math.abs(state.theta2) > 0.05) {
      stroke(150, 230, 180, 80);
      const link1Angle = -state.theta1;
      const link2Angle = -(state.theta1 + state.theta2);
      arc(elb.x, elb.y, 48, 48, Math.min(link1Angle, link2Angle), Math.max(link1Angle, link2Angle));
    }
  }

  function drawAnnotations(base, elb, ee) {
    const deg1 = (state.theta1 * 180 / Math.PI).toFixed(1);
    const deg2 = (state.theta2 * 180 / Math.PI).toFixed(1);
    fill(200, 200, 220, 180);
    noStroke();
    textSize(11);
    textAlign(LEFT, CENTER);
    text(`\u03B81: ${deg1}\u00B0`, base.x + 22, base.y - 18);
    text(`\u03B82: ${deg2}\u00B0`, elb.x + 16, elb.y - 14);

    const fk = Kinematics.forward(state.theta1, state.theta2, state.L1, state.L2);
    fill(255, 200, 100, 180);
    textSize(10);
    text(`(${fk.ee.x.toFixed(0)}, ${fk.ee.y.toFixed(0)})`, ee.x + 10, ee.y - 10);
  }

  function clearTrail() {
    state.trail = [];
  }

  return { state, init, update, render, clearTrail, screenToMath, mathToScreen };
})();
