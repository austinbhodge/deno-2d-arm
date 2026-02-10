window.UI = (() => {
  let mouseOverUI = false;
  let katexReady = false;
  let katexRendered = false;

  const els = {};

  function init() {
    els.btnInfo = document.getElementById("btn-info");
    els.btnSettings = document.getElementById("btn-settings");
    els.infoPanel = document.getElementById("info-panel");
    els.settingsPanel = document.getElementById("settings-panel");
    els.modeSelect = document.getElementById("mode-select");
    els.sliderL1 = document.getElementById("slider-l1");
    els.sliderL2 = document.getElementById("slider-l2");
    els.sliderTheta1 = document.getElementById("slider-theta1");
    els.sliderTheta2 = document.getElementById("slider-theta2");
    els.valL1 = document.getElementById("val-l1");
    els.valL2 = document.getElementById("val-l2");
    els.valTheta1 = document.getElementById("val-theta1");
    els.valTheta2 = document.getElementById("val-theta2");
    els.chkElbow = document.getElementById("chk-elbow");
    els.chkTrail = document.getElementById("chk-trail");
    els.currentValues = document.getElementById("current-values");
    els.fkEquations = document.getElementById("fk-equations");
    els.ikEquations = document.getElementById("ik-equations");

    // Panel toggle buttons
    els.btnInfo.addEventListener("click", () => togglePanel("info"));
    els.btnSettings.addEventListener("click", () => togglePanel("settings"));

    // Mouse over UI detection
    const uiEls = [els.infoPanel, els.settingsPanel,
      document.getElementById("button-cluster")];
    for (const el of uiEls) {
      el.addEventListener("mouseenter", () => { mouseOverUI = true; });
      el.addEventListener("mouseleave", () => { mouseOverUI = false; });
    }

    // Mode selector
    els.modeSelect.addEventListener("change", () => {
      Arm.state.mode = els.modeSelect.value;
      updateSliderStates();
      Arm.clearTrail();
    });

    // Link length sliders
    els.sliderL1.addEventListener("input", () => {
      Arm.state.L1 = parseInt(els.sliderL1.value);
      els.valL1.textContent = els.sliderL1.value;
      Arm.clearTrail();
    });
    els.sliderL2.addEventListener("input", () => {
      Arm.state.L2 = parseInt(els.sliderL2.value);
      els.valL2.textContent = els.sliderL2.value;
      Arm.clearTrail();
    });

    // Angle sliders (only active in FK mode)
    els.sliderTheta1.addEventListener("input", () => {
      if (Arm.state.mode === "fk") {
        Arm.state.theta1 = parseInt(els.sliderTheta1.value) * Math.PI / 180;
      }
    });
    els.sliderTheta2.addEventListener("input", () => {
      if (Arm.state.mode === "fk") {
        Arm.state.theta2 = parseInt(els.sliderTheta2.value) * Math.PI / 180;
      }
    });

    // Checkboxes
    els.chkElbow.addEventListener("change", () => {
      Arm.state.elbowUp = els.chkElbow.checked;
    });
    els.chkTrail.addEventListener("change", () => {
      Arm.state.showTrail = els.chkTrail.checked;
      if (!Arm.state.showTrail) Arm.clearTrail();
    });

    updateSliderStates();
    pollKaTeX();
  }

  let activePanel = null;

  function togglePanel(name) {
    if (activePanel === name) {
      // Close the active panel
      els.infoPanel.classList.add("hidden");
      els.settingsPanel.classList.add("hidden");
      els.btnInfo.classList.remove("active");
      els.btnSettings.classList.remove("active");
      activePanel = null;
      return;
    }

    // Close all, then open requested
    els.infoPanel.classList.add("hidden");
    els.settingsPanel.classList.add("hidden");
    els.btnInfo.classList.remove("active");
    els.btnSettings.classList.remove("active");

    if (name === "info") {
      els.infoPanel.classList.remove("hidden");
      els.btnInfo.classList.add("active");
      renderKaTeX();
    } else {
      els.settingsPanel.classList.remove("hidden");
      els.btnSettings.classList.add("active");
    }
    activePanel = name;
  }

  function updateSliderStates() {
    const isFk = Arm.state.mode === "fk";
    els.sliderTheta1.disabled = !isFk;
    els.sliderTheta2.disabled = !isFk;
  }

  function syncSliders() {
    const deg1 = Math.round(Arm.state.theta1 * 180 / Math.PI);
    const deg2 = Math.round(Arm.state.theta2 * 180 / Math.PI);
    els.valTheta1.textContent = deg1;
    els.valTheta2.textContent = deg2;

    if (Arm.state.mode === "ik") {
      els.sliderTheta1.value = deg1;
      els.sliderTheta2.value = deg2;
    }

    els.valL1.textContent = Arm.state.L1;
    els.valL2.textContent = Arm.state.L2;
  }

  function updateCurrentValues() {
    if (activePanel !== "info") return;
    const fk = Kinematics.forward(Arm.state.theta1, Arm.state.theta2, Arm.state.L1, Arm.state.L2);
    const deg1 = (Arm.state.theta1 * 180 / Math.PI).toFixed(1);
    const deg2 = (Arm.state.theta2 * 180 / Math.PI).toFixed(1);
    els.currentValues.innerHTML =
      `θ1 = ${deg1}°<br>` +
      `θ2 = ${deg2}°<br>` +
      `L1 = ${Arm.state.L1}<br>` +
      `L2 = ${Arm.state.L2}<br>` +
      `End-effector = (${fk.ee.x.toFixed(1)}, ${fk.ee.y.toFixed(1)})`;
  }

  function pollKaTeX() {
    if (typeof katex !== "undefined") {
      katexReady = true;
    } else {
      setTimeout(pollKaTeX, 200);
    }
  }

  function renderKaTeX() {
    if (!katexReady || katexRendered) return;
    katexRendered = true;

    const fkEqs = [
      "x_{elbow} = L_1 \\cos(\\theta_1)",
      "y_{elbow} = L_1 \\sin(\\theta_1)",
      "x_{ee} = L_1 \\cos(\\theta_1) + L_2 \\cos(\\theta_1 + \\theta_2)",
      "y_{ee} = L_1 \\sin(\\theta_1) + L_2 \\sin(\\theta_1 + \\theta_2)",
    ];

    const ikEqs = [
      "\\cos(\\theta_2) = \\frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}",
      "\\theta_2 = \\pm \\arccos(\\cos(\\theta_2))",
      "\\theta_1 = \\text{atan2}(y, x) - \\text{atan2}(k_2, k_1)",
      "k_1 = L_1 + L_2 \\cos(\\theta_2),\\quad k_2 = L_2 \\sin(\\theta_2)",
    ];

    els.fkEquations.innerHTML = "";
    for (const eq of fkEqs) {
      const div = document.createElement("div");
      div.className = "eq-block";
      katex.render(eq, div, { displayMode: true, throwOnError: false });
      els.fkEquations.appendChild(div);
    }

    els.ikEquations.innerHTML = "";
    for (const eq of ikEqs) {
      const div = document.createElement("div");
      div.className = "eq-block";
      katex.render(eq, div, { displayMode: true, throwOnError: false });
      els.ikEquations.appendChild(div);
    }
  }

  function isMouseOverUI() {
    return mouseOverUI;
  }

  return { init, syncSliders, updateCurrentValues, isMouseOverUI };
})();
