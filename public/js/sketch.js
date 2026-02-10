function setup() {
  createCanvas(windowWidth, windowHeight);
  Arm.init(width, height);
  UI.init();
}

function draw() {
  background(26, 26, 46);
  Arm.update(mouseX, mouseY, UI.isMouseOverUI());
  Arm.render();
  UI.syncSliders();
  UI.updateCurrentValues();
}

function windowResized() {
  resizeCanvas(windowWidth, windowHeight);
  Arm.init(width, height);
}
