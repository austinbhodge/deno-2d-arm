window.Kinematics = (() => {
  /**
   * Forward kinematics for a 2-link arm.
   * Returns elbow and end-effector positions in math coords (y-up).
   */
  function forward(theta1, theta2, L1, L2) {
    const elbowX = L1 * Math.cos(theta1);
    const elbowY = L1 * Math.sin(theta1);
    const eeX = elbowX + L2 * Math.cos(theta1 + theta2);
    const eeY = elbowY + L2 * Math.sin(theta1 + theta2);
    return {
      elbow: { x: elbowX, y: elbowY },
      ee: { x: eeX, y: eeY },
    };
  }

  /**
   * Inverse kinematics for a 2-link arm (analytical).
   * target: {x, y} in math coords (y-up)
   * Returns {theta1, theta2} in radians, or null if no solution.
   * elbowUp selects between the two valid solutions.
   * Clamps to reachable workspace when target is out of range.
   */
  function inverse(targetX, targetY, L1, L2, elbowUp) {
    let dist = Math.sqrt(targetX * targetX + targetY * targetY);
    const maxReach = L1 + L2;
    const minReach = Math.abs(L1 - L2);

    // Clamp to reachable workspace
    if (dist > maxReach) {
      const scale = maxReach / dist;
      targetX *= scale;
      targetY *= scale;
      dist = maxReach;
    }
    if (dist < minReach) {
      const scale = minReach / dist;
      targetX *= scale;
      targetY *= scale;
      dist = minReach;
    }

    // Law of cosines for theta2
    const cosTheta2 = (dist * dist - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    const clampedCos = Math.max(-1, Math.min(1, cosTheta2));
    const theta2 = elbowUp ? -Math.acos(clampedCos) : Math.acos(clampedCos);

    // Geometric solution for theta1
    const k1 = L1 + L2 * Math.cos(theta2);
    const k2 = L2 * Math.sin(theta2);
    const theta1 = Math.atan2(targetY, targetX) - Math.atan2(k2, k1);

    return { theta1, theta2 };
  }

  return { forward, inverse };
})();
