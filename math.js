/* 坐标约定：x 向右、y 向上，角度逆时针为正；q2 相对于第一根连杆。 */
(function (root) {
  "use strict";
  const TAU = 2 * Math.PI;
  const clamp = (x, low, high) => Math.max(low, Math.min(high, x));
  const wrap = a => ((a + Math.PI) % TAU + TAU) % TAU - Math.PI;
  const nearest = (angle, reference) => reference + wrap(angle - reference);
  const deg = radians => radians * 180 / Math.PI;
  const rad = degrees => degrees * Math.PI / 180;

  function fk(q1, q2, l1, l2) {
    const elbow = { x: l1 * Math.cos(q1), y: l1 * Math.sin(q1) };
    return { elbow, x: elbow.x + l2 * Math.cos(q1 + q2),
      y: elbow.y + l2 * Math.sin(q1 + q2) };
  }

  function ik(x, y, l1, l2, branch = "up", reference = [0, 0]) {
    if (![x, y, l1, l2, ...reference].every(Number.isFinite) || l1 <= 0 || l2 <= 0) {
      return { reachable: false, reason: "坐标必须有限，连杆长度必须大于零。" };
    }
    const r = Math.hypot(x, y), outer = l1 + l2, inner = Math.abs(l1 - l2);
    const tolerance = 1e-10 * Math.max(1, outer);
    if (r > outer + tolerance || r < inner - tolerance) {
      return { reachable: false, reason: r > outer ? "目标超出外侧可达边界" : "目标位于内侧不可达区域", r, inner, outer };
    }
    // 仅消除边界处浮点舍入误差；不可达点已在上面拒绝，不做目标投影。
    const cosine = clamp((x*x + y*y - l1*l1 - l2*l2) / (2*l1*l2), -1, 1);
    const sign = branch === "up" ? -1 : 1;
    const q2 = sign * Math.acos(cosine);
    const degenerate = r < tolerance && inner < tolerance;
    // 等长连杆且目标为原点时 q1 任意；保留参考 q1，避免 atan2(0,0) 歧义。
    const q1 = degenerate ? reference[0]
      : Math.atan2(y, x) - Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2));
    const angles = [nearest(q1, reference[0]), nearest(q2, reference[1])];
    const endpoint = fk(...angles, l1, l2);
    return { reachable: true, angles, cosine, endpoint, r, inner, outer,
      residual: Math.hypot(endpoint.x - x, endpoint.y - y),
      singular: Math.abs(Math.sin(q2)) < 1e-7, degenerate };
  }

  function pidStep(state, goals, gains, dt, params = {}) {
    const inertia = params.inertia || [1, 0.75];
    const damping = params.damping || [0.7, 0.55];
    const torqueLimit = params.torqueLimit || 45;
    const integralLimit = params.integralLimit || 3;
    const torques = [], saturated = [];
    for (let j = 0; j < 2; j++) {
      // goal 是选取最近 2π 等价解后的连续角，不在每一步 wrap，避免 π 处抖动。
      const error = goals[j] - state.q[j];
      let integral = gains.ki > 0 ? clamp(state.integral[j] + error * dt, -integralLimit, integralLimit) : 0;
      let raw = gains.kp * error + gains.ki * integral - gains.kd * state.v[j];
      // 条件积分 anti-windup：若积分会把饱和推得更严重，则冻结该次积分。
      if (Math.abs(raw) > torqueLimit && Math.sign(raw) === Math.sign(error)) {
        integral = gains.ki > 0 ? state.integral[j] : 0;
        raw = gains.kp * error + gains.ki * integral - gains.kd * state.v[j];
      }
      const torque = clamp(raw, -torqueLimit, torqueLimit);
      state.integral[j] = integral;
      state.v[j] += (torque - damping[j] * state.v[j]) / inertia[j] * dt;
      state.q[j] += state.v[j] * dt; // 半隐式 Euler；固定时间步，独立于刷新率。
      torques.push(torque);
      saturated.push(Math.abs(raw) > torqueLimit);
    }
    return { torques, saturated };
  }

  const api = { fk, ik, pidStep, clamp, wrap, nearest, deg, rad };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.RobotMath = api;
})(typeof window === "undefined" ? globalThis : window);
