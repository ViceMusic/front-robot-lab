"use strict";
(() => {
  const M = window.RobotMath;
  const $ = id => document.getElementById(id);
  const DT = 1/240, SAMPLE_PERIOD = 1/25, HISTORY_SECONDS = 20;
  const colors = { ink: "#29434a", actual: "#e66b48", target: "#148a86", blue: "#627ac4" };
  const state = { q: [M.rad(35), M.rad(65)], v: [0,0], integral: [0,0] };
  let lengths = [1.25, 1], goal = state.q.slice(), mode = "pid", paused = false;
  let requested = M.fk(...goal, ...lengths), accepted = { x: requested.x, y: requested.y };
  let solution = null, source = "fk", rejected = false, inputError = "";
  let gains = { kp:26, ki:0, kd:8 }, torque = [0,0], saturation = [false,false];
  let simTime = 0, sampleClock = 0, accumulator = 0, lastTime = null, lastUI = 0;
  let samples = [], trail = [], viewport = {}, stepIndex = 0;
  const arm = $("arm"), ctx = arm.getContext("2d");
  const format = (n, digits=3) => (Math.abs(n) < 0.5*10**(-digits) ? 0 : n).toFixed(digits);
  const pointText = p => `(${format(p.x)}, ${format(p.y)}) m`;
  const actual = () => M.fk(...state.q, ...lengths);

  function clearHistory() {
    samples = []; trail = []; sampleClock = 0;
    recordSample();
  }
  function recordSample() {
    const p = actual();
    samples.push({t:simTime, q:state.q.map(M.deg), goal:goal.map(M.deg), error:Math.hypot(p.x-accepted.x,p.y-accepted.y)});
    while(samples.length && samples[0].t < simTime-HISTORY_SECONDS) samples.shift();
    if (!trail.length || Math.hypot(p.x-trail[trail.length-1].x,p.y-trail[trail.length-1].y)>0.002) trail.push({x:p.x,y:p.y});
    if(trail.length>1600) trail.shift();
  }
  function syncInputs() {
    $("q1").value=M.deg(M.wrap(goal[0])); $("q2").value=M.deg(M.wrap(goal[1]));
    $("target-x").value=format(requested.x,4); $("target-y").value=format(requested.y,4);
  }
  function zeroController() { state.integral=[0,0]; torque=[0,0]; saturation=[false,false]; }
  function applyGoal(angles) {
    goal=angles.map((a,i)=>M.nearest(a,state.q[i]));
    accepted=M.fk(...goal,...lengths);
    zeroController();
    if(mode==="direct") {state.q=goal.slice(); state.v=[0,0];}
    syncInputs(); renderUI();
  }
  function fromFK() {
    source="fk"; rejected=false; inputError="";
    const angles=[M.rad(Number($("q1").value)), M.rad(Number($("q2").value))];
    requested=M.fk(...angles,...lengths);
    solution=M.ik(requested.x,requested.y,...lengths,$("branch").value,state.q);
    applyGoal(angles);
  }
  function fromIK(x,y) {
    source="ik";
    if(!Number.isFinite(x)||!Number.isFinite(y)) {
      inputError="请输入有限的 x、y 坐标。"; renderUI(); return;
    }
    inputError="";requested={x,y};
    solution=M.ik(x,y,...lengths,$("branch").value,state.q);
    rejected=!solution.reachable;
    if(rejected) {syncInputs();renderUI();return;}
    applyGoal(solution.angles);
  }
  function setMode(next) {
    mode=next; zeroController();
    if(mode==="direct") {state.q=goal.slice();state.v=[0,0];}
    $("mode-pid").classList.toggle("active",mode==="pid");
    $("mode-direct").classList.toggle("active",mode==="direct");
    $("mode-pid").setAttribute("aria-pressed",String(mode==="pid"));
    $("mode-direct").setAttribute("aria-pressed",String(mode==="direct"));
    $("mode-hint").textContent=mode==="pid" ? "滑块设置目标角，实体机械臂通过 PID 跟随。" : "直接把目标角赋给实际角，不经过动力学和 PID。";
    renderUI();
  }
  function renderUI() {
    const p=actual(), error=Math.hypot(p.x-accepted.x,p.y-accepted.y);
    $("q1-value").textContent=`${format(M.deg(M.wrap(goal[0])),1)}°`;
    $("q2-value").textContent=`${format(M.deg(M.wrap(goal[1])),1)}°`;
    for(const key of ["kp","ki","kd"]) $(key+"-value").textContent=format(gains[key],1);
    $("requested-value").textContent=pointText(requested);
    $("requested-value").style.color=rejected?"#bb5034":colors.target;
    $("actual-value").textContent=pointText(p);
    $("error-value").textContent=`${format(error,4)} m`;
    $("actual-angles").textContent=state.q.map(a=>format(M.deg(a),1)+"°").join(" / ");
    $("reach-label").textContent=`可达半径 ${format(Math.abs(lengths[0]-lengths[1]),2)}–${format(lengths[0]+lengths[1],2)} m`;
    const status=$("status");
    if(inputError) {status.className="status bad";status.textContent=inputError;}
    else if(rejected) {status.className="status bad";status.textContent=`${solution.reason}；未下发，继续执行上一有效目标。`;}
    else if(solution?.singular && source==="ik") {status.className="status warning";status.textContent=solution.degenerate?"等长连杆折叠至原点：θ₁ 有无穷多解，保留当前 θ₁。":"边界奇异姿态：肘上/肘下解在此合并。";}
    else {status.className="status";status.textContent=source==="fk"?"FK 目标已更新 · 可在画布点击体验 IK":"IK 解已验证 · 目标已下发至关节控制器";}
    $("fk-result").textContent=`实际 θ = [${state.q.map(a=>format(M.deg(a),2)+"°").join(", ")}] → ${pointText(p)}`;
    if(solution?.reachable) {
      const verify=M.fk(...solution.angles,...lengths);
      $("ik-result").textContent=`请求 ${pointText(requested)}\n c₂=${format(solution.cosine,5)}；${$("branch").value==="up"?"肘上":"肘下"} θ=[${solution.angles.map(a=>format(M.deg(a),2)+"°").join(", ")}]`;
      $("ik-verification").textContent=`FK(IK(p)) = ${pointText(verify)}；残差 ${solution.residual.toExponential(2)} m。${source==="fk"?"这是所选 IK 分支的候选解；当前滑块仍驱动 FK 目标。":"该解作为关节目标执行。"}`;
    } else {
      $("ik-result").textContent="不存在实数关节解，未截断或投影请求目标。";
      $("ik-verification").textContent=`可达条件：|L₁−L₂| ≤ √(x²+y²) ≤ L₁+L₂。当前执行目标仍为 ${pointText(accepted)}。`;
    }
    $("pid-result").textContent=`目标 θ*=[${goal.map(a=>format(M.deg(a),2)+"°").join(", ")}]；e=[${goal.map((q,i)=>format(q-state.q[i],4)).join(", ")}] rad`;
    $("torques").textContent=`${torque.map(x=>format(x,2)).join(" / ")} N·m`;
    $("pid-note").textContent=mode==="direct"?"直接 FK 模式：PID 与动力学已旁路。":saturation.some(Boolean)?"力矩已饱和（±45 N·m）· 条件积分抗饱和中":"固定步长 1/240 s · 限幅 ±45 N·m · 积分限幅 ±3 rad·s";
    $("pause").textContent=paused?"继续":"暂停";
    $("live-text").textContent=paused?"仿真已暂停":"仿真运行中";
    $("live-dot").style.background=paused?"#d7b578":"#93d8b6";
  }

  function canvasSize(canvas) {
    const bounds=canvas.getBoundingClientRect(), dpr=Math.min(window.devicePixelRatio||1,2);
    const w=Math.max(1,bounds.width), h=Math.max(1,bounds.height);
    if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)) {canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
    const c=canvas.getContext("2d");c.setTransform(dpr,0,0,dpr,0,0);return {w,h,c};
  }
  function drawArm() {
    const {w,h}=canvasSize(arm), outer=lengths[0]+lengths[1];
    viewport={w,h,cx:w/2,cy:h/2-3,scale:Math.min(w-100,h-75)/(2*outer*1.16)};
    const {cx,cy,scale}=viewport;
    const pixel=p=>({x:cx+p.x*scale,y:cy-p.y*scale});
    ctx.clearRect(0,0,w,h);
    ctx.strokeStyle="#eaf0e9";ctx.lineWidth=1;
    const gridStep=outer>3?1:0.5;
    for(let x=-Math.ceil(w/scale/gridStep);x<=Math.ceil(w/scale/gridStep);x++){
      const at=cx+x*gridStep*scale;ctx.beginPath();ctx.moveTo(at,0);ctx.lineTo(at,h);ctx.stroke();
    }
    for(let y=-Math.ceil(h/scale/gridStep);y<=Math.ceil(h/scale/gridStep);y++){
      const at=cy+y*gridStep*scale;ctx.beginPath();ctx.moveTo(0,at);ctx.lineTo(w,at);ctx.stroke();
    }
    ctx.beginPath();ctx.arc(cx,cy,outer*scale,0,Math.PI*2);ctx.fillStyle="#198a8607";ctx.fill();
    ctx.setLineDash([4,5]);ctx.strokeStyle="#a9c4b8";ctx.stroke();
    const inner=Math.abs(lengths[0]-lengths[1]);
    if(inner>0.0001){ctx.beginPath();ctx.arc(cx,cy,inner*scale,0,Math.PI*2);ctx.fillStyle="#ddd6c344";ctx.fill();ctx.strokeStyle="#c7bc9b";ctx.stroke();}
    ctx.setLineDash([]);ctx.strokeStyle="#b9c8be";ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(18,cy);ctx.lineTo(w-18,cy);ctx.moveTo(cx,15);ctx.lineTo(cx,h-30);ctx.stroke();
    ctx.font="10px ui-monospace,monospace";ctx.fillStyle="#81918a";
    ctx.fillText("x / m",w-48,cy-9);ctx.fillText("y / m",cx+9,24);
    for(let x=-Math.floor((w/2-30)/scale/gridStep);x<=Math.floor((w/2-30)/scale/gridStep);x++){
      if(x!==0)ctx.fillText(format(x*gridStep,1),cx+x*gridStep*scale-9,cy+16);
    }
    for(let y=-Math.floor((h/2-35)/scale/gridStep);y<=Math.floor((h/2-35)/scale/gridStep);y++){
      if(y!==0)ctx.fillText(format(y*gridStep,1),cx+8,cy-y*gridStep*scale+3);
    }
    if($("show-trail").checked && trail.length>1){ctx.beginPath();trail.forEach((p,i)=>{const v=pixel(p);i?ctx.lineTo(v.x,v.y):ctx.moveTo(v.x,v.y);});ctx.lineWidth=2;ctx.strokeStyle="#e66b4866";ctx.stroke();}
    const desired=M.fk(...goal,...lengths), desElbow=pixel(desired.elbow), desEnd=pixel(desired);
    ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(desElbow.x,desElbow.y);ctx.lineTo(desEnd.x,desEnd.y);ctx.lineWidth=6;ctx.lineCap="round";ctx.strokeStyle="#148a8633";ctx.setLineDash([5,7]);ctx.stroke();ctx.setLineDash([]);
    const end=actual(), elbow=pixel(end.elbow), tip=pixel(end);
    // 连杆外描边与细色带，让实际姿态和目标虚影容易区分。
    ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(elbow.x,elbow.y);ctx.lineTo(tip.x,tip.y);ctx.lineWidth=15;ctx.strokeStyle=colors.ink;ctx.stroke();
    ctx.lineWidth=7;ctx.strokeStyle="#98acaa";ctx.stroke();
    ctx.beginPath();ctx.moveTo(elbow.x,elbow.y);ctx.lineTo(tip.x,tip.y);ctx.strokeStyle=colors.actual;ctx.stroke();
    function joint(x,y,r,fill){ctx.beginPath();ctx.arc(x,y,r,0,Math.PI*2);ctx.fillStyle=fill;ctx.fill();ctx.lineWidth=3;ctx.strokeStyle=colors.ink;ctx.stroke();}
    joint(cx,cy,13,"#f5f5ee");joint(elbow.x,elbow.y,10,"#fafbf7");joint(tip.x,tip.y,6,colors.actual);
    ctx.fillStyle=colors.ink;ctx.beginPath();ctx.arc(cx,cy,4,0,Math.PI*2);ctx.fill();
    ctx.font="11px ui-monospace,monospace";ctx.fillText("BASE",cx-15,cy+32);
    ctx.fillStyle="#657c7a";ctx.fillText(`L₁ ${format(lengths[0],2)} m`,(cx+elbow.x)/2+12,(cy+elbow.y)/2-12);
    ctx.fillText(`L₂ ${format(lengths[1],2)} m`,(tip.x+elbow.x)/2+10,(tip.y+elbow.y)/2-12);
    // 关节角弧线（θ2 的零方向延续第一连杆）。
    ctx.strokeStyle="#e66b4899";ctx.lineWidth=1.5;
    const q1=M.wrap(state.q[0]),q2=M.wrap(state.q[1]);
    ctx.beginPath();ctx.arc(cx,cy,29,0,-q1,q1>0);ctx.stroke();
    ctx.beginPath();ctx.arc(elbow.x,elbow.y,23,-state.q[0],-state.q[0]-q2,q2>0);ctx.stroke();
    ctx.fillStyle=colors.actual;ctx.fillText("θ₁",cx+34,cy-14);ctx.fillText("θ₂",elbow.x+25,elbow.y+20);
    function cross(p,color,label){
      const v=pixel(p), x=M.clamp(v.x,18,w-18),y=M.clamp(v.y,18,h-34);
      ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(x,y,10,0,Math.PI*2);ctx.moveTo(x-15,y);ctx.lineTo(x+15,y);ctx.moveTo(x,y-15);ctx.lineTo(x,y+15);ctx.stroke();
      ctx.fillStyle=color;ctx.font="10px ui-monospace,monospace";ctx.fillText(label+(x!==v.x||y!==v.y?" · 画布外":""),M.clamp(x+16,8,w-130),M.clamp(y-15,16,h-40));
    }
    cross(accepted,colors.target,"执行目标");
    if(rejected)cross(requested,"#c45238","不可达请求");
    if(Math.hypot(end.x-accepted.x,end.y-accepted.y)>.02){ctx.setLineDash([3,4]);ctx.strokeStyle="#e66b4877";ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(tip.x,tip.y);ctx.lineTo(desEnd.x,desEnd.y);ctx.stroke();ctx.setLineDash([]);}
  }

  function drawChart(id,j,color) {
    const {w,h,c}=canvasSize($(id)), pad={l:36,r:9,t:9,b:22}, pw=w-pad.l-pad.r, ph=h-pad.t-pad.b;
    c.clearRect(0,0,w,h);
    let min=0,max=1;
    if(samples.length){const values=samples.flatMap(s=>j<2?[s.q[j],s.goal[j]]:[s.error]); min=j<2?Math.min(...values):0;max=Math.max(...values);const range=Math.max(max-min,j<2?20:.05);if(j<2){min-=range*.12;max+=range*.12;}else max=Math.max(max,.05)*1.12;}
    const span=Math.max(max-min,0.01),end=Math.max(HISTORY_SECONDS,simTime),start=end-HISTORY_SECONDS;
    const px=t=>pad.l+(t-start)/HISTORY_SECONDS*pw,py=v=>pad.t+ph-(v-min)/span*ph;
    c.font="9px ui-monospace,monospace";c.lineWidth=1;
    for(let k=0;k<4;k++){const v=min+span*k/3,y=py(v);c.strokeStyle="#e9eee9";c.beginPath();c.moveTo(pad.l,y);c.lineTo(w-pad.r,y);c.stroke();c.fillStyle="#8a9690";c.fillText(j<2?Math.round(v).toString():v.toFixed(2),1,y+3);}
    for(let k=0;k<3;k++){const t=start+HISTORY_SECONDS*k/2;c.fillStyle="#8a9690";c.fillText(`${t.toFixed(0)}s`,M.clamp(px(t)-8,0,w-22),h-4);}
    c.save();c.beginPath();c.rect(pad.l,pad.t,pw,ph);c.clip();
    function line(target){c.beginPath();samples.forEach((s,i)=>{const v=j<2?(target?s.goal[j]:s.q[j]):s.error;i?c.lineTo(px(s.t),py(v)):c.moveTo(px(s.t),py(v));});c.strokeStyle=target?color+"77":color;c.lineWidth=target?1.3:1.8;c.setLineDash(target?[4,4]:[]);c.stroke();}
    if(j<2)line(true);line(false);c.restore();c.setLineDash([]);
  }
  function renderCharts(){drawChart("chart-q1",0,colors.actual);drawChart("chart-q2",1,colors.blue);drawChart("chart-error",2,colors.target);}

  for(const id of ["q1","q2"]) $(id).addEventListener("input",fromFK);
  for(const id of ["l1","l2"]) $(id).addEventListener("change",()=>{
    const next=[Number($("l1").value),Number($("l2").value)];
    if(!next.every(v=>Number.isFinite(v)&&v>=.2&&v<=2.5)){ $(id).value=lengths[id==="l1"?0:1];return; }
    lengths=next; zeroController();
    if(source==="ik"){accepted=M.fk(...goal,...lengths);fromIK(requested.x,requested.y);}else fromFK();
    clearHistory();
  });
  $("solve").addEventListener("click",()=>{
    if($("target-x").value.trim()===""||$("target-y").value.trim()===""){inputError="请先填写 x 和 y 坐标。";renderUI();return;}
    fromIK(Number($("target-x").value),Number($("target-y").value));
  });
  for(const id of ["target-x","target-y"]) $(id).addEventListener("keydown",e=>{if(e.key==="Enter")$("solve").click();});
  $("branch").addEventListener("change",()=>fromIK(requested.x,requested.y));
  $("mode-pid").addEventListener("click",()=>setMode("pid"));$("mode-direct").addEventListener("click",()=>setMode("direct"));
  $("pause").addEventListener("click",()=>{paused=!paused;accumulator=0;renderUI();});
  $("clear-trail").addEventListener("click",clearHistory);
  for(const key of ["kp","ki","kd"]) $(key).addEventListener("input",()=>{gains[key]=Number($(key).value);state.integral=[0,0];document.querySelectorAll(".presets button").forEach(b=>b.classList.remove("chosen"));renderUI();});
  function preset(name){const presets={smooth:[26,0,8],overshoot:[38,0,0.7],integral:[18,6,3]};[gains.kp,gains.ki,gains.kd]=presets[name];for(const k of ["kp","ki","kd"])$(k).value=gains[k];state.integral=[0,0];document.querySelectorAll(".presets button").forEach(b=>b.classList.toggle("chosen",b.dataset.preset===name));renderUI();}
  document.querySelectorAll("[data-preset]").forEach(b=>b.addEventListener("click",()=>preset(b.dataset.preset)));
  $("step-test").addEventListener("click",()=>{setMode("pid");const steps=[[70,-65],[-25,110]];const pose=steps[stepIndex++%2];$("q1").value=pose[0];$("q2").value=pose[1];fromFK();});
  $("reset").addEventListener("click",()=>{
    lengths=[1.25,1];$("l1").value=1.25;$("l2").value=1;state.q=[M.rad(35),M.rad(65)];state.v=[0,0];goal=state.q.slice();
    source="fk";rejected=false;inputError="";paused=false;simTime=0;accumulator=0;stepIndex=0;$("branch").value="up";$("show-trail").checked=true;
    requested=M.fk(...goal,...lengths);accepted={x:requested.x,y:requested.y};solution=M.ik(requested.x,requested.y,...lengths,"up",state.q);
    setMode("pid");preset("smooth");syncInputs();clearHistory();renderUI();
  });
  function fromPointer(event,send){
    const box=arm.getBoundingClientRect();if(!viewport.scale)return;
    const x=(event.clientX-box.left-viewport.cx)/viewport.scale, y=-(event.clientY-box.top-viewport.cy)/viewport.scale;
    $("pointer-label").textContent=`鼠标 (${format(x,2)}, ${format(y,2)}) m`;
    if(send)fromIK(x,y);
  }
  let dragging=false;
  arm.addEventListener("pointerdown",e=>{if(e.button!==0)return;dragging=true;arm.setPointerCapture(e.pointerId);fromPointer(e,true);});
  arm.addEventListener("pointermove",e=>fromPointer(e,dragging));
  arm.addEventListener("pointerup",()=>{dragging=false;});arm.addEventListener("pointercancel",()=>{dragging=false;});arm.addEventListener("lostpointercapture",()=>{dragging=false;});
  document.addEventListener("visibilitychange",()=>{lastTime=null;accumulator=0;});

  function frame(now){
    if(lastTime===null)lastTime=now;
    const elapsed=Math.min((now-lastTime)/1000,.1);lastTime=now;
    if(!paused){accumulator+=elapsed;while(accumulator>=DT){
      if(mode==="pid"){const output=M.pidStep(state,goal,gains,DT);torque=output.torques;saturation=output.saturated;}
      simTime+=DT;sampleClock+=DT;accumulator-=DT;
      if(sampleClock>=SAMPLE_PERIOD){sampleClock-=SAMPLE_PERIOD;recordSample();}
    }}
    drawArm();
    if(now-lastUI>80){renderUI();renderCharts();lastUI=now;}
    requestAnimationFrame(frame);
  }
  solution=M.ik(requested.x,requested.y,...lengths,"up",state.q);
  syncInputs();renderUI();recordSample();requestAnimationFrame(frame);
  // 只读快照方便在浏览器控制台教学检查，不暴露内部可变状态。
  window.robotLab={snapshot:()=>JSON.parse(JSON.stringify({q:state.q,v:state.v,goal,lengths,requested,accepted,solution,mode,paused,simTime,samples:samples.length}))};
})();
