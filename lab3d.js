/* Three.js 只负责显示。FK / IK 复用独立、可测试的 math.js。
 * 数学平面 (x,y) 映射到 Three 世界 (x, height, -y)，两关节绕世界 Y 轴转动。
 * 本页是运动学演示：无动力学、插值或隐含的 PID 控制。
 */
(() => {
  "use strict";
  const $ = id => document.getElementById(id);
  const stage = $("stage"), M = window.RobotMath, T = window.THREE;
  let renderer;
  try {
    renderer = new T.WebGLRenderer({ antialias: true, alpha: true });
  } catch (error) {
    $("failure").hidden = false;
    $("failure").innerHTML = '无法启动 WebGL。请使用支持 WebGL 的浏览器，或打开 <a href="index-2d.html">不需要 WebGL 的二维版</a>。';
    console.error(error);
    return;
  }
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  stage.appendChild(renderer.domElement);
  const scene = new T.Scene();
  const camera = new T.PerspectiveCamera(38, 1, 0.1, 100);
  scene.add(new T.HemisphereLight(0xffffff, 0x80917a, 1.8));
  const light = new T.DirectionalLight(0xffffff, 1.8);
  light.position.set(3, 6, 2); scene.add(light);
  const material = color => new T.MeshStandardMaterial({ color, roughness: 0.48, metalness: 0.18 });
  const dark = material(0x2b4b47), silver = material(0x9cae98), orange = material(0xe78d57);
  const targetMaterial = material(0x239986);
  function mesh(geometry, mat, parent, x=0, y=0, z=0) {
    const object = new T.Mesh(geometry, mat);
    object.position.set(x,y,z); parent.add(object); return object;
  }
  const ground = mesh(new T.CircleGeometry(4,96), material(0xe5eadc), scene,0,-0.025);
  ground.rotation.x = -Math.PI/2;
  const grid = new T.GridHelper(8,32,0xc2cdbb,0xd4ddcb);
  grid.material.transparent = true; grid.material.opacity = 0.5; scene.add(grid);
  const l1=1.25, l2=1, height=0.28;
  function ring(radius) {
    const points = Array.from({length:129},(_,i) => {
      const a=i/128*Math.PI*2; return new T.Vector3(radius*Math.cos(a),0.012,-radius*Math.sin(a));
    });
    const line=new T.Line(new T.BufferGeometry().setFromPoints(points),
      new T.LineDashedMaterial({color:0x95a78f,dashSize:0.045,gapSize:0.035}));
    line.computeLineDistances(); scene.add(line);
  }
  ring(l1+l2); ring(Math.abs(l1-l2));
  function label(text,x,z) {
    const canvas=document.createElement("canvas"); canvas.width=128;canvas.height=64;
    const ctx=canvas.getContext("2d");ctx.font="32px sans-serif";ctx.fillStyle="#697d66";
    ctx.textAlign="center";ctx.fillText(text,64,43);
    const sprite=new T.Sprite(new T.SpriteMaterial({map:new T.CanvasTexture(canvas),depthTest:false}));
    sprite.position.set(x,0.08,z);sprite.scale.set(0.4,0.2,1);scene.add(sprite);
  }
  label("+x",2.65,0);label("+y",0,-2.65);
  mesh(new T.CylinderGeometry(.24,.29,.14,40),dark,scene,0,.07);
  const shoulder=new T.Group();shoulder.position.y=height;scene.add(shoulder);
  mesh(new T.CylinderGeometry(.17,.17,.28,40),dark,shoulder);
  mesh(new T.CylinderGeometry(.09,.09,.025,40),orange,shoulder,0,.155);
  mesh(new T.BoxGeometry(l1,.12,.16),silver,shoulder,l1/2);
  const elbow=new T.Group();elbow.position.x=l1;shoulder.add(elbow);
  mesh(new T.CylinderGeometry(.135,.135,.22,40),dark,elbow);
  mesh(new T.CylinderGeometry(.065,.065,.025,40),orange,elbow,0,.125);
  mesh(new T.BoxGeometry(l2,.13,.13),dark,elbow,l2/2);
  const end=mesh(new T.SphereGeometry(.095,24,16),orange,elbow,l2);
  const marker=new T.Group();scene.add(marker);
  mesh(new T.TorusGeometry(.105,.013,8,40),targetMaterial,marker).rotation.x=Math.PI/2;
  mesh(new T.CylinderGeometry(.012,.012,.2,12),targetMaterial,marker,0,.1);
  const trail=new T.Line(new T.BufferGeometry(),new T.LineBasicMaterial({color:0xc67b48,transparent:true,opacity:.7}));
  scene.add(trail);
  let trailPoints=[], mode="fk", q=[M.rad(30),M.rad(65)], branch="down";
  const endpoint=()=>M.fk(...q,l1,l2);
  let target={x:endpoint().x,y:endpoint().y}, solution=null, reachable=true;
  let yaw=.36, elevation=.9, distance=8.7, top=false;
  const keys=new Set(), pointers=new Set();
  function cameraUpdate() {
    camera.position.set(distance*Math.cos(elevation)*Math.sin(yaw),
      .25+distance*Math.sin(elevation),distance*Math.cos(elevation)*Math.cos(yaw));
    camera.lookAt(0,.25,0);
  }
  cameraUpdate();
  new ResizeObserver(()=>{
    const {width,height:h}=stage.getBoundingClientRect();
    renderer.setSize(width,h);camera.aspect=width/h;camera.updateProjectionMatrix();
  }).observe(stage);
  function clearTrail(){trailPoints=[];trail.geometry.setFromPoints([]);}
  function solve() {
    solution=M.ik(target.x,target.y,l1,l2,branch,q);
    reachable=solution.reachable;
    if(reachable) q=solution.angles;
  }
  function modeUpdate(next) {
    mode=next;keys.clear();pointers.clear();clearTrail();
    const p=endpoint();target={x:p.x,y:p.y};reachable=true;
    branch=Math.sin(q[1])<0?"up":"down";
    if(mode==="ik")solve();
    for(const id of ["fk","ik"]) {
      $(id).classList.toggle("active",id===mode);$(id).setAttribute("aria-pressed",String(id===mode));
      $(id+"-keys").hidden=id!==mode;
    }
    $("explanation").textContent=mode==="fk"
      ?"给定关节角度，计算手会到哪里。你直接转动关节，末端跟着走。"
      :"给定手要到的位置，反求关节角度。你移动目标，两个关节一起配合。";
    $("flow").textContent=mode==="fk"?"角度 θ₁、θ₂  →  FK  →  末端 x、y":"目标 x、y  →  IK  →  角度 θ₁、θ₂";
    $("elbow").disabled=mode!=="ik";
  }
  function reset(){q=[M.rad(30),M.rad(65)];modeUpdate(mode);}
  function toggleElbow(){if(mode==="ik"){branch=branch==="up"?"down":"up";clearTrail();solve();}}
  function toggleView(){top=!top;elevation=top?Math.PI/2-.001:.9;yaw=top?0:.36;cameraUpdate();}
  for(const id of ["fk","ik"])$(id).onclick=()=>{modeUpdate(id);stage.focus();};
  $("reset").onclick=()=>{reset();stage.focus();};
  $("view").onclick=()=>{toggleView();stage.focus();};
  $("elbow").onclick=()=>{toggleElbow();stage.focus();};
  const movement=["KeyA","KeyD","KeyW","KeyS","ArrowUp","ArrowDown","ArrowLeft","ArrowRight","ShiftLeft","ShiftRight"];
  stage.addEventListener("keydown",event=>{
    if(event.ctrlKey||event.metaKey||event.altKey)return;
    if(movement.includes(event.code)){event.preventDefault();keys.add(event.code);}
    if(event.repeat)return;
    if(event.code==="Tab"&&!event.shiftKey){event.preventDefault();modeUpdate(mode==="fk"?"ik":"fk");}
    if(event.code==="KeyE")toggleElbow();
    if(event.code==="KeyR")reset();
    if(event.code==="KeyV")toggleView();
    if(event.code==="KeyC")clearTrail();
    if(event.code==="Escape"){$("fk").focus();keys.clear();}
  });
  window.addEventListener("keyup",event=>keys.delete(event.code));
  const release=()=>{keys.clear();pointers.clear();};
  window.addEventListener("blur",release);
  stage.addEventListener("blur",()=>keys.clear());
  document.addEventListener("visibilitychange",release);
  document.querySelectorAll("[data-key]").forEach(button=>{
    button.addEventListener("pointerdown",event=>{
      event.preventDefault();stage.focus();button.setPointerCapture(event.pointerId);pointers.add(button.dataset.key);
    });
    for(const type of ["pointerup","pointercancel","lostpointercapture"])
      button.addEventListener(type,()=>pointers.delete(button.dataset.key));
  });
  let drag=null;
  stage.addEventListener("pointerdown",event=>{
    if(event.button!==0)return;
    stage.focus();stage.setPointerCapture(event.pointerId);
    drag={x:event.clientX,y:event.clientY,lastX:event.clientX,lastY:event.clientY,moved:false};
  });
  stage.addEventListener("pointermove",event=>{
    if(!drag)return;
    if(Math.hypot(event.clientX-drag.x,event.clientY-drag.y)>4)drag.moved=true;
    if(drag.moved){
      yaw-=(event.clientX-drag.lastX)*.006;
      elevation=M.clamp(elevation+(event.clientY-drag.lastY)*.006,.2,Math.PI/2-.001);
      top=false;cameraUpdate();
    }
    drag.lastX=event.clientX;drag.lastY=event.clientY;
  });
  stage.addEventListener("pointerup",event=>{
    if(drag&&!drag.moved&&mode==="ik"){
      const rect=stage.getBoundingClientRect(),ray=new T.Raycaster();
      ray.setFromCamera(new T.Vector2((event.clientX-rect.left)/rect.width*2-1,
        -(event.clientY-rect.top)/rect.height*2+1),camera);
      const hit=ray.ray.intersectPlane(new T.Plane(new T.Vector3(0,1,0),-height),new T.Vector3());
      if(hit){target={x:hit.x,y:-hit.z};solve();}
    }
    drag=null;
  });
  for(const type of ["pointercancel","lostpointercapture"])stage.addEventListener(type,()=>{drag=null;});
  stage.addEventListener("wheel",event=>{
    event.preventDefault();distance=M.clamp(distance*Math.exp(event.deltaY*.001),5,15);cameraUpdate();
  },{passive:false});
  function update(dt) {
    const held=code=>keys.has(code)||pointers.has(code);
    const slow = (held("ShiftLeft") || held("ShiftRight")) ? 0.25 : 1;
    if(mode==="fk"){
      q[0]+=(Number(held("KeyD"))-Number(held("KeyA")))*dt*slow;
      q[1]+=(Number(held("KeyW"))-Number(held("KeyS")))*dt*slow;
      q=q.map(M.wrap);
    }else{
      let dx=Number(held("ArrowRight"))-Number(held("ArrowLeft"));
      let dy=Number(held("ArrowUp"))-Number(held("ArrowDown"));
      const norm=Math.hypot(dx,dy);
      if(norm){target.x+=dx/norm*dt*.65*slow;target.y+=dy/norm*dt*.65*slow;solve();}
    }
    const p=endpoint();
    shoulder.rotation.y=q[0];elbow.rotation.y=q[1];
    marker.visible=mode==="ik";marker.position.set(target.x,height,-target.y);
    targetMaterial.color.set(reachable?0x239986:0xc95142);
    const last=trailPoints[trailPoints.length-1], point=new T.Vector3(p.x,height,-p.y);
    if(!last||last.distanceTo(point)>.003){
      trailPoints.push(point);if(trailPoints.length>800)trailPoints.shift();trail.geometry.setFromPoints(trailPoints);
    }
    $("angles").textContent=q.map(a=>M.deg(M.wrap(a)).toFixed(1)+"°").join(" / ");
    const xy=p=>p.x.toFixed(3)+" / "+p.y.toFixed(3)+" m";
    $("position").textContent=xy(p);$("target").textContent=mode==="ik"?xy(target):"— 由角度决定";
    const error=Math.hypot(p.x-target.x,p.y-target.y);
    const status=mode==="fk"?"FK：直接控制关节角度":!reachable
      ?"不可达：保留最后有效姿态，请移回圆环区域"
      :"IK："+(branch==="up"?"肘上":"肘下")+
        (error<1e-9?" · FK 验证通过（误差 < 10⁻⁹ m）":" · FK 验证偏差，请查看公式");
    // 只在状态文字变化时更新，避免辅助技术无意义重复播报。
    if($("status").textContent!==status)$("status").textContent=status;
    $("status").classList.toggle("error",!reachable);
    $("verify").textContent=mode==="ik"
      ? (reachable?"当前 c₂ = "+solution.cosine.toFixed(5)+"；FK(IK(目标)) 误差 = "+error.toExponential(2)+" m":"要求 0.25 ≤ √(x²+y²) ≤ 2.25 m")
      :"当前输入："+$("angles").textContent+"；FK 输出："+xy(p);
    document.querySelectorAll("[data-key]").forEach(b=>b.classList.toggle("pressed",held(b.dataset.key)));
  }
  // 只读快照供无依赖浏览器冒烟测试使用；不会暴露内部可变状态。
  window.robotLab3D={
    snapshot:()=>JSON.parse(JSON.stringify({mode,q,target,endpoint:endpoint(),reachable,branch,solution})),
    checkScene:()=>{scene.updateMatrixWorld(true);return end.getWorldPosition(new T.Vector3()).toArray();}
  };
  modeUpdate(mode);stage.focus();
  let previous=performance.now();
  function frame(now){
    const dt=Math.min((now-previous)/1000,.035);previous=now;
    update(dt);renderer.render(scene,camera);requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
})();
