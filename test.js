"use strict";
const assert = require("node:assert/strict");
const M = require("./math.js");
const near = (a,b,tol=1e-8) => assert.ok(Math.abs(a-b)<tol,`${a} != ${b}`);
near(M.fk(0,0,1.25,1).x,2.25);near(M.fk(0,0,1.25,1).y,0);
near(M.fk(Math.PI/2,0,1.25,1).x,0);near(M.fk(Math.PI/2,0,1.25,1).y,2.25);
near(M.fk(0,Math.PI/2,1,1).x,1);near(M.fk(0,Math.PI/2,1,1).y,1);
let count=0;
for(const lengths of [[1.25,1],[1,1],[0.4,2.5],[2.5,0.4]]){
  for(let i=0;i<600;i++){
    const q1=i*0.719-120,q2=i*0.491-43;
    const target=M.fk(q1,q2,...lengths);
    for(const branch of ["up","down"]){
      const result=M.ik(target.x,target.y,...lengths,branch,[q1,q2]);
      assert.ok(result.reachable);assert.ok(result.residual<1e-7);
      assert.ok(result.angles.every(Number.isFinite));
      result.angles.forEach((q,j)=>assert.ok(Math.abs(q-[q1,q2][j])<=Math.PI+1e-10));count++;
    }
  }
}
assert.equal(M.ik(3,0,1,1).reachable,false);
assert.equal(M.ik(0,0,1.25,1).reachable,false);
assert.equal(M.ik(0.249,0,1.25,1).reachable,false);
assert.equal(M.ik(NaN,0,1,1).reachable,false);
for(const p of [[2.25,0],[0.25,0],[-2.25,0],[0,-2.25]]){
  const result=M.ik(...p,1.25,1,"up");assert.ok(result.reachable&&result.singular);assert.ok(result.residual<1e-8);
}
for(const branch of ["up","down"]){const r=M.ik(0,0,1,1,branch,[1.2,0]);assert.ok(r.degenerate);near(r.angles[0],1.2);near(r.residual,0);}
const up=M.ik(1,1,1.25,1,"up"),down=M.ik(1,1,1.25,1,"down");
assert.ok(up.angles[1]<0&&down.angles[1]>0);
const state={q:[0,0],v:[0,0],integral:[0,0]},target=[1,-.8];
for(let i=0;i<240*12;i++)M.pidStep(state,target,{kp:26,ki:0,kd:8},1/240);
state.q.forEach((q,i)=>near(q,target[i],1e-5));
const wild={q:[0,0],v:[0,0],integral:[0,0]};
for(let i=0;i<240*20;i++){const r=M.pidStep(wild,[3,-3],{kp:100,ki:20,kd:0},1/240);assert.ok(r.torques.every(t=>Math.abs(t)<=45));assert.ok(wild.q.every(Number.isFinite));assert.ok(wild.integral.every(v=>Math.abs(v)<=3));}
console.log(`PASS: FK axes/relative angles, ${count} IK→FK roundtrips, both branches, inner/outer boundaries, singular origin, nearest angles, PID convergence and saturation.`);
