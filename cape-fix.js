/* Cape fix for the Meshy-rigged fighters (used by index.html and model-viewer.html).

   Meshy's auto-rig ties each piece of the cape to the nearest bone: the lower
   cape to the legs, the upper cape to the arms and shoulders. So the cape rips
   open whenever the limbs move. This finds the cape and re-ties it:

   - a vertex is cape if a ray from it to "its" bone line passes through another
     surface first (the clothes underneath). Leg-weighted vertices test against
     their leg bone; the rest test against the spine, but only from behind.
   - cape vertices get smooth weights (no jumps between neighbours, so no tears):
     upper back follows Spine02, the middle follows the hips, and below the hips
     a share of each thigh, left side to the left leg, right side to the right.
   - cape vertices are pushed a little away from the body so the clothes don't
     poke through: more at the hem than at the shoulders.

   Mesh space must be the model's own: +Z forward, Y up, metres (true for Meshy). */
window.fixCape = function fixCape(mesh, mode = 'fix') {
  const THREE = window.THREE;
  const g = mesh.geometry, pos = g.attributes.position, P = pos.array, I = g.index.array, NA = g.attributes.normal;
  const SI = g.attributes.skinIndex.array, SW = g.attributes.skinWeight.array;
  const sk = mesh.skeleton, names = sk.bones.map(b => b.name), id = n => names.indexOf(n);
  if (id('Hips') < 0 || id('Spine02') < 0) return 0;
  const at = n => new THREE.Vector3().setFromMatrixPosition(sk.boneInverses[id(n)].clone().invert())
                    .applyMatrix4(mesh.bindMatrixInverse);
  // each limb bone and the segment it runs along; FAR = no clothing sits that far from it
  const seg = {}, far = {};
  const addSeg = (a, b, lim) => { if (id(a) >= 0 && id(b) >= 0) { seg[id(a)] = [at(a), at(b)]; far[id(a)] = lim; } };
  for (const s of ['Left', 'Right']) {
    addSeg(s + 'UpLeg', s + 'Leg', 0.24); addSeg(s + 'Leg', s + 'Foot', 0.22); addSeg(s + 'Foot', s + 'ToeBase', 0.2);
    if (id(s + 'ToeBase') >= 0) { const t = at(s + 'ToeBase'); seg[id(s + 'ToeBase')] = [t, t]; far[id(s + 'ToeBase')] = 0.16; }
    addSeg(s + 'Arm', s + 'ForeArm', 0.2); addSeg(s + 'ForeArm', s + 'Hand', 0.2);
  }
  const legBone = new Set(['LeftUpLeg', 'LeftLeg', 'RightUpLeg', 'RightLeg'].map(id));
  const spine = [at('Hips'), at(id('neck') >= 0 ? 'neck' : 'Spine02')];
  const skip = new Set(names.map((n, i) => /head|hand|end/i.test(n) ? i : -1));

  // triangle boxes for a cheap ray test
  const nT = I.length / 3, bb = new Float32Array(nT * 6);
  for (let t = 0; t < nT; t++) for (let k = 0; k < 3; k++) {
    const o = I[t * 3 + k] * 3;
    for (let c = 0; c < 3; c++) {
      const x = P[o + c];
      if (k === 0 || x < bb[t * 6 + c]) bb[t * 6 + c] = x;
      if (k === 0 || x > bb[t * 6 + 3 + c]) bb[t * 6 + 3 + c] = x;
    }
  }
  const v = new THREE.Vector3(), c = new THREE.Vector3(), d = new THREE.Vector3(), ab = new THREE.Vector3();
  const e1 = new THREE.Vector3(), e2 = new THREE.Vector3(), pv = new THREE.Vector3(), tv = new THREE.Vector3(), qv = new THREE.Vector3();
  const A = new THREE.Vector3(), B = new THREE.Vector3(), C = new THREE.Vector3();
  const blocked = (len, vi, minD = 0.015) => {
    const lo0 = Math.min(v.x, c.x), lo1 = Math.min(v.y, c.y), lo2 = Math.min(v.z, c.z);
    const hi0 = Math.max(v.x, c.x), hi1 = Math.max(v.y, c.y), hi2 = Math.max(v.z, c.z);
    for (let t = 0; t < nT; t++) {
      const o = t * 6;
      if (bb[o] > hi0 || bb[o + 3] < lo0 || bb[o + 1] > hi1 || bb[o + 4] < lo1 || bb[o + 2] > hi2 || bb[o + 5] < lo2) continue;
      const a = I[t * 3], b = I[t * 3 + 1], cc = I[t * 3 + 2];
      if (a === vi || b === vi || cc === vi) continue;
      A.fromArray(P, a * 3); B.fromArray(P, b * 3); C.fromArray(P, cc * 3);
      e1.subVectors(B, A); e2.subVectors(C, A); pv.crossVectors(d, e2);
      const det = e1.dot(pv); if (Math.abs(det) < 1e-9) continue;
      const inv = 1 / det; tv.subVectors(v, A);
      const u = tv.dot(pv) * inv; if (u < 0 || u > 1) continue;
      qv.crossVectors(tv, e1);
      const w = d.dot(qv) * inv; if (w < 0 || u + w > 1) continue;
      const dist = e2.dot(qv) * inv;
      if (dist > minD && dist < len - 0.015) return true;
    }
    return false;
  };
  const closest = (s0, s1) => {
    ab.subVectors(s1, s0);
    const t = Math.max(0, Math.min(1, c.subVectors(v, s0).dot(ab) / ab.lengthSq()));
    return c.copy(s0).addScaledVector(ab, t);
  };

  // 1) find the cape (on the untouched mesh), remember which way is "out"
  const cape = [];
  for (let i = 0; i < pos.count; i++) {
    let best = 0, bw = -1;
    for (let k = 0; k < 4; k++) if (SW[i * 4 + k] > bw) { bw = SW[i * 4 + k]; best = SI[i * 4 + k]; }
    if (skip.has(best)) continue;
    v.fromArray(P, i * 3);
    const s = seg[best];
    closest(...(s || spine));
    const len = v.distanceTo(c); if (len < 0.03) continue;
    d.subVectors(c, v).normalize();
    // cape if it hangs too far from its limb to be clothing, or if a ray to the bone passes
    // through the clothes first: legs only from behind (knee pads and shoes are layered too,
    // so feet go by distance alone), torso only from behind the back
    let isC = s && len > far[best];
    if (!isC && s && legBone.has(best) && d.z >= 0.3) isC = blocked(len, i);
    if (!isC && !s && d.z >= 0.3) isC = blocked(len, i);
    // removing: anything behind the back that is a thin sheet (a ray straight in hits its
    // own other side within 5 cm) is cape; the body there is much thicker
    if (!isC && mode === 'remove' && v.z < spine[0].z - 0.02 && v.y > 0.12) {
      const nx = -NA.getX(i), ny = -NA.getY(i), nz = -NA.getZ(i);
      const sv = d.clone(), sc = c.clone();
      d.set(nx, ny, nz).normalize(); c.copy(v).addScaledVector(d, 0.065);
      isC = blocked(0.065, i, 0.001);
      d.copy(sv); c.copy(sc);
    }
    if (!isC) continue;
    const ox = -d.x, oz = -d.z, ol = Math.hypot(ox, oz) || 1;
    cape.push([i, ox / ol, oz / ol]);
  }

  // fill holes: a vertex behind the back whose mesh neighbours are mostly cape is
  // cape too (stray points left on an arm bone are what splits the cape into strips)
  const key = i => Math.round(P[i * 3] * 1e4) + ',' + Math.round(P[i * 3 + 1] * 1e4) + ',' + Math.round(P[i * 3 + 2] * 1e4);
  const weld = new Int32Array(pos.count), first = new Map();
  for (let i = 0; i < pos.count; i++) { const k = key(i); if (!first.has(k)) first.set(k, i); weld[i] = first.get(k); }
  const nb = new Map();
  const link = (a, b) => { a = weld[a]; b = weld[b]; if (a === b) return; (nb.get(a) || nb.set(a, new Set()).get(a)).add(b); (nb.get(b) || nb.set(b, new Set()).get(b)).add(a); };
  for (let t = 0; t < I.length; t += 3) { link(I[t], I[t + 1]); link(I[t + 1], I[t + 2]); link(I[t + 2], I[t]); }
  const isCape = new Map(cape.map(([i, ox, oz]) => [weld[i], [ox, oz]]));
  const backZ = spine[0].z;
  for (let pass = 0; pass < 4; pass++) {
    const add = [];
    for (const [w, set] of nb) {
      if (isCape.has(w) || P[w * 3 + 2] > backZ - 0.04 || P[w * 3 + 1] < 0.12) continue;
      let best = 0, bw = -1;
      for (let k = 0; k < 4; k++) if (SW[w * 4 + k] > bw) { bw = SW[w * 4 + k]; best = SI[w * 4 + k]; }
      if (skip.has(best)) continue;
      let n = 0, ox = 0, oz = 0;
      for (const o of set) { const c2 = isCape.get(o); if (c2) { n++; ox += c2[0]; oz += c2[1]; } }
      if (n * 2 > set.size) { const l = Math.hypot(ox, oz) || 1; add.push([w, ox / l, oz / l]); }
    }
    if (!add.length) break;
    for (const [w, ox, oz] of add) isCape.set(w, [ox, oz]);
  }
  cape.length = 0;
  for (let i = 0; i < pos.count; i++) { const c2 = isCape.get(weld[i]); if (c2) cape.push([i, c2[0], c2[1]]); }

  if (mode === 'remove') return removeTris(g, I, weld, isCape, P, backZ);

  // 2) re-tie and push out
  const hips = id('Hips'), sp = id('Spine02'), lu = id('LeftUpLeg'), ru = id('RightUpLeg');
  const hipY = spine[0].y, chestY = at('Spine02').y;
  for (const [i, ox, oz] of cape) {
    v.fromArray(P, i * 3);
    const up = THREE.MathUtils.smoothstep(v.y, hipY, chestY);
    const down = Math.min(1, Math.max(0, (hipY - v.y) / 0.6));
    const legW = 0.45 * down, side = THREE.MathUtils.smoothstep(v.x, -0.22, 0.22);
    SI.set([sp, hips, lu, ru], i * 4);
    SW.set([up, Math.max(0, 1 - up - legW), legW * side, legW * (1 - side)], i * 4);
    const push = 0.03 + 0.07 * down + 0.02 * (1 - up);
    P[i * 3] += ox * push; P[i * 3 + 2] += oz * push;
  }
  g.attributes.skinIndex.needsUpdate = g.attributes.skinWeight.needsUpdate = pos.needsUpdate = true;
  return cape.length;
};

/* 'remove' mode: drop every triangle with two or more cape corners, then drop the
   loose pieces that leaves behind the back (small islands mostly behind the spine). */
function removeTris(g, I, weld, isCape, P, backZ) {
  const keep = [];
  for (let t = 0; t < I.length; t += 3) {
    let n = 0;
    for (let k = 0; k < 3; k++) if (isCape.has(weld[I[t + k]])) n++;
    if (n < 2) keep.push(I[t], I[t + 1], I[t + 2]);
  }
  // islands over the kept triangles
  const par = new Map(), f = x => { while (par.get(x) !== x) { par.set(x, par.get(par.get(x))); x = par.get(x); } return x; };
  const u = (a, b) => { a = f(a); b = f(b); if (a !== b) par.set(a, b); };
  for (const i of keep) if (!par.has(weld[i])) par.set(weld[i], weld[i]);
  for (let t = 0; t < keep.length; t += 3) { u(weld[keep[t]], weld[keep[t + 1]]); u(weld[keep[t]], weld[keep[t + 2]]); }
  const size = new Map(), back = new Map();
  for (const w of par.keys()) { const r = f(w); size.set(r, (size.get(r) || 0) + 1); if (P[w * 3 + 2] < backZ - 0.02) back.set(r, (back.get(r) || 0) + 1); }
  let big = 0; for (const n of size.values()) big = Math.max(big, n);
  const out = [];
  for (let t = 0; t < keep.length; t += 3) {
    const r = f(weld[keep[t]]), n = size.get(r);
    if (n < big && (back.get(r) || 0) > n * 0.5) continue;
    out.push(keep[t], keep[t + 1], keep[t + 2]);
  }
  g.setIndex(out);
  return (I.length - out.length) / 3;
}
