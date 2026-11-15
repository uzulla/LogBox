#!/usr/bin/env node
// public/heading.js の確認: 直線とカーブのある合成コースで、ジャイロにバイアス・感度誤差・ノイズを足しても
// スリップアングルを 1° 程度で取り戻せるか。`node test-heading.js`
'use strict';
const assert = require('assert');
const H = require('./public/heading.js');

function rng(seed) { return () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 2 ** 32 * 2 - 1; }; }

// 長さ L の直線 2 本と半径 R の半円 2 つのコース。直線は速く、カーブは遅く、カーブの途中で車体が内側を向く(スリップ)
function synth({ bias = 0.5, scale = 1.02, noise = 0.3, headNoise = 0.3, seconds = 150, slipDeg = 8 } = {}) {
  const HZ = 25, L = 200, R = 60, lap = 2 * L + 2 * Math.PI * R;
  const rnd = rng(42);
  const n = HZ * seconds;
  const d = { n, t: new Float64Array(n), valid: new Uint8Array(n), speed_kmh: new Float64Array(n), heading_deg: new Float64Array(n),
    head_acc: new Float64Array(n).fill(0.5), rate: new Float64Array(n), trueSlip: new Float64Array(n) };
  let s = 0, prevBody = 0;
  for (let i = 0; i < n; i++) {
    const t = i / HZ;
    const u = s % lap;
    let course, inCurve, curveFrac = 0; // 進行方向(北から時計回り)
    if (u < L) { course = 0; inCurve = false; } // 北向きの直線
    else if (u < L + Math.PI * R) { curveFrac = (u - L) / (Math.PI * R); course = 180 * curveFrac; inCurve = true; } // 右カーブ
    else if (u < 2 * L + Math.PI * R) { course = 180; inCurve = false; }
    else { curveFrac = (u - 2 * L - Math.PI * R) / (Math.PI * R); course = 180 + 180 * curveFrac; inCurve = true; }
    const v = Math.min(1, t / 3) * (inCurve ? 14 : 25); // m/s(最初は発進)
    const slip = inCurve ? slipDeg * Math.sin(Math.PI * curveFrac) : 0; // 右カーブで車体が右(内側)を向く = +
    const body = course + slip;
    d.t[i] = t; d.valid[i] = 1; d.speed_kmh[i] = v * 3.6;
    d.heading_deg[i] = ((course + headNoise * rnd()) % 360 + 360) % 360;
    d.trueSlip[i] = slip;
    const trueRate = i ? H.wrap180(body - prevBody) * HZ : 0;
    d.rate[i] = trueRate * scale + bias + noise * rnd();
    prevBody = body;
    s += v / HZ;
  }
  return d;
}

function check(name, opts, maxRms, maxAbs) {
  const d = synth(opts);
  const r = H.estimate(d);
  assert.ok(r.ok, `${name}: not estimated`);
  let sum = 0, cnt = 0, worst = 0;
  for (let i = 0; i < d.n; i++) {
    if (!isFinite(r.slip[i]) || d.t[i] < 10) continue;
    const e = r.slip[i] - d.trueSlip[i];
    sum += e * e; cnt++; worst = Math.max(worst, Math.abs(e));
  }
  const rms = Math.sqrt(sum / cnt);
  console.log(`${name}: knots ${r.knots}, rows ${cnt}, slip error rms ${rms.toFixed(2)}°, max ${worst.toFixed(2)}°`);
  assert.ok(cnt > d.n * 0.8, `${name}: too few rows`);
  assert.ok(rms < maxRms && worst < maxAbs, `${name}: error too large`);
}

check('clean', { bias: 0, scale: 1, noise: 0, headNoise: 0 }, 0.2, 0.5);
check('bias 0.5 deg/s, scale +2%, noise', {}, 1.0, 2.5);
check('bias -1 deg/s, scale -3%, noise', { bias: -1, scale: 0.97 }, 1.2, 3.0);
// 途中で機器が車内で 90° 回った: その前後は推定でき、回った付近は出さない
{
  const d = synth({});
  const tFlip = 75;
  for (let i = 0; i < d.n; i++) if (Math.abs(d.t[i] - tFlip) < 0.02) d.rate[i] += 90 * 25; // 1 行で 90° 回る
  const r = H.estimate(d);
  assert.ok(r.breaks.length >= 1 && r.breaks.some((t) => Math.abs(t - tFlip) < 15), 'flip not detected');
  let sum = 0, cnt = 0;
  for (let i = 0; i < d.n; i++) if (isFinite(r.slip[i]) && d.t[i] >= 10) { const e = r.slip[i] - d.trueSlip[i]; sum += e * e; cnt++; }
  const rms = Math.sqrt(sum / cnt);
  console.log(`mount turned 90 deg at ${tFlip} s: breaks ${r.breaks.map((t) => t.toFixed(0)).join(',')}, rows ${cnt}, rms ${rms.toFixed(2)}°`);
  assert.ok(rms < 1.0 && cnt > d.n * 0.7, 'error around the flip');
}
// ジャイロの向き(符号)が逆なら推定は成り立たない: 呼び出し側(app.js)が GNSS と比べて符号を決める前提
console.log('OK');
