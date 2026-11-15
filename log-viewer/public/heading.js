'use strict';
// 車体の向き(ヨー角)とスリップアングルを、記録後にまとめて推定する。
//
// 車体の向き = ジャイロのヨー角速度の積分。積分はジャイロのバイアスや感度の誤差で少しずつずれるので、
// 「速度があって、まっすぐ走っている」区間(アンカー)では車体の向き ≈ 進行方向(GNSS の速度ベクトルの向き)とみなし、
// そこで積分を進行方向に合わせ直す。アンカーの間(カーブ)は、前後のアンカーでのずれを時間で按分して補正する
// (= その間のバイアスを一定とみなす)。記録後なので、カーブの後ろ側のアンカーも使えるのがその場で計算するより有利な点。
// 取り付けの向き(水平面内)のずれも、同じ補正に吸収される。
//
// 入力 d: {n, t(秒), valid(Uint8Array: 位置が使える行), speed_kmh, heading_deg(進行方向、北から時計回り),
//          head_acc(進行方向の精度 deg、無ければ null), rate(Float64Array: 車体の向きの変化 deg/s、時計回りが正), lr(左右の G、省略可)}
// 出力: {ok, body(車体の向き deg 0..360), slip(車体 − 進行方向 deg、+ = 車体が進行方向より右を向く), courseRate(deg/s),
//        bodyRate(deg/s、補正後), knots(アンカーの数), breaks(取り付けが変わったとみなした箇所の時刻 秒)}。推定できない行は NaN
(function (root) {
  const ANCHOR_SPEED = 20;      // km/h。これより速いときだけアンカーにする
  const ANCHOR_RATE = 2;        // deg/s。ジャイロ・進行方向の変化がこれ未満 = まっすぐ
  const ANCHOR_LR = 0.08;       // G。左右の加速度がこれ未満
  const ANCHOR_HEAD_ACC = 2;    // deg。進行方向の精度
  const KNOT_SEC = 1;           // 秒。アンカーをこの幅でまとめて 1 つの補正点にする
  const KNOT_MIN_ROWS = 5;      // 補正点にするのに必要なアンカーの行数
  const KNOT_OUTLIER = 8;       // deg。前後の補正点の中央値からこれ以上外れた補正点は捨てる
  // 隣の補正点との差が KNOT_BREAK + 経過秒 × KNOT_BREAK_RATE + 間に曲がった角度 × KNOT_BREAK_SCALE 以上
  // = ジャイロのバイアスや感度の誤差では説明できない = 機器が車内で回った(取り付けが変わった)とみなし、間を補間しない
  const KNOT_BREAK = 10, KNOT_BREAK_RATE = 1.5, KNOT_BREAK_SCALE = 0.08;
  const SLIP_SPEED = 10;        // km/h。スリップアングルを出す最低速度(低速では進行方向が不正確)
  const SLIP_HEAD_ACC = 5;      // deg
  const MAX_FROM_KNOT = 60;     // 秒。最寄りの補正点からこれ以上離れた行は推定しない(積分のずれが大きくなる)
  const MAX_EXTRAPOLATE = 20;   // 秒。最初の補正点より前 / 最後の補正点より後は、この長さまで
  const SLOPE_WINDOW = 30;      // 秒。端の外側は、この範囲の補正点の傾き(= バイアス)で延長する
  const RATE_SPAN = 0.4;        // 秒。進行方向の変化率を出す差分の半幅

  const wrap180 = (x) => ((x % 360) + 540) % 360 - 180;
  const wrap360 = (x) => ((x % 360) + 360) % 360;
  const median = (a) => { const b = Float64Array.from(a).sort(); return b.length ? b[b.length >> 1] : NaN; };

  // 進行方向の変化率(deg/s、時計回りが正)。中心差分、両端が有効で速度があるときだけ
  function courseRate(d) {
    const out = new Float64Array(d.n).fill(NaN);
    for (let i = 0, a = 0, b = 0; i < d.n; i++) {
      while (d.t[i] - d.t[a] > RATE_SPAN) a++;
      while (b + 1 < d.n && d.t[b + 1] - d.t[i] <= RATE_SPAN) b++;
      const span = d.t[b] - d.t[a];
      if (span < RATE_SPAN || !d.valid[a] || !d.valid[b] || d.speed_kmh[a] < SLIP_SPEED || d.speed_kmh[b] < SLIP_SPEED) continue;
      out[i] = wrap180(d.heading_deg[b] - d.heading_deg[a]) / span;
    }
    return out;
  }

  function estimate(d) {
    const n = d.n;
    const res = { ok: false, body: new Float64Array(n).fill(NaN), slip: new Float64Array(n).fill(NaN), courseRate: courseRate(d),
      bodyRate: new Float64Array(n).fill(NaN), knots: 0, breaks: [] };
    // 1. 積分(角速度は各行までの区間の平均なので、区間の長さを掛けて足す)
    const psi = new Float64Array(n);
    for (let i = 1; i < n; i++) {
      const dt = d.t[i] - d.t[i - 1];
      const w = d.rate[i];
      psi[i] = psi[i - 1] + (isFinite(w) && dt > 0 && dt < 1 ? w * dt : 0);
    }
    // 2. アンカー: まっすぐ走っている行。そこでの (進行方向 − 積分) が補正量
    const headOk = (i, lim) => !d.head_acc || !isFinite(d.head_acc[i]) || d.head_acc[i] < lim;
    const knots = [];
    let bin = null;
    const flush = () => {
      if (bin && bin.err.length >= KNOT_MIN_ROWS) knots.push({ t: bin.tSum / bin.err.length, err: median(bin.err), psi: bin.psiSum / bin.err.length });
      bin = null;
    };
    for (let i = 0; i < n; i++) {
      const anchor = d.valid[i] && d.speed_kmh[i] >= ANCHOR_SPEED && headOk(i, ANCHOR_HEAD_ACC) &&
        Math.abs(d.rate[i]) < ANCHOR_RATE && Math.abs(res.courseRate[i]) < ANCHOR_RATE &&
        (!d.lr || !isFinite(d.lr[i]) || Math.abs(d.lr[i]) < ANCHOR_LR);
      if (!anchor) continue;
      if (bin && d.t[i] - bin.t0 >= KNOT_SEC) flush();
      if (!bin) bin = { t0: d.t[i], tSum: 0, psiSum: 0, err: [], ref: null };
      // 1 つの補正点の中では、最初の値の近くに畳む(±180° をまたいでも中央値が壊れないように)
      let e = wrap180(d.heading_deg[i] - psi[i]);
      if (bin.ref === null) bin.ref = e; else e = bin.ref + wrap180(e - bin.ref);
      bin.err.push(e); bin.tSum += d.t[i]; bin.psiSum += psi[i];
    }
    flush();
    if (!knots.length) return res;
    // 補正点どうしを連続につなぐ(前の点との差を ±180° に畳む)
    for (let k = 1; k < knots.length; k++) knots[k].err = knots[k - 1].err + wrap180(knots[k].err - knots[k - 1].err);
    // 外れ値(まっすぐに見えて実は横滑りしていた、GNSS が乱れた等)を除く。前後 2 点ずつとの中央値と比べる
    const kept = knots.filter((q, k) => {
      const nb = [];
      for (let j = Math.max(0, k - 2); j <= Math.min(knots.length - 1, k + 2); j++) if (j !== k) nb.push(knots[j].err);
      return nb.length < 2 || Math.abs(q.err - median(nb)) < KNOT_OUTLIER;
    });
    if (!kept.length) return res;
    res.knots = kept.length;
    // 隣どうしで大きく食い違う = その間に機器が車内で回った。バイアスでは説明できないので、その間は推定しない
    const brk = new Uint8Array(kept.length); // brk[k] = kept[k - 1] と kept[k] の間で途切れる
    for (let k = 1; k < kept.length; k++) {
      const a = kept[k - 1], b = kept[k];
      const allow = KNOT_BREAK + KNOT_BREAK_RATE * (b.t - a.t) + KNOT_BREAK_SCALE * Math.abs(b.psi - a.psi);
      if (Math.abs(b.err - a.err) >= allow) { brk[k] = 1; res.breaks.push((kept[k - 1].t + kept[k].t) / 2); }
    }
    // 3. 補正量を時間で線形補間。両端の外側は、端に近い補正点の傾き(バイアス)で延長する。最寄りの補正点から遠い行は出さない
    const slopeNear = (fromEnd) => {
      const t0 = fromEnd ? kept[kept.length - 1].t : kept[0].t;
      const q = kept.filter((p) => Math.abs(p.t - t0) <= SLOPE_WINDOW);
      if (q.length < 2) return 0;
      const mt = q.reduce((a, p) => a + p.t, 0) / q.length, me = q.reduce((a, p) => a + p.err, 0) / q.length;
      let sxy = 0, sxx = 0;
      for (const p of q) { sxy += (p.t - mt) * (p.err - me); sxx += (p.t - mt) ** 2; }
      return sxx > 25 ? sxy / sxx : 0; // 範囲が 5 秒程度ないと傾きは当てにならない
    };
    const slope0 = slopeNear(false), slope1 = slopeNear(true);
    let k = 0;
    for (let i = 0; i < n; i++) {
      while (k + 1 < kept.length && kept[k + 1].t <= d.t[i]) k++;
      let corr, dist;
      if (d.t[i] <= kept[0].t) {
        dist = kept[0].t - d.t[i];
        if (dist > MAX_EXTRAPOLATE) continue;
        corr = kept[0].err - slope0 * dist;
      } else if (k + 1 >= kept.length) {
        dist = d.t[i] - kept[k].t;
        if (dist > MAX_EXTRAPOLATE) continue;
        corr = kept[k].err + slope1 * dist;
      }
      else {
        if (brk[k + 1]) continue;
        const a = kept[k], b = kept[k + 1], f = (d.t[i] - a.t) / (b.t - a.t);
        corr = a.err + (b.err - a.err) * f;
        dist = Math.min(d.t[i] - a.t, b.t - d.t[i]);
      }
      if (dist > MAX_FROM_KNOT) continue;
      res.body[i] = wrap360(psi[i] + corr);
      if (i > 0 && isFinite(res.body[i - 1]) && d.t[i] > d.t[i - 1]) res.bodyRate[i] = wrap180(res.body[i] - res.body[i - 1]) / (d.t[i] - d.t[i - 1]);
      if (d.valid[i] && d.speed_kmh[i] >= SLIP_SPEED && headOk(i, SLIP_HEAD_ACC)) res.slip[i] = wrap180(res.body[i] - d.heading_deg[i]);
    }
    res.ok = true;
    return res;
  }

  const api = { estimate, courseRate, wrap180 };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.Heading = api;
})(typeof self !== 'undefined' ? self : this);
