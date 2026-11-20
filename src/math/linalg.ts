/**
 * Small dense linear-algebra helpers used by the visual-servoing laws.
 * Matrices are row-major number[][].
 */

export type Vec = number[];
export type Mat = number[][];

export function zeros(n: number): Vec {
  return Array.from({ length: n }, () => 0);
}

export function zerosMat(rows: number, cols: number): Mat {
  return Array.from({ length: rows }, () => zeros(cols));
}

export function eye(n: number): Mat {
  const m = zerosMat(n, n);
  for (let i = 0; i < n; i++) m[i][i] = 1;
  return m;
}

export function cloneVec(v: Vec): Vec {
  return v.slice();
}

export function cloneMat(a: Mat): Mat {
  return a.map((row) => row.slice());
}

export function add(a: Vec, b: Vec): Vec {
  return a.map((x, i) => x + b[i]);
}

export function sub(a: Vec, b: Vec): Vec {
  return a.map((x, i) => x - b[i]);
}

export function scale(a: Vec, s: number): Vec {
  return a.map((x) => x * s);
}

export function dot(a: Vec, b: Vec): number {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i] * b[i];
  return s;
}

export function norm(a: Vec): number {
  return Math.sqrt(dot(a, a));
}

export function normalize(a: Vec): Vec {
  const n = norm(a);
  if (n < 1e-15) return zeros(a.length);
  return scale(a, 1 / n);
}

export function cross(a: Vec, b: Vec): Vec {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

export function matVec(a: Mat, v: Vec): Vec {
  return a.map((row) => dot(row, v));
}

export function matMul(a: Mat, b: Mat): Mat {
  const n = a.length;
  const m = b[0].length;
  const p = b.length;
  const c = zerosMat(n, m);
  for (let i = 0; i < n; i++) {
    for (let k = 0; k < p; k++) {
      const aik = a[i][k];
      if (aik === 0) continue;
      for (let j = 0; j < m; j++) c[i][j] += aik * b[k][j];
    }
  }
  return c;
}

export function transpose(a: Mat): Mat {
  const r = a.length;
  const c = a[0].length;
  const t = zerosMat(c, r);
  for (let i = 0; i < r; i++) {
    for (let j = 0; j < c; j++) t[j][i] = a[i][j];
  }
  return t;
}

export function matScale(a: Mat, s: number): Mat {
  return a.map((row) => row.map((x) => x * s));
}

export function matAdd(a: Mat, b: Mat): Mat {
  return a.map((row, i) => row.map((x, j) => x + b[i][j]));
}

export function matSub(a: Mat, b: Mat): Mat {
  return a.map((row, i) => row.map((x, j) => x - b[i][j]));
}

/** Skew-symmetric matrix [v]_× such that [v]_× w = v × w. */
export function skew(v: Vec): Mat {
  return [
    [0, -v[2], v[1]],
    [v[2], 0, -v[0]],
    [-v[1], v[0], 0],
  ];
}

/** Inverse of vee: extract the vector from a skew-symmetric 3×3 matrix. */
export function vee(s: Mat): Vec {
  return [s[2][1], s[0][2], s[1][0]];
}

/**
 * Invert a square matrix with Gaussian elimination and partial pivoting.
 * Throws if the matrix is (near) singular.
 */
export function invert(a: Mat): Mat {
  const n = a.length;
  const aug = a.map((row, i) => {
    const r = row.slice();
    for (let j = 0; j < n; j++) r.push(i === j ? 1 : 0);
    return r;
  });

  for (let col = 0; col < n; col++) {
    let pivot = col;
    let best = Math.abs(aug[col][col]);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(aug[r][col]);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }
    if (best < 1e-12) {
      throw new Error("Matrix is singular or ill-conditioned");
    }
    if (pivot !== col) {
      const tmp = aug[col];
      aug[col] = aug[pivot];
      aug[pivot] = tmp;
    }
    const diag = aug[col][col];
    for (let j = 0; j < 2 * n; j++) aug[col][j] /= diag;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = aug[r][col];
      if (f === 0) continue;
      for (let j = 0; j < 2 * n; j++) aug[r][j] -= f * aug[col][j];
    }
  }

  return aug.map((row) => row.slice(n));
}

/**
 * Moore–Penrose pseudo-inverse for a tall full-column-rank matrix
 * (used for the 8×6 stacked interaction matrix): L⁺ = (Lᵀ L)⁻¹ Lᵀ.
 * Falls back to the wide form Lᵀ (L Lᵀ)⁻¹ when there are fewer rows than columns.
 */
export function pseudoInverse(l: Mat): Mat {
  const rows = l.length;
  const cols = l[0].length;
  const lt = transpose(l);
  if (rows >= cols) {
    return matMul(invert(matMul(lt, l)), lt);
  }
  return matMul(lt, invert(matMul(l, lt)));
}

export function almostEqual(a: number, b: number, eps = 1e-9): boolean {
  return Math.abs(a - b) <= eps;
}

export function vecAlmostEqual(a: Vec, b: Vec, eps = 1e-9): boolean {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) {
    if (!almostEqual(a[i], b[i], eps)) return false;
  }
  return true;
}

export function matAlmostEqual(a: Mat, b: Mat, eps = 1e-9): boolean {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) {
    if (!vecAlmostEqual(a[i], b[i], eps)) return false;
  }
  return true;
}
