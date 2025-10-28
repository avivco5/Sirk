# ===== models/FCM.py =====
# ASCII-only; English comments
# Spatial intuitionistic Fuzzy C-means with dimensionality and dtype fixes.

import numpy as np
from scipy import signal
import cv2

def getGaussianElement(a, b, stda, stdb, quad=None):
    # Create a 2d gaussian weight kernel, normalized to [0,1]
    assert a > 0 and int(a) == a
    assert b > 0 and int(b) == b
    assert stda > 0 and stdb > 0
    assert quad in [None, "ne", "nw", "sw", "se"]

    ax = np.arange(-a, a + 1)
    bx = np.arange(-b, b + 1)
    x, y = np.meshgrid(ax, bx)
    ellipse = 0.5 * np.pi / stda / stdb * np.exp(-x ** 2 / (2.0 * stda ** 2) - y ** 2 / (2.0 * stdb ** 2))

    if quad is not None:
        if quad == "ne":
            ellipse = np.where((ellipse > 0) & (x >= 0) & (y >= 0), ellipse, 0)
        elif quad == "nw":
            ellipse = np.where((ellipse > 0) & (x <= 0) & (y >= 0), ellipse, 0)
        elif quad == "sw":
            ellipse = np.where((ellipse > 0) & (x <= 0) & (y <= 0), ellipse, 0)
        elif quad == "se":
            ellipse = np.where((ellipse > 0) & (x >= 0) & (y <= 0), ellipse, 0)

    m = np.max(ellipse)
    ellipse = ellipse / m if m > 0 else ellipse
    return ellipse.astype(np.float32)

class FCM:
    def __init__(self, image, n_clusters, m=2, kernel_size=5,
                 kernel_shape="uniform", lam=0.5, epsilon=0.05, max_iter=300):
        # input checks
        if np.ndim(image) != 2:
            raise Exception("<image> must be 2D grayscale.")
        if n_clusters <= 0 or n_clusters != int(n_clusters):
            raise Exception("<n_clusters> must be a positive integer.")
        if m < 1:
            raise Exception("<m> must be >= 1.")
        if kernel_size <= 0 or kernel_size != int(kernel_size):
            raise Exception("<kernel_size> must be a positive integer.")
        if kernel_shape not in ["uniform", "gaussian"]:
            raise Exception("<kernel_shape> must be 'uniform' or 'gaussian'.")
        if lam <= 0:
            raise Exception("<lam> must be > 0.")
        if epsilon <= 0:
            raise Exception("<epsilon> must be > 0.")

        # store
        self.image = image.astype(np.float32)
        self.n_clusters = int(n_clusters)
        self.m = float(m)
        self.kernel_size = int(kernel_size)
        self.kernel_shape = kernel_shape
        self.lam = float(lam)
        self.epsilon = float(epsilon)
        self.max_iter = int(max_iter)

        self.shape = self.image.shape
        self.X = self.image.flatten().astype(np.float32)
        self.numPixels = self.image.size

        # initial U (one-hot stripes)
        self.U = np.zeros((self.numPixels, self.n_clusters), dtype=np.float32)
        idx = np.arange(self.numPixels)
        for ii in range(self.n_clusters):
            self.U[idx % self.n_clusters == ii, ii] = 1.0

        # initial centers
        vmin, vmax = float(np.min(self.image)), float(np.max(self.image))
        if vmax <= vmin:
            vmax = vmin + 1.0
        self.C = np.linspace(vmin, vmax, self.n_clusters, dtype=np.float32).reshape(self.n_clusters, 1)

        # neighborhood kernel (ensure 3D to match (H,W,C))
        if kernel_shape == "uniform":
            self.kernel = np.ones((self.kernel_size, self.kernel_size, 1), dtype=np.float32)
        else:
            radius = max(1, int(self.kernel_size / 2))
            stdr = max(1, int(radius / 2))
            g2d = getGaussianElement(radius, radius, stdr, stdr)
            self.kernel = g2d[:, :, None].astype(np.float32)

        # initial hesitation
        self.hesitation = 1.0 - self.U - (1.0 - self.U) / (1.0 + 2.0 * self.U + 1e-8)

    def update_U(self):
        # classic FCM membership update with epsilon guards
        c_mesh, x_mesh = np.meshgrid(self.C.flatten(), self.X)
        power = 2.0 / max(1e-6, (self.m - 1.0))
        diff = np.abs(x_mesh - c_mesh) + 1e-8
        p1 = diff ** power
        inv = 1.0 / diff
        p2 = np.sum(inv ** power, axis=1, keepdims=True)  # (N,1)
        U_new = 1.0 / (p1 * p2)
        # normalize rows to avoid drift
        U_new = U_new / (np.sum(U_new, axis=1, keepdims=True) + 1e-8)
        return U_new.astype(np.float32)

    def update_C(self):
        num = np.dot(self.X, (self.U ** self.m))
        den = np.sum(self.U ** self.m, axis=0) + 1e-8
        C = (num / den).astype(np.float32)
        return C

    def calculate_h(self):
        # compute spatial prior by 3D convolution over (H,W,C)
        uu = self.U.reshape(self.shape + (self.n_clusters,)).astype(np.float32)
        ker = self.kernel.astype(np.float32)
        # ensure same dimensionality
        if ker.ndim == 2:
            ker = ker[:, :, None]
        h = signal.fftconvolve(uu, ker, mode="same")
        return h.reshape(self.U.shape).astype(np.float32)

    def compute_intuitionistic_U(self):
        self.hesitation = 1.0 - self.U - (1.0 - self.U) / (1.0 + self.lam * self.U + 1e-8)
        return (self.U + self.hesitation).astype(np.float32)

    def computeNew_U(self):
        p = 1.0
        q = 3.0
        self.h = self.calculate_h()
        num = (self.U ** p) * (self.h ** q)
        denom = np.sum(num, axis=1, keepdims=True) + 1e-8
        U_new = num / denom
        return U_new.astype(np.float32)

    def computeIntraDists(self):
        result = self.deFuzzify()
        c_mesh, x_mesh = np.meshgrid(self.C.flatten(), self.X)
        dist = np.abs(c_mesh - x_mesh)
        idx = np.arange(self.n_clusters)
        match = result[:, None] - idx[None, :]
        counts = np.maximum(1, np.bincount(result, minlength=self.n_clusters)).astype(np.float32)
        sigma = (dist * (match == 0)).sum(axis=0) / counts
        return sigma

    def calculate_DB_score(self):
        sigmas = self.computeIntraDists()
        rs = sigmas[:, None] + sigmas[None, :]
        dists = np.abs(self.C[:, 0][:, None] - self.C[:, 0][None, :]) + 1e-8
        np.fill_diagonal(dists, np.nan)
        rs = rs / dists
        ds = np.nanmax(rs, axis=0)
        _ = ds.sum() / max(1, self.n_clusters)

    def calculate_D_score(self):
        sigmas = self.computeIntraDists()
        denom = max(1e-8, float(np.max(sigmas)))
        dists = np.abs(self.C[:, 0][:, None] - self.C[:, 0][None, :]) / denom
        np.fill_diagonal(dists, np.inf)
        _ = np.min(dists)

    def calculate_scores(self):
        self.vpc = float((self.U ** 2).sum() / max(1, self.numPixels))
        vpe = self.U * np.log(self.U + 1e-8)
        self.vpe = float(-1.0 * vpe.sum() / max(1, self.numPixels))
        c_mesh, x_mesh = np.meshgrid(self.C.flatten(), self.X)
        numer = float((self.U * (x_mesh - c_mesh) ** 2).sum())
        denom = float(self.numPixels) * float((self.C[0] - self.C[min(1, self.n_clusters - 1)]) ** 2 + 1e-8)
        self.vxb = numer / denom
        self.calculate_DB_score()
        self.calculate_D_score()

    def form_clusters(self):
        d = 1e9
        i = 0
        while True:
            self.C = self.update_C().reshape(self.n_clusters, 1)
            old_u = np.copy(self.U)
            self.U = self.update_U()
            self.U = self.compute_intuitionistic_U()
            self.U = self.computeNew_U()
            d = float(np.sum(np.abs(self.U - old_u)))
            if d < self.epsilon or i >= self.max_iter:
                break
            i += 1
        self.segmentImage()

    def deFuzzify(self):
        return np.argmax(self.U, axis=1).astype(np.int32)

    def segmentImage(self):
        result = self.deFuzzify()
        self.result = result.reshape(self.shape).astype(np.int32)
        return self.result

def postFCM(result):
    """Normalize to uint8 single-channel and apply median blur."""
    try:
        if result.dtype != np.uint8:
            result = cv2.normalize(result, None, 0, 255, cv2.NORM_MINMAX)
            result = result.astype(np.uint8)
        if result.ndim == 3 and result.shape[2] > 1:
            result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        result = cv2.medianBlur(result, 5)
    except Exception as e:
        print("[WARN] postFCM failed:", e)
        thr = np.mean(result)
        result = (result > thr).astype(np.uint8) * 255
    return result

def fcm_bbox_correction(bbox, frame):
    # Simple boundary checks for bbox = [x,y,w,h]
    H, W = frame.shape[:2]
    x, y, w, h = bbox
    x = max(0.0, min(float(x), W - 2.0))
    y = max(0.0, min(float(y), H - 2.0))
    w = max(2.0, min(float(w), W - x))
    h = max(2.0, min(float(h), H - y))
    return [x, y, w, h]
