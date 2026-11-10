"""policy_np.py -- the route-following driver's policy in plain numpy (the Jetson runs this: its
container has numpy but no onnxruntime). Written by rl2/export.py; this file is copied to the
Jetson as nav/nodes/rl2_policy.py and must stay import-free beyond numpy.

npz: mean, std, eps (state normaliser); conv{i}_W (out, in, k, k), conv{i}_b, conv{i}_stride,
conv{i}_pad, conv{i}_pool (MaxPool 3/2/1 after the activation); mlp{i}_W, mlp{i}_b; ELU everywhere
but the last layer; latent = [normalised state, flattened CNN (C, H, W order)].
"""
import json

import numpy as np


def _elu(x):
    return np.where(x > 0, x, np.expm1(np.minimum(x, 0)))


def _conv(x, W, b, stride, pad):
    """x (C, H, W) -> (O, H', W'), zero padding, im2col."""
    C, H, Wd = x.shape
    O, _, k, _ = W.shape
    ph, pw = pad
    xp = np.pad(x, ((0, 0), (ph, ph), (pw, pw)))
    Ho = (H + 2 * ph - k) // stride + 1
    Wo = (Wd + 2 * pw - k) // stride + 1
    s0, s1, s2 = xp.strides
    cols = np.lib.stride_tricks.as_strided(xp, shape=(C, k, k, Ho, Wo),
                                           strides=(s0, s1, s2, s1 * stride, s2 * stride))
    out = W.reshape(O, -1) @ cols.reshape(C * k * k, Ho * Wo)
    return out.reshape(O, Ho, Wo) + b[:, None, None]


def _maxpool(x):
    """MaxPool2d(3, stride 2, padding 1)."""
    C, H, W = x.shape
    xp = np.pad(x, ((0, 0), (1, 1), (1, 1)), constant_values=-np.inf)
    Ho, Wo = (H + 2 - 3) // 2 + 1, (W + 2 - 3) // 2 + 1
    s0, s1, s2 = xp.strides
    win = np.lib.stride_tricks.as_strided(xp, shape=(C, Ho, Wo, 3, 3), strides=(s0, s1 * 2, s2 * 2, s1, s2))
    return win.max(axis=(3, 4))


class Policy:
    def __init__(self, path):
        d = np.load(path)
        self.mean, self.std, self.eps = d['mean'], d['std'], float(d['eps'])
        self.conv = []
        i = 0
        while f'conv{i}_W' in d:
            self.conv.append((d[f'conv{i}_W'], d[f'conv{i}_b'], int(d[f'conv{i}_stride']),
                              tuple(int(v) for v in d[f'conv{i}_pad']), bool(d[f'conv{i}_pool'])))
            i += 1
        self.mlp = []
        i = 0
        while f'mlp{i}_W' in d:
            self.mlp.append((d[f'mlp{i}_W'], d[f'mlp{i}_b']))
            i += 1
        self.meta = json.loads(str(d['meta'])) if 'meta' in d else {}
        self.state_dim = int(d['state_dim'])
        self.patch_shape = tuple(int(v) for v in d['patch_shape'])

    def __call__(self, state, patch):
        """state (STATE_DIM,), patch (C, 64, 64) -> action (2,) in [-1, 1] after the caller's clip."""
        s = (np.asarray(state, np.float32) - self.mean) / (self.std + self.eps)
        x = np.asarray(patch, np.float32)
        for W, b, st, pad, pool in self.conv:
            x = _elu(_conv(x, W, b, st, pad))
            if pool:
                x = _maxpool(x)
        h = np.concatenate([s, x.reshape(-1)]).astype(np.float32)
        for k, (W, b) in enumerate(self.mlp):
            h = h @ W.T + b
            if k < len(self.mlp) - 1:
                h = _elu(h)
        return h
