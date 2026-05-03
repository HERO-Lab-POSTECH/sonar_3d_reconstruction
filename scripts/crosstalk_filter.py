#!/usr/bin/env python3
"""
2D FFT-based Crosstalk Noise Removal Filter for Multibeam Sonar

Removes horizontal stripe artifacts (crosstalk noise) from polar sonar images
by applying a notch filter in the 2D frequency domain along the fx~0 axis.
"""

import numpy as np


class CrosstalkFilter:
    """
    Removes horizontal stripe noise from sonar polar images using 2D FFT.

    Horizontal stripes appear at constant range across all bearings,
    concentrating energy along the fx~0 axis in 2D frequency domain.
    A notch filter suppresses this axis while preserving the DC component.
    """

    def __init__(self, enabled: bool, filter_width: float, filter_strength: float,
                 dc_preserve_ratio: float):
        self.enabled = enabled
        self.filter_width = filter_width
        self.filter_strength = filter_strength
        self.dc_preserve_ratio = dc_preserve_ratio

        # Mask cache (invalidated on shape/parameter change)
        self._cached_mask = None
        self._cached_shape = None

    def apply(self, polar_image: np.ndarray) -> np.ndarray:
        """Apply crosstalk stripe removal filter.

        Args:
            polar_image: uint8 sonar polar image (range_bins x bearing_bins)

        Returns:
            Filtered uint8 image
        """
        if not self.enabled:
            return polar_image

        if polar_image.shape[0] < 4 or polar_image.shape[1] < 4:
            return polar_image

        image_float = polar_image.astype(np.float32) / 255.0
        image_float = self._fft_stripe_removal(image_float)
        return np.clip(image_float * 255.0, 0, 255).astype(np.uint8)

    def _fft_stripe_removal(self, image_float: np.ndarray) -> np.ndarray:
        """2D FFT -> notch mask -> IFFT"""
        F = np.fft.fftshift(np.fft.fft2(image_float))
        mask = self._get_or_create_mask(image_float.shape)
        result = np.fft.ifft2(np.fft.ifftshift(F * mask)).real
        return result

    def _get_or_create_mask(self, shape: tuple) -> np.ndarray:
        """Return cached mask or create new one if shape changed."""
        if self._cached_mask is not None and self._cached_shape == shape:
            return self._cached_mask

        self._cached_mask = self._create_notch_mask(shape[0], shape[1])
        self._cached_shape = shape
        return self._cached_mask

    def _create_notch_mask(self, rows: int, cols: int) -> np.ndarray:
        """Create vectorized notch filter mask.

        Suppresses the fx~0 axis (horizontal stripes in spatial domain)
        while preserving DC component (fy~0, fx~0 intersection).

        Args:
            rows: Number of rows (range bins)
            cols: Number of columns (bearing bins)

        Returns:
            2D mask array with values in [0, 1]
        """
        fy = np.arange(rows, dtype=np.float32) - rows // 2
        fx = np.arange(cols, dtype=np.float32) - cols // 2

        FX, FY = np.meshgrid(fx, fy)

        # Normalized fx distance from center (0 to 1)
        fx_norm = np.abs(FX) / max(cols / 2, 1)

        # fx~0 suppression profile (Gaussian centered at fx=0)
        fx_profile = np.exp(-(fx_norm ** 2) / (2 * self.filter_width ** 2))

        # DC preservation: keep fy~0 region (Gaussian hole around DC)
        dc_radius = max(2.0, self.dc_preserve_ratio * rows / 2)
        fy_abs = np.abs(FY)
        fy_profile = 1.0 - np.exp(-(fy_abs ** 2) / (2 * dc_radius ** 2))

        # Combined suppression
        # fx_profile Gaussian already provides smooth transition (no sharp edges)
        suppression = fx_profile * fy_profile * self.filter_strength

        return np.clip(1.0 - suppression, 0.0, 1.0).astype(np.float32)

    def _invalidate_cache(self):
        """Invalidate mask cache (call when parameters change)."""
        self._cached_mask = None
        self._cached_shape = None

    # === Dynamic parameter update handlers ===

    def update_crosstalk_enabled(self, value: bool):
        self.enabled = bool(value)

    def update_crosstalk_filter_width(self, value: float):
        self.filter_width = float(value)
        self._invalidate_cache()

    def update_crosstalk_filter_strength(self, value: float):
        self.filter_strength = float(value)
        self._invalidate_cache()

    def update_crosstalk_dc_preserve_ratio(self, value: float):
        self.dc_preserve_ratio = float(value)
        self._invalidate_cache()

