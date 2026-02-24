package applepose

import "math"

// hsvColor represents a color in HSV space.
type hsvColor struct {
	H float64 // Hue in degrees [0, 360)
	S float64 // Saturation [0, 1]
	V float64 // Value [0, 1]
}

// colorClass categorizes a point's color.
type colorClass int

const (
	colorAppleSkin colorClass = iota
	colorStemLike
	colorCalyxLike
	colorUnknown
)

// rgbToHSV converts RGB (0-255) to HSV.
func rgbToHSV(r, g, b uint8) hsvColor {
	rf := float64(r) / 255.0
	gf := float64(g) / 255.0
	bf := float64(b) / 255.0

	maxC := math.Max(rf, math.Max(gf, bf))
	minC := math.Min(rf, math.Min(gf, bf))
	delta := maxC - minC

	var h float64
	switch {
	case delta == 0:
		h = 0
	case maxC == rf:
		h = 60 * math.Mod((gf-bf)/delta, 6)
	case maxC == gf:
		h = 60 * ((bf-rf)/delta + 2)
	case maxC == bf:
		h = 60 * ((rf-gf)/delta + 4)
	}
	if h < 0 {
		h += 360
	}

	var s float64
	if maxC > 0 {
		s = delta / maxC
	}

	return hsvColor{H: h, S: s, V: maxC}
}

// isAppleColored returns true if the given RGB color is within the apple color range
// (red through green hues with sufficient saturation and brightness).
// Returns true if the color filter is disabled (AppleHueMax <= 0).
func isAppleColored(r, g, b uint8, cfg SceneConfig) bool {
	if cfg.AppleHueMax <= 0 {
		return true
	}
	hsv := rgbToHSV(r, g, b)
	if hsv.S < cfg.AppleMinSaturation || hsv.V < cfg.AppleMinValue {
		return false
	}
	if hsv.H <= cfg.AppleHueMax {
		return true
	}
	if cfg.AppleHueRedWrapMin > 0 && hsv.H >= cfg.AppleHueRedWrapMin {
		return true
	}
	return false
}

// classifyColor determines whether a point's color is apple skin, stem-like, or calyx-like.
func classifyColor(r, g, b uint8, cfg FeatureConfig) colorClass {
	hsv := rgbToHSV(r, g, b)

	// Very dark points are ambiguous.
	if hsv.V < 0.15 {
		return colorUnknown
	}

	// Stem-like: brownish/greenish hues with moderate saturation.
	// Hue range ~20-90 degrees (yellow-brown-green).
	if hsv.H >= cfg.StemHueMin && hsv.H <= cfg.StemHueMax && hsv.S <= cfg.StemMaxSaturation {
		return colorStemLike
	}

	// Calyx-like: similar to stem but can also be darker/greener.
	// Often indistinguishable from stem by color alone; geometry disambiguates.
	if hsv.H >= cfg.StemHueMin && hsv.H <= cfg.StemHueMax+20 && hsv.S <= cfg.StemMaxSaturation {
		return colorCalyxLike
	}

	// Reddish/yellowish apple skin.
	if (hsv.H < cfg.StemHueMin || hsv.H > cfg.StemHueMax+20) && hsv.S > 0.2 {
		return colorAppleSkin
	}

	return colorUnknown
}

// stemLikeFraction computes the fraction of points in a set that are stem/calyx-like in color.
func stemLikeFraction(colors []colorClass) float64 {
	if len(colors) == 0 {
		return 0
	}
	count := 0
	for _, c := range colors {
		if c == colorStemLike || c == colorCalyxLike {
			count++
		}
	}
	return float64(count) / float64(len(colors))
}
