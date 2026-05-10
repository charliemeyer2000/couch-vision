from __future__ import annotations

from dataclasses import dataclass
from typing import Any

BERKELEY_MONO_FONT = "Berkeley Mono"


@dataclass(frozen=True, slots=True)
class MidnightPalette:
    foreground: str = "#121212"
    red: str = "#ff6b6b"
    green: str = "#98c379"
    yellow: str = "#e5c07b"
    blue: str = "#7aa2f7"
    magenta: str = "#c678dd"
    cyan: str = "#56b6c2"
    orange: str = "#e5a56b"
    background: str = "#e0e0e0"
    bright_black: str = "#666666"
    bright_red: str = "#f48771"
    bright_green: str = "#b5e890"
    bright_yellow: str = "#f0d197"
    bright_blue: str = "#9db8f7"
    bright_magenta: str = "#e298ff"
    bright_cyan: str = "#7dd6e0"
    bright_white: str = "#ffffff"
    light_black: str = "#666666"
    grey: str = "#3d3d3d"
    med_grey: str = "#2d2d2d"
    dark_grey: str = "#222222"
    medium_emphasis: str = "#999999"
    diff_add: str = "#0c2f1e"
    diff_delete: str = "#291f27"
    diff_change: str = "#3a4a6d"


MIDNIGHT_PALETTE = MidnightPalette()
DAYLIGHT_PALETTE = MidnightPalette(
    foreground="#f5f5f5",
    red="#c7254e",
    green="#2d7f3e",
    yellow="#996800",
    blue="#3b5bdb",
    magenta="#ae3ec9",
    cyan="#1098ad",
    orange="#d9730d",
    background="#1a1a1a",
    bright_black="#999999",
    bright_red="#e03e52",
    bright_green="#37b24d",
    bright_yellow="#f59f00",
    bright_blue="#4c6ef5",
    bright_magenta="#da77f2",
    bright_cyan="#15aabf",
    bright_white="#000000",
    light_black="#6b6b6b",
    grey="#d0d0d0",
    med_grey="#e8e8e8",
    dark_grey="#ebebeb",
    medium_emphasis="#666666",
    diff_add="#a5c5ab",
    diff_delete="#e2a1b2",
    diff_change="#a9b7e5",
)
STYLE_PALETTES = {
    "midnight": MIDNIGHT_PALETTE,
    "daylight": DAYLIGHT_PALETTE,
}
PALETTE = MIDNIGHT_PALETTE
FONT_FALLBACKS = (BERKELEY_MONO_FONT, "DejaVu Sans Mono", "DejaVu Sans")
CHART_BACKGROUND = PALETTE.foreground
CHART_TEXT = PALETTE.background
RELIABILITY_COLORS = {
    "reliable": PALETTE.green,
    "marginal": PALETTE.yellow,
    "unsafe": PALETTE.red,
    "insufficient_data": PALETTE.medium_emphasis,
    "no_data": PALETTE.medium_emphasis,
}
RELIABILITY_LABELS = {
    "reliable": "Reliable",
    "marginal": "Marginal",
    "unsafe": "Unsafe",
    "insufficient_data": "Insufficient data",
    "no_data": "No data",
}


def choose_font_family(font_names: set[str]) -> str:
    for family in FONT_FALLBACKS:
        if family in font_names:
            return family
    return FONT_FALLBACKS[-1]


def configure_matplotlib(plt: Any) -> None:
    import matplotlib.font_manager as font_manager

    font_family = choose_font_family(
        {font.name for font in font_manager.fontManager.ttflist}
    )
    plt.rcParams.update(
        {
            "figure.dpi": 160,
            "savefig.dpi": 160,
            "savefig.bbox": "tight",
            "figure.facecolor": CHART_BACKGROUND,
            "savefig.facecolor": CHART_BACKGROUND,
            "axes.facecolor": CHART_BACKGROUND,
            "axes.edgecolor": PALETTE.light_black,
            "axes.labelcolor": CHART_TEXT,
            "axes.titlecolor": CHART_TEXT,
            "xtick.color": CHART_TEXT,
            "ytick.color": CHART_TEXT,
            "text.color": CHART_TEXT,
            "font.family": font_family,
            "grid.color": PALETTE.grey,
            "grid.alpha": 0.45,
            "legend.facecolor": CHART_BACKGROUND,
            "legend.edgecolor": PALETTE.light_black,
            "legend.labelcolor": CHART_TEXT,
        }
    )


def chart_color(name: str) -> str:
    return getattr(PALETTE, name)


def reliability_color(band: str) -> str:
    return RELIABILITY_COLORS.get(band, PALETTE.medium_emphasis)


def reliability_label(band: str) -> str:
    return RELIABILITY_LABELS.get(band, band.replace("_", " ").title())


def report_band_label(band: str) -> str:
    return f"{reliability_label(band)} (`{reliability_color(band)}`)"
