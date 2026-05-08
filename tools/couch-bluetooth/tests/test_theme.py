from couch_range.theme import (
    BERKELEY_MONO_FONT,
    CHART_BACKGROUND,
    CHART_TEXT,
    MIDNIGHT_PALETTE,
    STYLE_PALETTES,
    choose_font_family,
    report_band_label,
)


def test_midnight_palette_matches_source_style_names() -> None:
    assert STYLE_PALETTES["midnight"].blue == "#7aa2f7"
    assert STYLE_PALETTES["daylight"].blue == "#3b5bdb"
    assert MIDNIGHT_PALETTE.foreground == "#121212"
    assert MIDNIGHT_PALETTE.background == "#e0e0e0"


def test_chart_colors_follow_midnight_normal_semantics() -> None:
    assert CHART_BACKGROUND == MIDNIGHT_PALETTE.foreground
    assert CHART_TEXT == MIDNIGHT_PALETTE.background


def test_font_prefers_berkeley_mono() -> None:
    font = choose_font_family({"DejaVu Sans", BERKELEY_MONO_FONT})

    assert font == BERKELEY_MONO_FONT


def test_report_band_label_includes_palette_color() -> None:
    assert report_band_label("reliable") == "Reliable (`#98c379`)"
