#pragma once
#include <QString>

struct ThemeData {
    const char *bgPrimary;
    const char *bgSurface;
    const char *bgControl;
    const char *bgElevated;
    const char *border;
    const char *borderLight;
    const char *textPrimary;
    const char *textSecondary;
    const char *textMuted;
    const char *accentGreen;
    const char *accentGreenBg;
    const char *accentOrange;
    const char *accentRed;
    const char *btnOn;
    const char *btnOnBg;
    const char *btnOnBorder;
    const char *btnOff;
    const char *btnOffBg;
    const char *btnOffBorder;

    QString toStyleSheet() const;
};

extern const ThemeData themeDark;
extern const ThemeData themeLight;
extern const ThemeData *currentTheme;