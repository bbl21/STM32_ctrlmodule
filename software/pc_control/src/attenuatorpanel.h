#ifndef ATTENUATORPANEL_H
#define ATTENUATORPANEL_H

#include <QGroupBox>
#include <QSlider>
#include <QSpinBox>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QScrollArea>
#include <array>
#include "controlword.h"

class AttenuatorPanel : public QGroupBox {
    Q_OBJECT
public:
    explicit AttenuatorPanel(QWidget *parent = nullptr);
    void applyTo(ControlWord &word) const;
    void loadFrom(const ControlWord &word);
    void setTheme();
signals:
    void changed();
private:
    struct IFChan {
        QSlider *slider;
        QSpinBox *spin;
        QPushButton *btnWide;
        QPushButton *btnNarrow;
    };
    std::array<IFChan, 9> m_ch; // 0-7: A-H, 8: reference
    QComboBox *m_refChannel = nullptr;
    QPushButton *m_allWide;
    QPushButton *m_allNarrow;
    QSlider *m_globalSlider;
    QSpinBox *m_globalSpin;
    void syncGlobalFromChannels();
};

#endif
