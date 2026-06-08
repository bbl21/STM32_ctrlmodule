#ifndef ANTENNASTATUSPANEL_H
#define ANTENNASTATUSPANEL_H

#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <array>
#include "controlword.h"

class AntennaStatusPanel : public QGroupBox {
    Q_OBJECT
public:
    explicit AntennaStatusPanel(QWidget *parent = nullptr);
    void applyTo(ControlWord &word) const;
    void loadFrom(const ControlWord &word);
    void setTheme();
signals:
    void changed();
private:
    struct AntBlock {
        QPushButton *btnV;
        QPushButton *btnH;
        QLabel *name;
        QPushButton *btnZT;
    };
    std::array<AntBlock, 8> m_ant;
    std::array<bool, 8> m_vOn{}, m_hOn{}, m_ztOn{};

    QPushButton *m_allV, *m_allH, *m_allZT;
};

#endif
