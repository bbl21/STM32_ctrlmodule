#ifndef ANTENNAPANEL_H
#define ANTENNAPANEL_H

#include <QGroupBox>
#include <QCheckBox>
#include <array>
#include "controlword.h"

class AntennaPanel : public QGroupBox {
    Q_OBJECT
public:
    explicit AntennaPanel(QWidget *parent = nullptr);

    void applyTo(ControlWord &word) const;
    void loadFrom(const ControlWord &word);

signals:
    void changed();

private:
    std::array<QCheckBox*, 8> m_chkV;
    std::array<QCheckBox*, 8> m_chkH;
};

#endif
