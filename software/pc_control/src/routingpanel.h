#ifndef ROUTINGPANEL_H
#define ROUTINGPANEL_H

#include <QGroupBox>
#include <QCheckBox>
#include "controlword.h"

class RoutingPanel : public QGroupBox {
    Q_OBJECT
public:
    explicit RoutingPanel(QWidget *parent = nullptr);

    void applyTo(ControlWord &word) const;
    void loadFrom(const ControlWord &word);

signals:
    void changed();

private:
    QCheckBox *m_directB, *m_directINV_B;
    QCheckBox *m_directC, *m_directINV_C;
    QCheckBox *m_directF, *m_directINV_F;
    QCheckBox *m_directG, *m_directINV_G;
};

#endif
