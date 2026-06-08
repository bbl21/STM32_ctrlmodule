#include "routingpanel.h"
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QFrame>

RoutingPanel::RoutingPanel(QWidget *parent)
    : QGroupBox(tr("直通选择"), parent)
{
    QGridLayout *grid = new QGridLayout(this);
    grid->setSpacing(4);

    grid->addWidget(new QLabel(""), 0, 0);
    grid->addWidget(new QLabel(tr("直通")), 0, 1, Qt::AlignCenter);
    grid->addWidget(new QLabel(tr("截止")), 0, 2, Qt::AlignCenter);

    // B, C (module 1)
    m_directB = new QCheckBox();
    m_directC = new QCheckBox();
    m_directINV_B = new QCheckBox();
    m_directINV_C = new QCheckBox();

    // F, G (module 2)
    m_directF = new QCheckBox();
    m_directG = new QCheckBox();
    m_directINV_F = new QCheckBox();
    m_directINV_G = new QCheckBox();

    auto addRow = [&](int row, const QString &name, QCheckBox *zt, QCheckBox *cut) {
        grid->addWidget(new QLabel(name), row, 0);
        grid->addWidget(zt, row, 1, Qt::AlignCenter);
        grid->addWidget(cut, row, 2, Qt::AlignCenter);
        connect(zt, &QCheckBox::clicked, this, [this, zt, cut]() {
            if (zt->isChecked()) cut->setChecked(false);
            emit changed();
        });
        connect(cut, &QCheckBox::clicked, this, [this, zt, cut]() {
            if (cut->isChecked()) zt->setChecked(false);
            emit changed();
        });
    };

    addRow(1, "B", m_directB, m_directINV_B);
    addRow(2, "C", m_directC, m_directINV_C);

    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    grid->addWidget(line, 3, 0, 1, 3);

    addRow(4, "F", m_directF, m_directINV_F);
    addRow(5, "G", m_directG, m_directINV_G);

    // Multiple selection prevention
    m_directB->setChecked(false);
    m_directINV_B->setChecked(true);
    m_directC->setChecked(false);
    m_directINV_C->setChecked(true);
    m_directF->setChecked(false);
    m_directINV_F->setChecked(true);
    m_directG->setChecked(false);
    m_directINV_G->setChecked(true);
}

void RoutingPanel::applyTo(ControlWord &word) const {
    word.setDirectThrough('B', m_directB->isChecked());
    word.setDirectThrough('C', m_directC->isChecked());
    word.setDirectThrough('F', m_directF->isChecked());
    word.setDirectThrough('G', m_directG->isChecked());
}

void RoutingPanel::loadFrom(const ControlWord &word) {
    m_directB->setChecked(word.getDirectThrough('B'));
    m_directINV_B->setChecked(!word.getDirectThrough('B'));
    m_directC->setChecked(word.getDirectThrough('C'));
    m_directINV_C->setChecked(!word.getDirectThrough('C'));
    m_directF->setChecked(word.getDirectThrough('F'));
    m_directINV_F->setChecked(!word.getDirectThrough('F'));
    m_directG->setChecked(word.getDirectThrough('G'));
    m_directINV_G->setChecked(!word.getDirectThrough('G'));
}
