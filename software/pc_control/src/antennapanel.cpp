#include "antennapanel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFrame>

AntennaPanel::AntennaPanel(QWidget *parent)
    : QGroupBox(tr("天线极化选择"), parent)
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // Vertical axis: A, B, C, D
    QHBoxLayout *vLayout = new QHBoxLayout();
    vLayout->addStretch();
    QVBoxLayout *vCol = new QVBoxLayout();
    for (int i = 0; i < 4; i++) {
        int idx = i; // A=0, B=1, C=2, D=3
        char ant = 'A' + i;
        QHBoxLayout *row = new QHBoxLayout();
        row->addWidget(new QLabel(QString("  %1").arg(ant)));
        m_chkV[idx] = new QCheckBox("V");
        m_chkH[idx] = new QCheckBox("H");
        row->addWidget(m_chkV[idx]);
        row->addWidget(m_chkH[idx]);
        vCol->addLayout(row);
        connect(m_chkV[idx], &QCheckBox::clicked, this, &AntennaPanel::changed);
        connect(m_chkH[idx], &QCheckBox::clicked, this, &AntennaPanel::changed);
    }
    vLayout->addLayout(vCol);
    vLayout->addStretch();
    mainLayout->addLayout(vLayout);

    // Separator
    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    mainLayout->addWidget(line);

    // Horizontal axis: E, F, G, H
    QHBoxLayout *hLayout = new QHBoxLayout();
    hLayout->addStretch();
    QStringList hAnts = {"E", "F", "G", "H"};
    for (int i = 0; i < 4; i++) {
        int idx = 4 + i;
        QVBoxLayout *col = new QVBoxLayout();
        col->addWidget(new QLabel(hAnts[i], this), 0, Qt::AlignCenter);
        m_chkV[idx] = new QCheckBox("V");
        m_chkH[idx] = new QCheckBox("H");
        col->addWidget(m_chkV[idx], 0, Qt::AlignCenter);
        col->addWidget(m_chkH[idx], 0, Qt::AlignCenter);
        hLayout->addLayout(col);
        connect(m_chkV[idx], &QCheckBox::clicked, this, &AntennaPanel::changed);
        connect(m_chkH[idx], &QCheckBox::clicked, this, &AntennaPanel::changed);
    }
    hLayout->addStretch();
    mainLayout->addLayout(hLayout);
}

void AntennaPanel::applyTo(ControlWord &word) const {
    for (int i = 0; i < 8; i++) {
        word.setAntenna('A' + i, 'V', m_chkV[i]->isChecked());
        word.setAntenna('A' + i, 'H', m_chkH[i]->isChecked());
    }
}

void AntennaPanel::loadFrom(const ControlWord &word) {
    for (int i = 0; i < 8; i++) {
        m_chkV[i]->setChecked(word.getAntenna('A' + i, 'V'));
        m_chkH[i]->setChecked(word.getAntenna('A' + i, 'H'));
    }
}
