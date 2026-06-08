#include "antennastatuspanel.h"
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QStyle>

static void setBtnState(QPushButton *b, bool on) {
    b->setProperty("state", on ? "on" : "off");
    b->style()->unpolish(b);
    b->style()->polish(b);
}

AntennaStatusPanel::AntennaStatusPanel(QWidget *parent)
    : QGroupBox(tr("天线控制"), parent)
{
    QVBoxLayout *outer = new QVBoxLayout(this);
    outer->setContentsMargins(10, 6, 10, 8);
    outer->setSpacing(6);

    // 连接线颜色使用 accentOrange（由 QSS 控制 #antConnector / #vAntConnector）

    // 9x9 网格：偶数行列放天线块，奇数行列放连接线
    //     c0  c1  c2  c3  c4  c5  c6  c7  c8
    // r0:                  A
    // r1:                  │
    // r2:                  B
    // r3:                  │
    // r4:  E ── F ── ┼ ── G ── H
    // r5:                  │
    // r6:                  C
    // r7:                  │
    // r8:                  D

    QGridLayout *g = new QGridLayout();
    g->setSpacing(0);
    g->setContentsMargins(0, 0, 0, 0);
    g->setColumnStretch(4, 1);
    g->setColumnMinimumWidth(0, 84);
    g->setColumnMinimumWidth(1, 1);
    g->setColumnMinimumWidth(2, 84);
    g->setColumnMinimumWidth(3, 1);
    g->setColumnMinimumWidth(4, 84);
    g->setColumnMinimumWidth(5, 1);
    g->setColumnMinimumWidth(6, 84);
    g->setColumnMinimumWidth(7, 1);
    g->setColumnMinimumWidth(8, 84);
    g->setRowStretch(4, 1);
    g->setRowMinimumHeight(0, 84);
    g->setRowMinimumHeight(1, 1);
    g->setRowMinimumHeight(2, 84);
    g->setRowMinimumHeight(3, 1);
    g->setRowMinimumHeight(4, 84);
    g->setRowMinimumHeight(5, 1);
    g->setRowMinimumHeight(6, 84);
    g->setRowMinimumHeight(7, 1);
    g->setRowMinimumHeight(8, 84);

    // 天线块生成器（九宫格 3x3）
    auto makeBlock = [&](int r, int c, int idx, char name) {
        QWidget *block = new QWidget();
        block->setObjectName("antBlock");
        block->setFixedSize(84, 84);

        bool hasZT = (name == 'B' || name == 'C' || name == 'F' || name == 'G');

        QGridLayout *inner = new QGridLayout(block);
        inner->setSpacing(2);
        inner->setContentsMargins(2, 4, 2, 4);
        inner->setColumnStretch(0, 1);
        inner->setColumnStretch(1, 1);
        inner->setColumnStretch(2, 1);
        inner->setRowStretch(0, 1);
        inner->setRowStretch(1, 1);
        inner->setRowStretch(2, 1);

        // 保证三列等宽
        inner->setColumnMinimumWidth(0, 24);

        // V 上中
        m_ant[idx].btnV = new QPushButton("V");
        m_ant[idx].btnV->setFixedSize(24, 24);
        m_ant[idx].btnV->setStyleSheet("padding:0;border-radius:2px;");
        m_ant[idx].btnV->setCheckable(true);
        m_ant[idx].btnV->setCursor(Qt::PointingHandCursor);
        inner->addWidget(m_ant[idx].btnV, 0, 1, Qt::AlignCenter);

        // 天线名最中间
        m_ant[idx].name = new QLabel(QString("%1").arg(name));
        m_ant[idx].name->setObjectName("antName");
        m_ant[idx].name->setAlignment(Qt::AlignCenter);
        m_ant[idx].name->setFixedSize(22, 22);
        inner->addWidget(m_ant[idx].name, 1, 1, Qt::AlignCenter);

        // H 右中
        m_ant[idx].btnH = new QPushButton("H");
        m_ant[idx].btnH->setFixedSize(24, 24);
        m_ant[idx].btnH->setStyleSheet("padding:0;border-radius:2px;");
        m_ant[idx].btnH->setCheckable(true);
        m_ant[idx].btnH->setCursor(Qt::PointingHandCursor);
        inner->addWidget(m_ant[idx].btnH, 1, 2, Qt::AlignCenter);

        m_vOn[idx] = false; m_hOn[idx] = false; m_ztOn[idx] = false;
        setBtnState(m_ant[idx].btnV, false);
        setBtnState(m_ant[idx].btnH, false);

        int i = idx;
        connect(m_ant[idx].btnV, &QPushButton::clicked, this, [this, i]() {
            m_vOn[i] = m_ant[i].btnV->isChecked();
            if (m_vOn[i]) {
                m_hOn[i] = false;
                m_ant[i].btnH->setChecked(false);
                setBtnState(m_ant[i].btnH, false);
            }
            setBtnState(m_ant[i].btnV, m_vOn[i]);
            emit changed();
        });
        connect(m_ant[idx].btnH, &QPushButton::clicked, this, [this, i]() {
            m_hOn[i] = m_ant[i].btnH->isChecked();
            if (m_hOn[i]) {
                m_vOn[i] = false;
                m_ant[i].btnV->setChecked(false);
                setBtnState(m_ant[i].btnV, false);
            }
            setBtnState(m_ant[i].btnH, m_hOn[i]);
            emit changed();
        });

        // 直通下面三格
        if (hasZT) {
            m_ant[idx].btnZT = new QPushButton("直通");
            m_ant[idx].btnZT->setMinimumSize(48, 20);
            m_ant[idx].btnZT->setCheckable(true);
            m_ant[idx].btnZT->setCursor(Qt::PointingHandCursor);
            m_ant[idx].btnZT->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
            setBtnState(m_ant[idx].btnZT, false);
            int zi = idx;
            connect(m_ant[idx].btnZT, &QPushButton::clicked, this, [this, zi]() {
                m_ztOn[zi] = m_ant[zi].btnZT->isChecked();
                setBtnState(m_ant[zi].btnZT, m_ztOn[zi]);
                emit changed();
            });
            inner->addWidget(m_ant[idx].btnZT, 2, 0, 1, 3, Qt::AlignCenter);
        } else {
            m_ant[idx].btnZT = nullptr;
        }

        g->addWidget(block, r, c, Qt::AlignCenter);
    };

    // 垂直连接线辅助
    auto vln = [&](int r, int c) {
        QWidget *ln = new QWidget();
        ln->setObjectName("vAntConnector");
        g->addWidget(ln, r, c, Qt::AlignCenter);
    };
    // 水平连接线辅助
    auto hln = [&](int r, int c) {
        QWidget *ln = new QWidget();
        ln->setObjectName("hAntConnector");
        g->addWidget(ln, r, c, Qt::AlignCenter);
    };

    // 放天线块
    makeBlock(0, 4, 0, 'A');
    makeBlock(2, 4, 1, 'B');
    makeBlock(4, 0, 4, 'E');
    makeBlock(4, 2, 5, 'F');
    makeBlock(4, 6, 6, 'G');
    makeBlock(4, 8, 7, 'H');
    makeBlock(6, 4, 2, 'C');
    makeBlock(8, 4, 3, 'D');

    // 垂直连接线: A-B, B-中, 中-C, C-D
    vln(1, 4);
    vln(3, 4);
    vln(5, 4);
    vln(7, 4);

    // 水平连接线: E-F, F-中, 中-G, G-H
    hln(4, 1);
    hln(4, 3);
    hln(4, 5);
    hln(4, 7);

    // 中心交叉点
    {
        QWidget *cx = new QWidget();
        cx->setObjectName("cAntConnector");
        g->addWidget(cx, 4, 4, Qt::AlignCenter);
    }

    // 将网格居中
    QHBoxLayout *center = new QHBoxLayout();
    center->setContentsMargins(0, 0, 0, 0);
    center->addStretch();
    center->addLayout(g);
    center->addStretch();
    outer->addStretch();
    outer->addLayout(center);
    outer->addStretch();

    // 快捷按钮行
    QHBoxLayout *br = new QHBoxLayout();
    br->addStretch();
    m_allV = new QPushButton("全V");
    m_allH = new QPushButton("全H");
    m_allZT = new QPushButton("全直通");
    m_allZT->setCheckable(true);
    m_allV->setMinimumWidth(48);
    m_allH->setMinimumWidth(48);
    m_allZT->setMinimumWidth(56);
    setBtnState(m_allV, false);
    setBtnState(m_allH, false);
    setBtnState(m_allZT, false);
    br->addWidget(m_allV);
    br->addWidget(m_allH);
    br->addWidget(m_allZT);
    br->addStretch();
    outer->addLayout(br);

    auto allVOn = [this]() {
        for(int i=0;i<8;i++) if(!m_vOn[i]) return false;
        return true;
    };
    auto allHOn = [this]() {
        for(int i=0;i<8;i++) if(!m_hOn[i]) return false;
        return true;
    };

    connect(m_allV, &QPushButton::clicked, this, [this, allVOn]() {
        bool on = !allVOn();
        for(int i=0;i<8;i++){
            m_vOn[i]=on; m_ant[i].btnV->setChecked(on);
            setBtnState(m_ant[i].btnV, on);
            if (on) {
                m_hOn[i]=false; m_ant[i].btnH->setChecked(false);
                setBtnState(m_ant[i].btnH, false);
            }
        }
        setBtnState(m_allV, on);
        if (on) setBtnState(m_allH, false);
        emit changed();
    });
    connect(m_allH, &QPushButton::clicked, this, [this, allHOn]() {
        bool on = !allHOn();
        for(int i=0;i<8;i++){
            m_hOn[i]=on; m_ant[i].btnH->setChecked(on);
            setBtnState(m_ant[i].btnH, on);
            if (on) {
                m_vOn[i]=false; m_ant[i].btnV->setChecked(false);
                setBtnState(m_ant[i].btnV, false);
            }
        }
        setBtnState(m_allH, on);
        if (on) setBtnState(m_allV, false);
        emit changed();
    });
    connect(m_allZT, &QPushButton::clicked, this, [this]() {
        bool on = m_allZT->isChecked();
        for(int i=0;i<8;i++) if(m_ant[i].btnZT){
            m_ztOn[i]=on; m_ant[i].btnZT->setChecked(on);
            setBtnState(m_ant[i].btnZT, on);}
        setBtnState(m_allZT, on);
        emit changed();
    });
}

void AntennaStatusPanel::setTheme() {
    for (int i = 0; i < 8; i++) {
        setBtnState(m_ant[i].btnV, m_vOn[i]);
        setBtnState(m_ant[i].btnH, m_hOn[i]);
        if (m_ant[i].btnZT) setBtnState(m_ant[i].btnZT, m_ztOn[i]);
    }
    bool allV=true, allH=true, allZT=true;
    for(int i=0;i<8;i++){
        if(!m_vOn[i]) allV=false;
        if(!m_hOn[i]) allH=false;
        if(m_ant[i].btnZT && !m_ztOn[i]) allZT=false;
    }
    setBtnState(m_allV, allV);
    setBtnState(m_allH, allH);
    setBtnState(m_allZT, allZT);
}

void AntennaStatusPanel::applyTo(ControlWord &word) const {
    for (int i = 0; i < 8; i++) {
        word.setAntenna('A'+i, 'V', m_vOn[i]);
        word.setAntenna('A'+i, 'H', m_hOn[i]);
        if (m_ant[i].btnZT) word.setDirectThrough('A'+i, m_ztOn[i]);
    }
}

void AntennaStatusPanel::loadFrom(const ControlWord &word) {
    for (int i = 0; i < 8; i++) {
        m_vOn[i] = word.getAntenna('A'+i, 'V');
        m_hOn[i] = word.getAntenna('A'+i, 'H');
        m_ant[i].btnV->setChecked(m_vOn[i]);
        m_ant[i].btnH->setChecked(m_hOn[i]);
        setBtnState(m_ant[i].btnV, m_vOn[i]);
        setBtnState(m_ant[i].btnH, m_hOn[i]);
        if (m_ant[i].btnZT) {
            m_ztOn[i] = word.getDirectThrough('A'+i);
            m_ant[i].btnZT->setChecked(m_ztOn[i]);
            setBtnState(m_ant[i].btnZT, m_ztOn[i]);
        }
    }
    bool allV=true, allH=true, allZT=true;
    for(int i=0;i<8;i++){
        if(!m_vOn[i]) allV=false;
        if(!m_hOn[i]) allH=false;
        if(m_ant[i].btnZT && !m_ztOn[i]) allZT=false;
    }
    setBtnState(m_allV, allV);
    setBtnState(m_allH, allH);
    setBtnState(m_allZT, allZT);
}
