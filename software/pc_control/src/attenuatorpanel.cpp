#include "attenuatorpanel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QStyle>

static void setBtnState(QPushButton *b, bool on) {
    b->setProperty("state", on ? "on" : "off");
    b->style()->unpolish(b);
    b->style()->polish(b);
}

// Map channel index (0-8) to (module, ifch)
static void chToModuleIfch(int ch, int &mod, int &ifch) {
    if (ch < 3)      { mod = 1; ifch = ch + 1; }
    else if (ch < 6) { mod = 2; ifch = ch - 2; }
    else             { mod = 3; ifch = ch - 5; }
}

AttenuatorPanel::AttenuatorPanel(QWidget *parent)
    : QGroupBox(tr("信号处理"), parent)
{
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(6);
    mainLayout->setContentsMargins(12, 16, 12, 10);

    QStringList labels = {"A", "B", "C", "D", "E", "F", "G", "H", "参考"};

    for (int i = 0; i < 9; i++) {
        QHBoxLayout *row = new QHBoxLayout();
        row->setSpacing(6);

        QLabel *nl = new QLabel(labels[i]);
        nl->setMinimumWidth(28);
        nl->setStyleSheet("font-weight: bold; font-size: 11px;");
        nl->setAlignment(Qt::AlignCenter);

        m_ch[i].btnNarrow = new QPushButton("窄");
        m_ch[i].btnWide = new QPushButton("宽");
        m_ch[i].btnNarrow->setMinimumSize(26, 22);
        m_ch[i].btnWide->setMinimumSize(26, 22);
        m_ch[i].btnNarrow->setCheckable(true);
        m_ch[i].btnWide->setCheckable(true);
        m_ch[i].btnNarrow->setChecked(true);
        setBtnState(m_ch[i].btnNarrow, true);
        setBtnState(m_ch[i].btnWide, false);

        connect(m_ch[i].btnWide, &QPushButton::clicked, this, [this, i]() {
            m_ch[i].btnWide->setChecked(true);
            m_ch[i].btnNarrow->setChecked(false);
            setBtnState(m_ch[i].btnWide, true);
            setBtnState(m_ch[i].btnNarrow, false);
            emit changed();
        });
        connect(m_ch[i].btnNarrow, &QPushButton::clicked, this, [this, i]() {
            m_ch[i].btnNarrow->setChecked(true);
            m_ch[i].btnWide->setChecked(false);
            setBtnState(m_ch[i].btnNarrow, true);
            setBtnState(m_ch[i].btnWide, false);
            emit changed();
        });

        // 所有通道均为完整4bit衰减（0-15dB）
        m_ch[i].slider = new QSlider(Qt::Horizontal);
        m_ch[i].slider->setRange(0, 15);
        m_ch[i].slider->setMinimumWidth(100);
        m_ch[i].slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_ch[i].slider->setTickPosition(QSlider::TicksBelow);
        m_ch[i].slider->setTickInterval(3);

        m_ch[i].spin = new QSpinBox();
        m_ch[i].spin->setRange(0, 15);
        m_ch[i].spin->setSuffix(" dB");
        m_ch[i].spin->setMinimumWidth(64);
        m_ch[i].spin->setAlignment(Qt::AlignCenter);

        connect(m_ch[i].slider, &QSlider::valueChanged, m_ch[i].spin, &QSpinBox::setValue);
        connect(m_ch[i].spin, QOverload<int>::of(&QSpinBox::valueChanged), m_ch[i].slider, &QSlider::setValue);
        connect(m_ch[i].spin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, i](int) {
            emit changed();
        });

        row->addWidget(nl);
        row->addWidget(m_ch[i].btnNarrow);
        row->addWidget(m_ch[i].btnWide);
        row->addSpacing(4);
        row->addWidget(m_ch[i].slider, 1);
        row->addWidget(m_ch[i].spin);
        mainLayout->addLayout(row);
    }

    // 参考通道参选下拉框
    QHBoxLayout *refRow = new QHBoxLayout();
    refRow->addStretch();
    QLabel *refChLabel = new QLabel(tr("参考通道选择"));
    refChLabel->setObjectName("refChLabel");
    refRow->addWidget(refChLabel);
    m_refChannel = new QComboBox();
    m_refChannel->setObjectName("refChannelCombo");
    m_refChannel->addItem("CH1");
    m_refChannel->addItem("CH2");
    m_refChannel->addItem("CH3");
    m_refChannel->addItem("CH4");
    m_refChannel->setMinimumWidth(64);
    refRow->addWidget(m_refChannel);
    refRow->addStretch();
    mainLayout->addLayout(refRow);
    connect(m_refChannel, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &AttenuatorPanel::changed);

    // 分隔线：通道区与全局控制之间
    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    mainLayout->addWidget(line);

    // 第 10 行：全局控制
    {
        QHBoxLayout *row = new QHBoxLayout();
        row->setSpacing(6);

        QLabel *nl = new QLabel("全局");
        nl->setMinimumWidth(28);
        nl->setStyleSheet("font-weight: bold; font-size: 11px; color: #cc9a2a;");
        nl->setAlignment(Qt::AlignCenter);

        m_allNarrow = new QPushButton("窄");
        m_allWide = new QPushButton("宽");
        m_allNarrow->setMinimumSize(26, 22);
        m_allWide->setMinimumSize(26, 22);
        m_allNarrow->setCheckable(true);
        m_allWide->setCheckable(true);
        m_allNarrow->setChecked(true);
        setBtnState(m_allNarrow, true);
        setBtnState(m_allWide, false);

        connect(m_allWide, &QPushButton::clicked, this, [this]() {
            m_allWide->setChecked(true);
            m_allNarrow->setChecked(false);
            setBtnState(m_allWide, true);
            setBtnState(m_allNarrow, false);
            for (int i = 0; i < 9; i++) {
                m_ch[i].btnWide->setChecked(true);
                m_ch[i].btnNarrow->setChecked(false);
                setBtnState(m_ch[i].btnWide, true);
                setBtnState(m_ch[i].btnNarrow, false);
            }
            emit changed();
        });
        connect(m_allNarrow, &QPushButton::clicked, this, [this]() {
            m_allNarrow->setChecked(true);
            m_allWide->setChecked(false);
            setBtnState(m_allNarrow, true);
            setBtnState(m_allWide, false);
            for (int i = 0; i < 9; i++) {
                m_ch[i].btnNarrow->setChecked(true);
                m_ch[i].btnWide->setChecked(false);
                setBtnState(m_ch[i].btnNarrow, true);
                setBtnState(m_ch[i].btnWide, false);
            }
            emit changed();
        });

        m_globalSlider = new QSlider(Qt::Horizontal);
        m_globalSlider->setRange(0, 15);
        m_globalSlider->setMinimumWidth(100);
        m_globalSlider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_globalSlider->setTickPosition(QSlider::TicksBelow);
        m_globalSlider->setTickInterval(3);

        m_globalSpin = new QSpinBox();
        m_globalSpin->setRange(0, 15);
        m_globalSpin->setSuffix(" dB");
        m_globalSpin->setMinimumWidth(64);
        m_globalSpin->setAlignment(Qt::AlignCenter);

        connect(m_globalSlider, &QSlider::valueChanged, m_globalSpin, &QSpinBox::setValue);
        connect(m_globalSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int val) {
            for (int i = 0; i < 9; i++) {
                m_ch[i].spin->setValue(val);
                m_ch[i].slider->setValue(val);
            }
            emit changed();
        });

        row->addWidget(nl);
        row->addWidget(m_allNarrow);
        row->addWidget(m_allWide);
        row->addSpacing(4);
        row->addWidget(m_globalSlider, 1);
        row->addWidget(m_globalSpin);
        mainLayout->addLayout(row);
    }

    // 任一通道变化时自动同步全局控件
    connect(this, &AttenuatorPanel::changed, this, &AttenuatorPanel::syncGlobalFromChannels);
}

void AttenuatorPanel::syncGlobalFromChannels() {
    // 衰减全局控件：9 通道值一致时同步
    int v = m_ch[0].spin->value();
    bool same = true;
    for (int i = 1; i < 9; i++)
        if (m_ch[i].spin->value() != v) { same = false; break; }
    if (same) {
        m_globalSpin->blockSignals(true);
        m_globalSpin->setValue(v);
        m_globalSpin->blockSignals(false);
        m_globalSlider->setValue(v);
    }

    // 带宽全局控件：9 通道一致时同步
    bool w0 = m_ch[0].btnWide->isChecked();
    same = true;
    for (int i = 1; i < 9; i++)
        if (m_ch[i].btnWide->isChecked() != w0) { same = false; break; }
    if (same) {
        m_allWide->setChecked(w0);
        m_allNarrow->setChecked(!w0);
        setBtnState(m_allWide, w0);
        setBtnState(m_allNarrow, !w0);
    }
}

void AttenuatorPanel::setTheme() {
    auto syncBtn = [](QPushButton *w, QPushButton *n) {
        setBtnState(w, w->isChecked());
        setBtnState(n, n->isChecked());
    };
    for (int i = 0; i < 9; i++)
        syncBtn(m_ch[i].btnWide, m_ch[i].btnNarrow);
    syncBtn(m_allWide, m_allNarrow);
}

void AttenuatorPanel::applyTo(ControlWord &word) const {
    for (int i = 0; i < 9; i++) {
        int mod, ifch;
        chToModuleIfch(i, mod, ifch);
        word.setIFAttenuation(mod, ifch, m_ch[i].spin->value());
        bool wide = m_ch[i].btnWide->isChecked();
        word.setIFBand(mod, ifch, wide);
    }
    if (m_refChannel)
        word.setReferenceChannel(m_refChannel->currentIndex() + 1);
}

void AttenuatorPanel::loadFrom(const ControlWord &word) {
    for (int i = 0; i < 9; i++) {
        int mod, ifch;
        chToModuleIfch(i, mod, ifch);
        m_ch[i].spin->setValue(word.getIFAttenuation(mod, ifch));
        bool wide = word.getIFBand(mod, ifch);
        m_ch[i].btnWide->setChecked(wide);
        m_ch[i].btnNarrow->setChecked(!wide);
        setBtnState(m_ch[i].btnWide, wide);
        setBtnState(m_ch[i].btnNarrow, !wide);
    }
    if (m_refChannel)
        m_refChannel->setCurrentIndex(word.getReferenceChannel() - 1);
    syncGlobalFromChannels();
}
