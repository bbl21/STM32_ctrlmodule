#include "statspanel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QDateTime>
#include <QRegularExpression>
#include <QDebug>

StatsPanel::StatsPanel(QWidget *parent)
    : QWidget(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 14, 10, 8);
    layout->setSpacing(8);

    QGridLayout *grid = new QGridLayout();
    grid->setSpacing(6);

    // 96位 Hex（可编辑，1行高）+ 发送按钮
    m_hexEdit = new QTextEdit();
    m_hexEdit->setObjectName("hexDisplay");
    m_hexEdit->setFont(QFont("Consolas", 14));
    m_hexEdit->setFixedHeight(32);
    m_hexEdit->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_hexEdit->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_hexEdit->setLineWrapMode(QTextEdit::NoWrap);

    m_sendBtn = new QPushButton(tr("发送 ▶"));
    m_sendBtn->setObjectName("sendBtn");
    m_sendBtn->setFixedHeight(32);

    // 二进制显示（3行高）
    m_bitEdit = new QTextEdit();
    m_bitEdit->setObjectName("bitDisplay");
    m_bitEdit->setReadOnly(true);
    m_bitEdit->setFont(QFont("Consolas", 14));
    m_bitEdit->setFixedHeight(78);
    m_bitEdit->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_bitEdit->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_bitEdit->setLineWrapMode(QTextEdit::WidgetWidth);

    // 参考通道
    m_refChLabel = new QLabel(tr("--"));
    m_refChLabel->setObjectName("refChannel");
    m_refChLabel->setFont(QFont("Consolas", 14));

    // IF 衰减信息
    m_ifLabel1 = new QLabel(tr("--"));
    m_ifLabel2 = new QLabel(tr("--"));
    m_ifLabel3 = new QLabel(tr("--"));
    QString ifStyle = "font-family: Consolas, monospace; font-size: 14px;";
    m_ifLabel1->setStyleSheet(ifStyle);
    m_ifLabel2->setStyleSheet(ifStyle);
    m_ifLabel3->setStyleSheet(ifStyle);

    QLabel *hexLabel = new QLabel(tr("控制码:"));
    hexLabel->setObjectName("hexLabel");
    m_saveTemplateBtn = new QPushButton(tr("保存为模板"));
    m_saveTemplateBtn->setFixedHeight(28);
    QHBoxLayout *headerRow = new QHBoxLayout();
    headerRow->addWidget(hexLabel);
    headerRow->addStretch();
    headerRow->addWidget(m_saveTemplateBtn);
    grid->addLayout(headerRow, 0, 0, 1, 2);
    QHBoxLayout *hexRow = new QHBoxLayout();
    hexRow->setSpacing(4);
    hexRow->addWidget(m_hexEdit, 1);
    hexRow->addWidget(m_sendBtn);
    grid->addLayout(hexRow, 1, 0, 1, 2);
    QLabel *binLabel = new QLabel(tr("二进制:"));
    binLabel->setObjectName("binLabel");
    grid->addWidget(binLabel, 2, 0, 1, 2);
    grid->addWidget(m_bitEdit, 3, 0, 1, 2);
    QLabel *refLabel = new QLabel(tr("参考通道:"));
    refLabel->setObjectName("refLabel");
    grid->addWidget(refLabel, 4, 0, 1, 2);
    grid->addWidget(m_refChLabel, 5, 0, 1, 2);

    QLabel *ifLabel = new QLabel(tr("IF 衰减:"));
    ifLabel->setObjectName("ifLabel");
    grid->addWidget(ifLabel, 6, 0, 1, 2);
    QVBoxLayout *ifLayout = new QVBoxLayout();
    ifLayout->setSpacing(2);
    ifLayout->setContentsMargins(0, 0, 0, 0);
    ifLayout->addWidget(m_ifLabel1);
    ifLayout->addWidget(m_ifLabel2);
    ifLayout->addWidget(m_ifLabel3);
    grid->addLayout(ifLayout, 7, 0, 1, 2);

    QLabel *logLabel = new QLabel(tr("通信日志:"));
    logLabel->setObjectName("logLabel");
    grid->addWidget(logLabel, 8, 0, 1, 2);
    grid->setRowStretch(9, 1);

    layout->addLayout(grid);

    m_log = new QTextEdit();
    m_log->setObjectName("logDisplay");
    m_log->setReadOnly(true);
    m_log->setFont(QFont("Consolas", 14));
    layout->addWidget(m_log, 1);
}

void StatsPanel::updateDisplay(const ControlWord &word) {
    auto bytes = word.rawBytes();
    QString hex;
    for (auto b : bytes)
        hex += QString("%1 ").arg(b, 2, 16, QChar('0'));
    m_hexEdit->setText(hex.toUpper().trimmed());

    // 二进制：bit 0→95，匹配定义表方向；每32位对应一个 D-sub，分三行
    QString bits;
    for (int dsub = 0; dsub < 3; dsub++) {
        int lineStart = dsub * 32;
        bits += QString("DS%1(b%2-%3): ").arg(dsub + 1).arg(lineStart).arg(lineStart + 31);
        for (int i = 0; i < 32; i++) {
            int bitIdx = lineStart + i;
            bits += (word.getBit(bitIdx) ? '1' : '0');
            if (i % 8 == 7 && i < 31) bits += ' ';
        }
        bits += '\n';
    }
    m_bitEdit->setText(bits.trimmed());

    int ch = word.getReferenceChannel();
    m_refChLabel->setText(tr("通道 %1").arg(ch));

    // 分模块显示 IF 信息
    auto formatModule = [&](int m) -> QString {
        QStringList chs;
        for (int c = 1; c <= 3; c++) {
            int att = word.getIFAttenuation(m, c);
            bool wide = word.getIFBand(m, c);
            chs << QString("IF%1:%2%3").arg(c).arg(att, 2, 10, QChar(' ')).arg(wide ? "W" : "N");
        }
        return QString("M%1: %2").arg(m).arg(chs.join("  "));
    };
    m_ifLabel1->setText(formatModule(1));
    m_ifLabel2->setText(formatModule(2));
    m_ifLabel3->setText(formatModule(3));
}

void StatsPanel::appendLog(const QString &msg) {
    QString ts = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    m_log->append(QString("[%1] %2").arg(ts, msg));
}

void StatsPanel::clearLog() {
    m_log->clear();
}

bool StatsPanel::parseHexBytes(std::array<uint8_t, 12> &out) const {
    QString text = m_hexEdit->toPlainText().trimmed();
    QStringList parts = text.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    if (parts.size() != 12) return false;
    bool ok;
    for (int i = 0; i < 12; i++) {
        uint val = parts[i].toUInt(&ok, 16);
        if (!ok || val > 0xFF) return false;
        out[i] = static_cast<uint8_t>(val);
    }
    return true;
}
