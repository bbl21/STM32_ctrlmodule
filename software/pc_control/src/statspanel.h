#ifndef STATSPANEL_H
#define STATSPANEL_H

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include "controlword.h"

class StatsPanel : public QWidget {
    Q_OBJECT
public:
    explicit StatsPanel(QWidget *parent = nullptr);

    void updateDisplay(const ControlWord &word);
    void appendLog(const QString &msg);
    void clearLog();
    QPushButton* sendButton() const { return m_sendBtn; }
    QPushButton* saveTemplateButton() const { return m_saveTemplateBtn; }
    // 从 hex 编辑框解析 12 字节，返回 true 表示成功
    bool parseHexBytes(std::array<uint8_t, 12> &out) const;

signals:
    void saveTemplateClicked();

private:
    QTextEdit *m_hexEdit;
    QTextEdit *m_bitEdit;
    QPushButton *m_sendBtn;
    QPushButton *m_saveTemplateBtn;
    QLabel *m_refChLabel;
    QLabel *m_ifLabel1;
    QLabel *m_ifLabel2;
    QLabel *m_ifLabel3;
    QTextEdit *m_log;
};

#endif // STATSPANEL_H
