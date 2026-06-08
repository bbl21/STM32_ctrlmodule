#ifndef CDCCOMM_H
#define CDCCOMM_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>

class CdccComm : public QObject {
    Q_OBJECT
public:
    explicit CdccComm(QObject *parent = nullptr);
    ~CdccComm();

    // 枚举可用 COM 口
    static QStringList availablePorts();

    // 连接/断开
    bool open(const QString &portName, int baud = 115200);
    void close();
    bool isOpen() const;
    QString currentPort() const { return m_currentPort; }

    // 发送 12 字节模板码
    bool sendTemplate(const std::array<uint8_t, 12> &data);

    // 发送并等待回复 (超时 ms)
    QString sendAndReceive(const std::array<uint8_t, 12> &data, int timeoutMs = 100);

signals:
    void connected();
    void disconnected();
    void dataReceived(const QByteArray &data);
    void errorOccurred(const QString &error);

private slots:
    void onReadyRead();
    void onErrorOccurred(QSerialPort::SerialPortError error);

private:
    QSerialPort *m_serial;
    QString m_currentPort;
    QByteArray m_rxBuffer;
};

#endif // CDCCOMM_H
